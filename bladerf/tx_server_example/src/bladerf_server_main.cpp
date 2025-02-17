// ----------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS
#include <ryml_all.hpp>

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)

#elif defined(__linux__)

#endif
// ArrayFire Includes
//#include <arrayfire.h>

#include <cstdint>
#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <complex>
#include <csignal>
#include <chrono>
#include <thread>

// ZMQ includes
#include <zmq.hpp>
#include <zmq_addon.hpp>

// bladeRF includes
#include <libbladeRF.h>
#include <bladeRF2.h>

// Custom Includes
#include "num2string.h"
#include "get_current_time.h"
#include "file_parser.h"
#include "sleep_ms.h"
#include "iq_utils.h"

// Project Includes
#include "bladerf_common.h"
#include "parse_hop_input.h"
#include "bladerf_server_commands.h"


//-----------------------------------------------------------------------------
// Globals
const double pi = 3.14159265358979323846;
const double pi2 = 2 * 3.14159265358979323846;

//bladerf_frequency hop_step = 25000;
//const bladerf_frequency start_freq = 47000000;
//const bladerf_frequency stop_freq = start_freq + 5000000;
bool is_running = false;
bool transmit = false;
uint32_t blade_timeout_ms = 10000;


//-----------------------------------------------------------------------------
void sig_handler(int signo)
{
    if (signo == SIGINT) 
    {
        fprintf(stderr, "received SIGINT\n");
        is_running = false;
        transmit = false;
    }
}   // end of sig_handler

//-----------------------------------------------------------------------------
inline void transmit_thread(struct bladerf* dev, std::vector<std::complex<int16_t>>& samples)
{
    uint32_t num_samples = samples.size();
    int32_t blade_status;
    
    while(transmit == true)
    {
        blade_status = bladerf_sync_tx(dev, (int16_t*)samples.data(), num_samples, NULL, blade_timeout_ms);

        if (blade_status != 0)
        {
            std::cout << "Unable to send the required number of samples: " << std::string(bladerf_strerror(blade_status)) << std::endl;
        }
    }
    
}   // end of transmit_thread


//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{

    uint32_t idx, jdx;
    
    // timing variables
    auto start_time = std::chrono::high_resolution_clock::now();
    auto stop_time = std::chrono::high_resolution_clock::now();
    double duration = std::chrono::duration_cast<chrono::nanoseconds>(stop_time - start_time).count();

    // bladeRF variable
    struct bladerf_devinfo *device_list = NULL;
    struct bladerf_devinfo dev_info;
    struct bladerf* dev;
    int bladerf_num;
    int blade_status;
    bladerf_sample_rate sample_rate = 30e6;     // 10 MHz
    bladerf_channel tx = BLADERF_CHANNEL_TX(0);
    bladerf_frequency start_freq = 1500000000;// 314300000;
    bladerf_frequency stop_freq = 1500000000;// 314300000;
    int64_t hop_step = 500000000;// 314300000;
    bladerf_bandwidth tx_bw = 200000;
    bladerf_gain tx1_gain = 60000;

    double on_time;
    double off_time;

    uint64_t num_samples;
    const uint32_t num_buffers = 1024;
    const uint32_t buffer_size = 1024*4;        // must be a multiple of 1024
    const uint32_t num_transfers = 128;
    uint32_t hop_index;
    uint16_t hop_type;

    std::vector<std::complex<int16_t>> samples;
    std::string iq_filename;

    std::vector<hop_params> hops;
    
    if (argc < 2)
    {
        std::cout << "supply a file name for the IQ data." << std::endl;
        std::cin.ignore();
        return -1;
    }

    // read in the parameters
    std::string param_filename = argv[1];
    read_hop_params(param_filename, start_freq, stop_freq, hop_step, sample_rate, tx_bw, hop_type, on_time, off_time, tx1_gain, iq_filename);

    std::vector<bladerf_frequency> hop_sequence;
    generate_range(start_freq, stop_freq, (double)hop_step, hop_sequence);
    uint32_t num_hops = hop_sequence.size();

    // initialize a random number generator
    srand((unsigned)time(NULL));

    //-----------------------------------------------------------------------------
    // ZMQ message containers
    // zmq::message_t to hold the received messages
    zmq::message_t command_messages;
    std::optional<uint32_t> num_messages = std::nullopt;

    // vector of bytes to store the data portion of the multipart message
    std::vector<uint32_t> command;

    // vector to store and send the response back to the client
    std::vector<uint32_t> msg_result;

    std::cout << "Starting BladeRF SDR Server..." << std::endl;
    zmq::context_t bladerf_context{ 1 };
    zmq::socket_t bladerf_socket = create_server_connection(BLADERF_IP_ADDRESS, BLADERF_PORT, bladerf_context, static_cast<int32_t>(zmq::socket_type::rep));
    std::cout << "BladeRF SDR Server Ready!" << std::endl << std::endl;


    //-----------------------------------------------------------------------------
    // read in the data
    read_iq_data(iq_filename, samples);

    // the number of IQ samples is the number of samples divided by 2
    num_samples = samples.size();
    double sample_duration = (num_samples) / (double)sample_rate;

    // ----------------------------------------------------------------------------
    try 
    {

        int num_devices = bladerf_get_device_list(&device_list);

        bladerf_num = select_bladerf(num_devices, device_list);

        if (bladerf_num < 0)
        {
            std::cout << "could not detect any bladeRF devices..." << std::endl;
            return 0;
        }

        std::cout << std::endl;

        blade_status = bladerf_open(&dev, ("*:serial=" + std::string(device_list[bladerf_num].serial)).c_str());
        if (blade_status != 0)
        {
            std::cout << "Unable to open device: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            return blade_status;
        }

        blade_status = bladerf_get_devinfo(dev, &dev_info);
        if (blade_status != 0)
        {
            std::cout << "Unable to get the device info: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            return blade_status;
        }
        std::cout << dev_info << std::endl;

        std::cout << "number of hops: " << num_hops << std::endl << std::endl;

        for (idx = 0; idx < num_hops; ++idx)
        {

            hop_params p;
            p.f = hop_sequence[idx];

            hops.push_back(p);

            blade_status = bladerf_set_frequency(dev, tx, hops[idx].f);
            if (blade_status != 0)
            {
                std::cout << "Failed to set frequency: " << hops[idx].f << "Hz.   error: " << std::string(bladerf_strerror(blade_status)) << std::endl;
                std::cin.ignore();
            }

            //blade_status = bladerf_get_quick_tune(dev, tx, &hops[idx].qt);
            //if (blade_status != 0)
            //{
            //    std::cout << "Failed to get quick tune: " << hops[idx].f << "Hz.   error: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            //    std::cin.ignore();
            //}

            std::cout << idx << ": " << hops[idx].f << ", nios_profile: " << hops[idx].qt.nios_profile << ", rffe_profile: " << (uint16_t)hops[idx].qt.rffe_profile << std::endl;

        }

        // set the sample_rate and bandwidth
        blade_status = bladerf_set_sample_rate(dev, tx, sample_rate, &sample_rate);
        blade_status = bladerf_set_bandwidth(dev, tx, tx_bw, &tx_bw);

        // configure the sync to receive/transmit data
        blade_status = bladerf_sync_config(dev, BLADERF_TX_X1, BLADERF_FORMAT_SC16_Q11, num_buffers, buffer_size, num_transfers, blade_timeout_ms);

        if (blade_status != 0)
        {
            std::cout << "Failed to configure TX sync interface - error: " << std::string(bladerf_strerror(blade_status)) << std::endl;
        }

        // enable the TX channel RF frontend
        blade_status = bladerf_enable_module(dev, BLADERF_TX, true);
        if (blade_status != 0)
        {
            std::cout << "Error enabling TX - error: " << std::string(bladerf_strerror(blade_status)) << std::endl;
        }

        // the gain must be set after the module has been enabled
        //blade_status = bladerf_set_gain_mode(dev, tx, BLADERF_GAIN_MANUAL);
        blade_status = bladerf_set_gain(dev, tx, tx1_gain);
        blade_status = bladerf_get_gain(dev, tx, &tx1_gain);
        if (blade_status != 0)
        {
            std::cout << "Error setting TX gain - error: " << std::string(bladerf_strerror(blade_status)) << std::endl;
        }

        // set initial frequency
        switch (hop_type)
        {
        case 0:
            hop_index = 0;
            break;
        case 1:
            hop_index = (uint32_t)(rand() % num_hops);
            break;
        default:
            hop_index = 0;
            break;
        }

        blade_status = bladerf_set_frequency(dev, tx, hops[hop_index].f);

        is_running = true;

        if (signal(SIGINT, sig_handler) == SIG_ERR) 
        {
            std::cerr << "Unable to catch SIGINT signals" << std::endl;
        }
        

        // start the rx thread
        std::thread tx_thread(transmit_thread, dev, std::ref(samples));

        //std::cout << std::endl << "Sending signals..." << std::endl << std::endl;


        // main loop
        while (is_running)
        {

            // non-blocking call to receive multiple messages
            //num_messages = zmq::recv_multipart(sb_socket, std::back_inserter(command_messages), zmq::recv_flags::dontwait);
            num_messages = bladerf_socket.recv(command_messages, zmq::recv_flags::dontwait);
            

            // check to see if any messages have arrived
            if (num_messages.has_value() == false)
            {
                //std::cout << ".";
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            
            // clear out the containers if we receive a new message
            command.clear();
            
            // parse the first part of the message to get the command
            std::cout << "Received message with " << num_messages.value() << " messages" << std::endl;
            
            // place the command messages into the command vector
            command.resize(command_messages.size() / sizeof(uint32_t));
            std::memcpy(command.data(), command_messages.data(), command_messages.size());
            std::cout << "message command id: " << num2str(command[0], "0x%08X") << std::endl;
                
            // check the command value and perform some action
            switch (command[0])
            {

            case static_cast<uint32_t>(BLADE_MSG_ID::SET_TX_FREQ):
                start_freq = ((uint64_t)command[1]) << 32 | command[2];
                blade_status = bladerf_set_frequency(dev, tx, hops[hop_index].f);
                msg_result.resize(2);
                msg_result[0] = static_cast<uint32_t>(BLADE_MSG_ID::SET_TX_FREQ);
                msg_result[1] = blade_status;
                break;
                       
            case static_cast<uint32_t>(BLADE_MSG_ID::TRANSMIT_SAMPLES):
                if(command[1] == 0)
                    transmit = false;
                else if(command[1] == 1)
                    transmit = true;

                msg_result.resize(2);
                msg_result[0] = static_cast<uint32_t>(BLADE_MSG_ID::SET_TX_FREQ);
                msg_result[1] = (uint32_t)transmit;
                break;

            default:
                msg_result.resize(1);
                msg_result[0] = static_cast<uint32_t>(BLADE_MSG_ID::UNKNOWN);  // command[0]
                break;
            }   // end of switch (command[0])
            
            
            // send the reply to the client
            bladerf_socket.send(zmq::buffer(msg_result), zmq::send_flags::none);

            // little bit of sleep time
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        transmit = false;
        tx_thread.join();

        std::cout << "Closing BladeRF SDR Server..." << std::endl;


        // disable the tx channel RF frontend
        blade_status = bladerf_enable_module(dev, BLADERF_TX, false);

    }
    catch (std::exception e)
    {
        std::cout << "error: " << e.what() << std::endl;
    }

    bladerf_close(dev);

    std::cout << "Press Enter to close..." << std::endl;
    //std::cin.ignore();    
    
    return 0;
    
}   // end of main
