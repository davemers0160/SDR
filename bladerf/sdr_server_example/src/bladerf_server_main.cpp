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
//--------------------------- GLOBAL VARIABLES --------------------------------
//const double pi = 3.14159265358979323846;
//const double pi2 = 2 * 3.14159265358979323846;

bool is_running = false;
bool transmit = false;
bool recieve = false;
uint32_t blade_timeout_ms = 10000;


//-----------------------------------------------------------------------------
void sig_handler(int signo)
{
    if (signo == SIGINT) 
    {
        fprintf(stderr, "received SIGINT\n");
        is_running = false;
        transmit = false;
        recieve = false;
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
    int32_t bladerf_num;
    int32_t blade_status;
    uint32_t blade_mode = 1;        // RX --> 0, TX --> 1
    
    const uint32_t num_buffers = 1024;
    const uint32_t buffer_size = 1024 * 4;        // must be a multiple of 1024
    const uint32_t num_transfers = 128;


    // TX parameters
    bladerf_sample_rate tx_sample_rate = 40000000;
    bladerf_channel tx = BLADERF_CHANNEL_TX(0);
    bladerf_bandwidth tx_bw = 800000;
    bladerf_gain tx_gain = 60;
    bladerf_frequency tx_start_freq = 1500000000;
    bladerf_frequency tx_stop_freq = 1500000000;
    int64_t tx_step = 30000000;
    bool tx_enable = true;
    std::vector<hop_params> tx_hops;
    std::vector<bladerf_frequency> tx_hop_sequence;
    std::thread tx_thread;

    // RX parameters
    bladerf_sample_rate rx_sample_rate = 40000000;
    bladerf_channel rx = BLADERF_CHANNEL_RX(0);
    bladerf_bandwidth rx_bw = 800000;
    bladerf_gain rx_gain = 60;
    bladerf_frequency rx_start_freq = 1500000000;
    bladerf_frequency rx_stop_freq = 1500000000;
    int64_t rx_step = 30000000;
    bool rx_enable = false;
    std::vector<hop_params> rx_hops;
    std::vector<bladerf_frequency> rx_hop_sequence;

    bladerf_frequency tuned_freq;

    double on_time;
    double off_time;
    uint64_t num_samples;
    uint32_t hop_index = 0;
    uint16_t hop_type;
    bool tmp_enable;

    std::vector<std::complex<int16_t>> samples;
    std::string iq_filename;

    
    if (argc < 2)
    {
        std::cout << "supply a file name for the IQ data." << std::endl;
        std::cin.ignore();
        return -1;
    }

    // read in the parameters
    std::string param_filename = argv[1];
    //read_hop_params(param_filename, start_freq, stop_freq, hop_step, sample_rate, tx_bw, hop_type, on_time, off_time, tx1_gain, iq_filename);

    generate_range(tx_start_freq, tx_stop_freq, (double)tx_step, tx_hop_sequence);
    uint32_t num_tx_hops = tx_hop_sequence.size();

    generate_range(rx_start_freq, rx_stop_freq, (double)rx_step, rx_hop_sequence);
    uint32_t num_rx_hops = rx_hop_sequence.size();

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
    double sample_duration = (num_samples) / (double)tx_sample_rate;

    //-----------------------------------------------------------------------------
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

        std::cout << "number of TX hops: " << num_tx_hops << std::endl << std::endl;

        //for (idx = 0; idx < num_tx_hops; ++idx)
        //{

        //    hop_params p;
        //    p.f = tx_hop_sequence[idx];

        //    tx_hops.push_back(p);

        //    blade_status = bladerf_set_frequency(dev, tx, hops[idx].f);
        //    if (blade_status != 0)
        //    {
        //        std::cout << "Failed to set frequency: " << hops[idx].f << "Hz.   error: " << std::string(bladerf_strerror(blade_status)) << std::endl;
        //        std::cin.ignore();
        //    }

        //    //blade_status = bladerf_get_quick_tune(dev, tx, &hops[idx].qt);
        //    //if (blade_status != 0)
        //    //{
        //    //    std::cout << "Failed to get quick tune: " << hops[idx].f << "Hz.   error: " << std::string(bladerf_strerror(blade_status)) << std::endl;
        //    //    std::cin.ignore();
        //    //}

        //    std::cout << idx << ": " << hops[idx].f << ", nios_profile: " << hops[idx].qt.nios_profile << ", rffe_profile: " << (uint16_t)hops[idx].qt.rffe_profile << std::endl;
        //}

        std::cout << "number of RX hops: " << num_rx_hops << std::endl << std::endl;

        //-----------------------------------------------------------------------------
        // configure the sync to receive/transmit data
        blade_status = bladerf_sync_config(dev, BLADERF_TX_X1, BLADERF_FORMAT_SC16_Q11, num_buffers, buffer_size, num_transfers, blade_timeout_ms);
        if (blade_status != 0)
        {
            std::cout << "Failed to configure TX sync interface - error: " << std::string(bladerf_strerror(blade_status)) << std::endl;
        }
        
        blade_status = bladerf_sync_config(dev, BLADERF_RX_X1, BLADERF_FORMAT_SC16_Q11, num_buffers, buffer_size, num_transfers, blade_timeout_ms);
        if (blade_status != 0)
        {
            std::cout << "Failed to configure RX sync interface - error: " << std::string(bladerf_strerror(blade_status)) << std::endl;
        }

        blade_status = bladerf_set_gain_mode(dev, rx, BLADERF_GAIN_MGC);

        // config the tx side
        blade_status = config_blade_channel(dev, tx, tx_hop_sequence[0], tx_sample_rate, tx_bw, tx_gain);
        // config the rx side
        blade_status = config_blade_channel(dev, rx, rx_hop_sequence[0], rx_sample_rate, rx_bw, rx_gain);

        if(blade_mode == 0)
            blade_status = bladerf_enable_module(dev, rx, rx_enable);
        else
            blade_status = bladerf_enable_module(dev, tx, tx_enable);

        if (blade_status != 0)
        {
            std::cout << "Error enabling channel: " << std::string(bladerf_strerror(blade_status)) << std::endl;
        }

        //// set initial frequency
        //switch (hop_type)
        //{
        //case 0:
        //    hop_index = 0;
        //    break;
        //case 1:
        //    hop_index = (uint32_t)(rand() % num_tx_hops);
        //    break;
        //default:
        //    hop_index = 0;
        //    break;
        //}

        if (signal(SIGINT, sig_handler) == SIG_ERR) 
        {
            std::cerr << "Unable to catch SIGINT signals" << std::endl;
        }

        is_running = true;
        

        // start the tx thread
        tx_thread  = std::thread(transmit_thread, dev, std::ref(samples));

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
            // template for all methods
            //case static_cast<uint32_t>(BLADE_MSG_ID::XXXXXXXX):
            //    msg_result.resize(2);
            //    msg_result[0] = static_cast<uint32_t>(BLADE_MSG_ID::XXXXXXXX);
            //    msg_result[1] = blade_status;
            //    break;

            case static_cast<uint32_t>(BLADE_MSG_ID::GET_VERSION):
                msg_result.resize(4);
                msg_result[0] = static_cast<uint32_t>(BLADE_MSG_ID::GET_VERSION);
                msg_result[1] = MAJOR_REVISION;
                msg_result[2] = MINOR_REVISION;
                msg_result[3] = FIX_REVISION;
                break;

            case static_cast<uint32_t>(BLADE_MSG_ID::SELECT_MODE):
                blade_mode = command[1];
                if (blade_mode == 0)
                {
                    // stop transmitting
                    transmit = false;
                    tx_thread.join();

                    tuned_freq = rx_hop_sequence[hop_index];
                    blade_status = switch_blade_mode(dev, blade_mode, rx);
                    blade_status |= bladerf_set_frequency(dev, rx, tuned_freq);

                    recieve = true;
                }
                else
                {
                    // stop recieving
                    recieve = false;
                    tuned_freq = tx_hop_sequence[hop_index];
                    blade_status = switch_blade_mode(dev, blade_mode, tx);
                    blade_status |= bladerf_set_frequency(dev, tx, tuned_freq);

                    // start the tx thread
                    transmit = true;
                    tx_thread = std::thread(transmit_thread, dev, std::ref(samples));
                }
                msg_result.resize(2);
                msg_result[0] = static_cast<uint32_t>(BLADE_MSG_ID::SELECT_MODE);
                msg_result[1] = blade_status;
                break;

            case static_cast<uint32_t>(BLADE_MSG_ID::CONFIG_TX):
                tx_start_freq = ((uint64_t)command[CONFIG_START_FREQ_MSB_INDEX] << 32) | command[CONFIG_START_FREQ_LSB_INDEX];
                tx_stop_freq = ((uint64_t)command[CONFIG_STOP_FREQ_MSB_INDEX] << 32) | command[CONFIG_STOP_FREQ_LSB_INDEX];
                tx_step = command[CONFIG_FREQ_STEP_INDEX];
                tx_sample_rate = command[CONFIG_SAMPLERATE_INDEX];
                tx_bw = command[CONFIG_BANDWIDTH_INDEX];
                tx_gain = command[CONFIG_GAIN_INDEX];

                generate_range(tx_start_freq, tx_stop_freq, (double)tx_step, tx_hop_sequence);
                num_tx_hops = tx_hop_sequence.size();

                blade_status = config_blade_channel(dev, tx, tx_hop_sequence[0], tx_sample_rate, tx_bw, tx_gain);
                if (blade_status != 0)
                {
                    std::cout << "Error configuring channel: " << std::string(bladerf_strerror(blade_status)) << std::endl;
                }

                msg_result.resize(2);
                msg_result[0] = static_cast<uint32_t>(BLADE_MSG_ID::CONFIG_TX);
                msg_result[1] = blade_status;
                break;

            case static_cast<uint32_t>(BLADE_MSG_ID::ENABLE_TX):
                // requires that rx be disables before Tx is enabled
                tmp_enable = (bool)command[1];

                enable_channel(dev, tx, tmp_enable, tx_enable);

                if (tx_enable == true && transmit == false)
                {
                    transmit = true;
                    tx_thread = std::thread(transmit_thread, dev, std::ref(samples));
                }
                else if (tx_enable == false && transmit == true)
                {
                    transmit = false;
                    tx_thread.join();
                }

                msg_result.resize(2);
                msg_result[0] = static_cast<uint32_t>(BLADE_MSG_ID::ENABLE_TX);
                msg_result[1] = 1;
                break;

            //case static_cast<uint32_t>(BLADE_MSG_ID::SET_TX_FREQ):
            //    start_freq = ((uint64_t)command[1]) << 32 | command[2];
            //    blade_status = bladerf_set_frequency(dev, tx, hops[hop_index].f);
            //    msg_result.resize(2);
            //    msg_result[0] = static_cast<uint32_t>(BLADE_MSG_ID::SET_TX_FREQ);
            //    msg_result[1] = blade_status;
            //    break;
            //           
            //case static_cast<uint32_t>(BLADE_MSG_ID::TRANSMIT_SAMPLES):
            //    if(command[1] == 0)
            //        transmit = false;
            //    else if(command[1] == 1)
            //        transmit = true;

            //    msg_result.resize(2);
            //    msg_result[0] = static_cast<uint32_t>(BLADE_MSG_ID::SET_TX_FREQ);
            //    msg_result[1] = (uint32_t)transmit;
            //    break;

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
        recieve = false;
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
