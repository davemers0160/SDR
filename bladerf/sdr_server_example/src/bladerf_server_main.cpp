// ----------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS
#if defined(USE_RAPIDYAML)

#include <ryml_all.hpp>
#endif

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
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <filesystem>

// ZMQ includes
#include <zmq.hpp>
#include <zmq_addon.hpp>

// bladeRF includes
#include <libbladeRF.h>
#include <bladeRF2.h>

#if defined(WITH_RPI)
// libgpiod header
#include <gpiod.hpp>
#endif

// Custom Includes
#include "num2string.h"
#include "get_current_time.h"
#include "file_parser.h"
#include "sleep_ms.h"
#include "iq_utils.h"
#include "dsp/dsp_windows.h"
#include "data_logger.h"
#include "file_ops.h"

// Project Includes
#include "bladerf_common.h"
#include "parse_hop_input.h"
#include "bladerf_server_commands.h"

//-----------------------------------------------------------------------------
//--------------------------- GLOBAL VARIABLES --------------------------------
//const double pi = 3.14159265358979323846;
//const double pi2 = 2 * 3.14159265358979323846;

atomic<bool> is_running = false;

atomic<bool> transmit_thread_running = false;
atomic<bool> transmit = false;
atomic<uint32_t> num_tx_samples;
std::mutex tx_mutex;
std::condition_variable tx_cv;

atomic<bool> scan = false;
std::mutex scan_mutex;
std::condition_variable scan_cv;

atomic<bool> recieve_thread_running = false;
atomic<bool> recieve = false;
std::mutex rx_mutex;
std::condition_variable rx_cv;

uint32_t blade_timeout_ms = 10000;

#if defined(WITH_RPI)
const gpiod::line::value gpio_on = gpiod::line::value::ACTIVE;
const gpiod::line::value gpio_off = gpiod::line::value::INACTIVE;
const gpiod::line::offset rf_ctrl_pin = 20;
#endif

std::vector<hop_parameters> tx_hops;
std::vector<hop_parameters> rx_hops;
atomic<uint32_t> num_tx_hops;
atomic<uint32_t> num_rx_hops;
atomic<uint16_t> tx_hop_type(1);

//-----------------------------------------------------------------------------
void sig_handler(int sig_num)
{
    if ((sig_num == SIGINT) | (sig_num == SIGTERM))
    {
        //fprintf(stderr, "received SIGINT: %d\n", sig_num);
        std::cout << info << "Received SIGINT: " << sig_num << std::endl;
        is_running = false;
        transmit_thread_running = false;
        recieve_thread_running = false;
        transmit = false;
        recieve = false;
        scan = false;
    }

}   // end of sig_handler

//-----------------------------------------------------------------------------
inline void publisher_thread(zmq::socket_t &pub_socket) //zmq::context_t& context)
{
    uint8_t index = 0;
    //zmq::socket_t bladerf_pub_socket = create_server_connection(BLADERF_IP_ADDRESS, BLADERF_STATUS_PORT, context, static_cast<int32_t>(zmq::socket_type::pub));
    
    std::cout << info << "Starting Publisher Thread!" << std::endl << std::endl;

    while(is_running == true)
    {
        std::string message = num2str(++index, "0x%02X");
        zmq::message_t zmq_message(message.size());
        memcpy(zmq_message.data(), message.c_str(), message.size());
        pub_socket.send(zmq_message, zmq::send_flags::none);
        
        //std::cout << "Sent: " << message << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    std::cout << info << "Publisher thread stopped." << std::endl;

}   // end of publisher_thread

//-----------------------------------------------------------------------------
inline void transmit_thread(struct bladerf* dev, bladerf_channel tx, std::vector<std::complex<int16_t>>& samples)
{
    int32_t blade_status = 0;
    uint32_t hop_index = 0;
    uint8_t rffe_index = 0;
	
    std::cout << info << "Transmit thread started." << std::endl;

    // main thread loop
    while (transmit_thread_running == true)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(10));

        // main transmit loop
        while (transmit == true)
        {    
            //std::cout << "ns1: " << num_tx_samples << " ns2: " << samples.size() << std::endl;
            blade_status = bladerf_sync_tx(dev, (int16_t*)samples.data(), num_tx_samples, NULL, blade_timeout_ms);

            if (blade_status != 0)
            {
                std::cout << warning << "Unable to send the required number of samples: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            }

            if(num_tx_hops > 1)
			{
                // select the hop type
			    switch (tx_hop_type)
			    {
			    case 0:
                   hop_index = (uint32_t)((hop_index+1) % num_tx_hops);
			       break;
			    case 1:
			       hop_index = (uint32_t)(rand() % num_tx_hops);
			       break;
			    default:
			       hop_index = 0;
			       break;
			    }

                //std::cout << "hop_index: " << hop_index << std::endl;

#if defined(WITH_FASTTUNE)
                tx_hops[hop_index].qt_params.rffe_profile = rffe_index;
                rffe_index = (rffe_index + 1) & 0x07;
                blade_status = bladerf_schedule_retune(dev, tx, BLADERF_RETUNE_NOW, tx_hops[hop_index].freq, &tx_hops[hop_index].qt_params);
#else
                blade_status = bladerf_set_frequency(dev, tx, tx_hops[hop_index].freq);
#endif	
                if (blade_status != 0)
                {
                    std::cout << warning << "Unable to set the center frequency: " << std::string(bladerf_strerror(blade_status)) << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::microseconds(10));

            }
        }

        // this wait will put the thread to sleep until transmit is true
        {
            std::unique_lock<std::mutex> lock(tx_mutex);
            tx_cv.wait(lock, [] { return (transmit == false); });
        }

    }

    std::cout << info << "Transmit thread stopped." << std::endl;

}   // end of transmit_thread

//-----------------------------------------------------------------------------
inline void scan_thread(struct bladerf* dev, bladerf_channel tx, double &scan_time, bladerf_gain &gain)
{
    int32_t blade_status = 0;
    int32_t scan_taps = 51;
    bladerf_gain g = 0;
    uint32_t index = 0;

    std::vector<float> scan_steps = DSP::blackman_harris_window(scan_taps);
    
    // calculate the number of nano seconds delay for each step
    uint64_t time_per_step = std::floor((scan_time / (double)scan_taps - 1.0e-7) * 1e9);

    std::cout << info << "Scan thread started." << std::endl;

    // main thread loop
    while (transmit_thread_running == true)
    {

        std::this_thread::sleep_for(std::chrono::microseconds(10));

        // main transmit loop
        while (scan == true)
        {
            g = std::ceil(gain * scan_steps[index++]);
            blade_status = bladerf_set_gain(dev, tx, g);

            if (blade_status != 0)
            {
                std::cout << warning << "Unable to set the gain value: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            }

            //std::cout << "gain: " << g << std::endl;
            if(index >= scan_taps)
                index = 0;

            time_per_step = std::floor((scan_time / (double)scan_taps - 1.0e-7) * 1e9);
            std::this_thread::sleep_for(std::chrono::nanoseconds(time_per_step));
        }

        // this wait will put the thread to sleep until transmit is true
        {
            std::unique_lock<std::mutex> lock(scan_mutex);
            scan_cv.wait(lock, [] { return (scan == false); });
        }
    }

    std::cout << info << "Scan thread stopped." << std::endl;

}   // end of scan_thread

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
    bladerf_sample_rate tx_sample_rate = 20000000;
    bladerf_channel tx = BLADERF_CHANNEL_TX(0);
    bladerf_bandwidth tx_bw = 10000000;
    bladerf_gain tx_gain = 66;
    bladerf_frequency tx_start_freq = 2000000000;
    bladerf_frequency tx_stop_freq = 2000000000;
    int64_t tx_step = 2000000;
    //bool tx_enable = true;
    //std::vector<bladerf_frequency> tx_hop_sequence;
    std::thread tx_thread;

    // RX parameters
    bladerf_sample_rate rx_sample_rate = 20000000;
    bladerf_channel rx = BLADERF_CHANNEL_RX(0);
    bladerf_bandwidth rx_bw = 10000000;
    bladerf_gain rx_gain = 65;
    bladerf_frequency rx_start_freq = 2000000000;
    bladerf_frequency rx_stop_freq = 2000000000;
    int64_t rx_step = 2000000;
    //bool rx_enable = false;
    //std::vector<bladerf_frequency> rx_hop_sequence;
    std::thread rx_thread;

    bladerf_frequency tuned_freq;

    double on_time;
    double off_time;
    //uint64_t num_samples;
    uint32_t hop_index = 0;
    //uint16_t hop_type = 0;
    bool tmp_enable;
    bool current_tx_status;

    double scan_time = 1.0;
    uint32_t tmp_scan = 0.0;

    std::vector<std::complex<int16_t>> samples;
    std::string iq_filename;
    std::string iq_file_path;
    std::vector<std::string> iq_file_list;

    std::string tmp_file;
    std::vector<uint8_t> tmp_vector;
    std::string error_msg;

    union {
        float f32;
        uint32_t u32;
    } t;

#if defined(WITH_RPI)
    // define the gpio chip and pins
    const std::filesystem::path chip_path("/dev/gpiochip0");

    gpiod::line::value gpio_value;

    // initialize the chip & gpio line
    gpiod::chip gpio_chip = gpiod::chip(chip_path);
    gpiod::request_builder gpio_request = gpio_chip.prepare_request();

    gpio_request.set_consumer("rf_control");
    gpio_request.add_line_settings(rf_ctrl_pin, gpiod::line_settings().set_direction(gpiod::line::direction::OUTPUT));

	// allocate the gpio for use
    gpiod::line_request rf_gpio_line = gpio_request.do_request();
#endif

    if (argc < 2)
    {
        std::cout << warning << "Supply a directory location of the IQ data." << std::endl;
        std::cin.ignore();
        return -1;
    }

    // make sure there is a path seperator 
    std::string save_directory = path_check(std::string(argv[1]));
    std::string base_name = "bladerf_server_log";
    std::string log_version = "1.0";

    std::string current_date = get_date();
        
    // create the full log file name
    std::string filename = save_directory + base_name + "_" + current_date + ".txt";

    // open up file stream as txt file and appendable
    std::ofstream data_log;
    data_log.open(filename, std::ios::out | std::ios::app);
        
    // write a header with the log version and date
    data_log << "#-----------------------------------------------------------------------------" << std::endl;
    data_log << "# Version: " << log_version << std::endl;
    data_log << "# Date: " << current_date << std::endl;
    data_log << "#-----------------------------------------------------------------------------" << std::endl;
 
    // read in the parameters
    //std::string param_filename = std::string(argv[1]);
    //iq_filename = std::string(argv[1]);
    
    iq_file_path = std::string(argv[1]);
    iq_file_list = directory_listing(iq_file_path, ".sc16");
    if (iq_file_list.empty() == false)
    {
        iq_filename = iq_file_path + iq_file_list[0];
    }

    // list the files found
    for(idx=0; idx< iq_file_list.size(); ++idx)
    {
        std::cout << info << iq_file_list[idx] << std::endl;
        //std::cout << warning << iq_file_list[idx] << std::endl;
        //std::cout << error(__FILE__, __LINE__) << "test" << std::endl;       
    }

    //read_hop_params(param_filename, start_freq, stop_freq, hop_step, sample_rate, tx_bw, hop_type, on_time, off_time, tx1_gain, iq_filename);

    //generate_range(tx_start_freq, tx_stop_freq, (double)tx_step, tx_hop_sequence);
    //uint32_t num_tx_hops = tx_hop_sequence.size();

    //generate_range(rx_start_freq, rx_stop_freq, (double)rx_step, rx_hop_sequence);
    //uint32_t num_rx_hops = rx_hop_sequence.size();

    // initialize a random number generator
    srand((unsigned)time(NULL));

    //-----------------------------------------------------------------------------
    // ZMQ message containers
    // zmq::message_t to hold the received messages
    std::vector<zmq::message_t> command_messages;
    std::optional<uint32_t> num_messages = std::nullopt;

    // vector of bytes to store the data portion of the multipart message
    std::vector<uint32_t> command;
    std::vector<uint8_t> message_data;

    // vector to store and send the response back to the client
    std::vector<uint32_t> msg_result;

    //-----------------------------------------------------------------------------
    std::cout << "------------------------------------------------------------------------" << std::endl;
    std::cout << "BLADERF SDR SERVER - Version: " << SDR_SERVER_MAJOR_VERSION << "." << SDR_SERVER_MINOR_VERSION << "." << SDR_SERVER_BUILD_VERSION << std::endl;
    std::cout << "------------------------------------------------------------------------" << std::endl;
    std::cout << std::endl;

    data_log << "BLADERF SDR SERVER - Version: " << SDR_SERVER_MAJOR_VERSION << "." << SDR_SERVER_MINOR_VERSION << "." << SDR_SERVER_BUILD_VERSION << std::endl;
    data_log << "------------------------------------------------------------------------" << std::endl;
    data_log << std::endl;

    std::cout << info << "Starting BladeRF SDR Server..." << std::endl;
    data_log << info << "Starting BladeRF SDR Server..." << std::endl;
    zmq::context_t bladerf_context{ 1 };
    zmq::socket_t bladerf_socket = create_server_connection(BLADERF_IP_ADDRESS, BLADERF_PORT, bladerf_context, static_cast<int32_t>(zmq::socket_type::rep));
    zmq::socket_t bladerf_pub_socket = create_server_connection(BLADERF_IP_ADDRESS, BLADERF_STATUS_PORT, bladerf_context, static_cast<int32_t>(zmq::socket_type::pub));
    std::cout << info << "BladeRF SDR Server Ready!" << std::endl << std::endl;
    data_log << info << "BladeRF SDR Server Ready!" << std::endl << std::endl;

    //-----------------------------------------------------------------------------
    // read in the data
    std::cout << info << "Loading IQ file: " << iq_filename << std::endl;
    data_log << info << "Loading IQ file: " << iq_filename << std::endl;
    read_iq_data(iq_filename, samples);

    // the number of IQ samples is the number of samples divided by 2
    num_tx_samples = samples.size();
    //double sample_duration = (num_tx_samples) / (double)tx_sample_rate;
    std::cout << info << "num_tx_samples: " << num_tx_samples << std::endl;
    data_log << info << "num_tx_samples: " << num_tx_samples << std::endl;


    //-----------------------------------------------------------------------------
    try 
    {

        int num_devices = bladerf_get_device_list(&device_list);

        bladerf_num = select_bladerf(num_devices, device_list);

        if (bladerf_num < 0)
        {
            error_msg = "Could not detect any bladeRF devices (" + std::to_string(bladerf_num) + ")";
            std::cout << warning << error_msg << std::endl;
            data_log << warning << error_msg << std::endl;
            return 0;
        }

        std::cout << std::endl;

        blade_status = bladerf_open(&dev, ("*:serial=" + std::string(device_list[bladerf_num].serial)).c_str());
        if (blade_status != 0)
        {
            error_msg = "Unable to open device: " + std::string(bladerf_strerror(blade_status));
            std::cout << warning << error_msg << std::endl;
            data_log << warning << error_msg << std::endl;
            return blade_status;
        }

        blade_status = bladerf_get_devinfo(dev, &dev_info);
        if (blade_status != 0)
        {
            error_msg = "Unable to get the device info: " + std::string(bladerf_strerror(blade_status));
            std::cout << warning << error_msg << std::endl;
            data_log << warning << error_msg << std::endl;
            return blade_status;
        }
        std::cout << dev_info << std::endl;

        tx_hops = get_hop_parameters(dev, tx, tx_start_freq, tx_stop_freq, tx_step);
        rx_hops = get_hop_parameters(dev, rx, rx_start_freq, rx_stop_freq, rx_step);
        num_tx_hops = std::max(0U, (uint32_t)tx_hops.size());
        num_rx_hops = std::max(0U, (uint32_t)rx_hops.size());

        std::cout << info << "number of TX hops: " << num_tx_hops << std::endl << std::endl;
        std::cout << info << "number of RX hops: " << num_rx_hops << std::endl << std::endl;
        data_log << info << "number of TX hops: " << num_tx_hops << std::endl << std::endl;
        data_log << info << "number of RX hops: " << num_rx_hops << std::endl << std::endl;

        //-----------------------------------------------------------------------------
        // set the sample_rate and bandwidth
        blade_status = bladerf_set_sample_rate(dev, tx, tx_sample_rate, &tx_sample_rate);
        blade_status |= bladerf_set_bandwidth(dev, tx, tx_bw, &tx_bw);

        // configure the sync to receive/transmit data
        blade_status |= bladerf_sync_config(dev, BLADERF_TX_X1, BLADERF_FORMAT_SC16_Q11, num_buffers, buffer_size, num_transfers, blade_timeout_ms);
        if (blade_status != 0)
        {
            error_msg = "Failed to configure TX sync interface - error: " + std::string(bladerf_strerror(blade_status));
            std::cout << warning << error_msg << std::endl;
            data_log << warning << error_msg << std::endl;
        }
        
        //blade_status = bladerf_sync_config(dev, BLADERF_RX_X1, BLADERF_FORMAT_SC16_Q11, num_buffers, buffer_size, num_transfers, blade_timeout_ms);
        //if (blade_status != 0)
        //{
        //    std::cout << "Failed to configure RX sync interface - error: " << std::string(bladerf_strerror(blade_status)) << std::endl;
        //}

        //blade_status = bladerf_set_gain_mode(dev, rx, BLADERF_GAIN_MGC);

        // config the tx side
        //blade_status = config_blade_channel(dev, tx, tx_hops[0].freq, tx_sample_rate, tx_bw, tx_gain);
        // enable the TX channel RF frontend
        blade_status = bladerf_enable_module(dev, BLADERF_TX, true);
        if (blade_status != 0)
        {
            error_msg = "Error enabling TX - error: " + std::string(bladerf_strerror(blade_status));
            std::cout << warning << error_msg << std::endl;
            data_log << warning << error_msg << std::endl;
        }

        // the gain must be set after the module has been enabled
        blade_status = bladerf_set_gain_mode(dev, tx, BLADERF_GAIN_MANUAL);
        blade_status |= bladerf_set_gain(dev, tx, tx_gain);
        blade_status |= bladerf_get_gain(dev, tx, &tx_gain);
        if (blade_status != 0)
        {
            error_msg = "Error setting TX gain - error: " + std::string(bladerf_strerror(blade_status));
            std::cout << warning << error_msg << std::endl;
            data_log << warning << error_msg << std::endl;
        }

        blade_status = bladerf_set_bandwidth(dev, tx, 20000000, NULL);
        blade_status = bladerf_set_bandwidth(dev, tx, tx_bw, &tx_bw);

        // config the rx side
        //blade_status = config_blade_channel(dev, rx, rx_hops[0].freq, rx_sample_rate, rx_bw, rx_gain);

        // recieve mode
        //if (blade_mode == 0)
        //{
        //    // stop transmitting
        //    transmit = false;

        //    //tuned_freq = rx_hops[hop_index].freq;
        //    blade_status = switch_blade_mode(dev, blade_mode, tx, rx);
        //    //blade_status |= bladerf_set_frequency(dev, rx, tuned_freq);

        //    // start the rx thread
        //    //recieve = true;
        //}
        //// transmit mode
        //else
        //{
        //    // stop recieving
        //    recieve = false;

        //    //tuned_freq = tx_hops[hop_index].freq;
        //    blade_status = switch_blade_mode(dev, blade_mode, tx, rx);
        //    //blade_status |= bladerf_set_frequency(dev, tx, tuned_freq);

        //    // start the tx thread
        //    //transmit = true;
        //}

        blade_status = bladerf_set_frequency(dev, tx, tx_hops[0].freq);
        if (blade_status != 0)
        {
            error_msg = "Error setting frequency: " + std::string(bladerf_strerror(blade_status));
            std::cout << warning << error_msg << std::endl;
            data_log << warning << error_msg << std::endl;
        }

        std::cout << info << "Config:" << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl;
        std::cout << "Transmitter:" << std::endl;
        std::cout << "  start freq:  " << tx_start_freq << std::endl;
        std::cout << "  stop freq:   " << tx_stop_freq << std::endl;
        std::cout << "  freq step:   " << tx_step << std::endl;
        std::cout << "  num hops:    " << num_tx_hops << std::endl;
        std::cout << "  hop type:    " << tx_hop_type << std::endl;
        std::cout << "  sample_rate: " << tx_sample_rate << std::endl;
        std::cout << "  bw:          " << tx_bw << std::endl;
        std::cout << "  tx1_gain:    " << tx_gain << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl << std::endl;

        data_log << info << "Config:" << std::endl;
        data_log << "------------------------------------------------------------------" << std::endl;
        data_log << "Transmitter:" << std::endl;
        data_log << "  start freq:  " << tx_start_freq << std::endl;
        data_log << "  stop freq:   " << tx_stop_freq << std::endl;
        data_log << "  freq step:   " << tx_step << std::endl;
        data_log << "  num hops:    " << num_tx_hops << std::endl;
        data_log << "  hop type:    " << tx_hop_type << std::endl;
        data_log << "  sample_rate: " << tx_sample_rate << std::endl;
        data_log << "  bw:          " << tx_bw << std::endl;
        data_log << "  tx1_gain:    " << tx_gain << std::endl;
        data_log << "------------------------------------------------------------------" << std::endl << std::endl;

        // use this to flush out some samples that might be hung
        std::vector<int16_t> tmp_samples(buffer_size*4);
        blade_status = bladerf_sync_tx(dev, (int16_t*)tmp_samples.data(), buffer_size*2, NULL, blade_timeout_ms);

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

        // handle SIGINT signals
        if (signal(SIGINT, sig_handler) == SIG_ERR) 
        {
            std::cerr << warning << "Unable to catch SIGINT signals" << std::endl;
            data_log << warning << "Unable to catch SIGINT signals" << std::endl;
        }
        // handle SIGTERM signals
        if (signal(SIGTERM, sig_handler) == SIG_ERR)
        {
            std::cerr << warning << "Unable to catch SIGTERM signals" << std::endl;
            data_log << warning << "Unable to catch SIGTERM signals" << std::endl;
        }

        is_running = true;
        
        // set the thread conditions before initializing the threads
        transmit_thread_running = true;
        recieve_thread_running = true;

        // start the tx thread
        tx_thread = std::thread(transmit_thread, dev, tx, std::ref(samples));

        // publisher thread
        std::thread pub_thread = std::thread(publisher_thread, std::ref(bladerf_pub_socket));

        // wait for a little to get the thread started
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        //scan = true;
        std::thread sc_thread = std::thread(scan_thread, dev, tx, std::ref(scan_time), std::ref(tx_gain));
        
        // wait for a little to get the thread started
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::cout << std::endl << info << "SDR Server Running..." << std::endl << std::endl;
        data_log << std::endl << info << "SDR Server Running..." << std::endl << std::endl;

        //-----------------------------------------------------------------------------
        // main loop
        while (is_running)
        {
            // clear all the old messages
            command_messages.clear();

            // non-blocking call to receive multiple messages
            num_messages = zmq::recv_multipart(bladerf_socket, std::back_inserter(command_messages), zmq::recv_flags::dontwait);
            
            // check to see if any messages have arrived
            if (num_messages.has_value() == false)
            {
                //std::cout << ".";
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            
            // clear out the containers if we receive a new message
            command.clear();
            message_data.clear();
            msg_result.clear();
            
            // parse the first part of the message to get the command
            error_msg = "Received multipart message with " + std::to_string(num_messages.value()) + " messages"; 
            std::cout << info << error_msg << std::endl;
            data_log << info << error_msg << std::endl;

            // place the command messages into the command vector
            command.resize(command_messages[0].size() / sizeof(uint32_t));
            std::memcpy(command.data(), command_messages[0].data(), command_messages[0].size());
            std::cout << info << "Message command id: " << num2str(command[0], "0x%08X") << std::endl << std::endl;
            data_log << info << "Message command id: " << num2str(command[0], "0x%08X") << std::endl << std::endl;

            // if there are more than one message in a multipart message try to parse the message
            if (num_messages.value() > 1)
            {
                // parse the message into the message_data vector
                message_data.resize(command_messages[1].size());               // / sizeof(uint8_t)
                std::memcpy(message_data.data(), command_messages[1].data(), command_messages[1].size());
                std::cout << info << "Data size (bytes): " << message_data.size() << std::endl;
                data_log << info << "Data size (bytes): " << message_data.size() << std::endl;

                // TODO: Debug
                //for (idx = 0; idx < message_data.size(); ++idx)
                //    std::cout << (uint16_t)message_data[idx] << " ";

                //std::cout << std::endl << std::endl;
            }

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
                msg_result[1] = SDR_SERVER_MAJOR_VERSION;
                msg_result[2] = SDR_SERVER_MINOR_VERSION;
                msg_result[3] = SDR_SERVER_BUILD_VERSION;
                break;

            case static_cast<uint32_t>(BLADE_MSG_ID::SELECT_MODE):
                blade_mode = command[1];

                // recieve mode
                if (blade_mode == 0)
                {
                    // stop transmitting
                    transmit = false;

                    tuned_freq = rx_hops[hop_index].freq;
                    blade_status = switch_blade_mode(dev, blade_mode, tx, rx);
                    blade_status |= bladerf_set_frequency(dev, rx, tuned_freq);

                    // start the rx thread
                    recieve = true;
                }
                // transmit mode
                else
                {
                    // stop recieving
                    recieve = false;

                    tuned_freq = tx_hops[hop_index].freq;
                    blade_status = switch_blade_mode(dev, blade_mode, tx, rx);
                    blade_status |= bladerf_set_frequency(dev, tx, tuned_freq);

                    // start the tx thread
                    transmit = true;
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
                t.u32 = command[CONFIG_SCAN_INDEX];

                scan_time = t.f32;

                //generate_range(tx_start_freq, tx_stop_freq, (double)tx_step, tx_hop_sequence);
                tx_hops = get_hop_parameters(dev, tx, tx_start_freq, tx_stop_freq, tx_step);
                num_tx_hops = std::max(0U, (uint32_t)tx_hops.size());

                //blade_status = config_blade_channel(dev, tx, tx_hops[0].freq, tx_sample_rate, tx_bw, tx_gain);
                blade_status = bladerf_set_gain(dev, tx, tx_gain);
                blade_status = bladerf_get_gain(dev, tx, &tx_gain);
                blade_status |= bladerf_set_bandwidth(dev, tx, tx_bw, &tx_bw);
                blade_status |= bladerf_set_frequency(dev, tx, tx_hops[0].freq);
                if (blade_status != 0)
                {
                    std::cout << warning << "Error configuring channel: " << std::string(bladerf_strerror(blade_status)) << std::endl;
                    data_log << warning << "Error configuring channel: " << std::string(bladerf_strerror(blade_status)) << std::endl;
                }

                if (scan_time == 0.0)
                {
                    {
                        std::lock_guard<std::mutex> lock(scan_mutex);
                        scan = false;
                    }
                }

                std::cout << info << "Config:" << std::endl;
                std::cout << "------------------------------------------------------------------" << std::endl;
                std::cout << "Transmitter:" << std::endl;
                std::cout << "  start freq:  " << tx_start_freq << std::endl;
                std::cout << "  stop freq:   " << tx_stop_freq << std::endl;
                std::cout << "  freq step:   " << tx_step << std::endl;
                std::cout << "  num hops:    " << num_tx_hops << std::endl;
                std::cout << "  hop type:    " << tx_hop_type << std::endl;
                std::cout << "  sample_rate: " << tx_sample_rate << std::endl;
                std::cout << "  bw:          " << tx_bw << std::endl;
                std::cout << "  tx1_gain:    " << tx_gain << std::endl;
                std::cout << "------------------------------------------------------------------" << std::endl << std::endl;

                data_log << info << "Config:" << std::endl;
                data_log << "------------------------------------------------------------------" << std::endl;
                data_log << "Transmitter:" << std::endl;
                data_log << "  start freq:  " << tx_start_freq << std::endl;
                data_log << "  stop freq:   " << tx_stop_freq << std::endl;
                data_log << "  freq step:   " << tx_step << std::endl;
                data_log << "  num hops:    " << num_tx_hops << std::endl;
                data_log << "  hop type:    " << tx_hop_type << std::endl;
                data_log << "  sample_rate: " << tx_sample_rate << std::endl;
                data_log << "  bw:          " << tx_bw << std::endl;
                data_log << "  tx1_gain:    " << tx_gain << std::endl;
                data_log << "------------------------------------------------------------------" << std::endl << std::endl;


                msg_result.resize(2);
                msg_result[0] = static_cast<uint32_t>(BLADE_MSG_ID::CONFIG_TX);
                msg_result[1] = (uint32_t)(blade_status == 0);
                break;
				
            case static_cast<uint32_t>(BLADE_MSG_ID::ENABLE_AMP):
                recieve = false;
                tmp_enable = (bool)command[1];

                std::cout << info << "Entered ENABLE_AMP state: " << tmp_enable << std::endl;
                data_log << info << "Entered ENABLE_AMP state: " << tmp_enable << std::endl;
#if defined(WITH_RPI)
                // set the amp gpio pin
                gpio_value = (tmp_enable == true) ? gpio_on : gpio_off;
                rf_gpio_line.set_value(rf_ctrl_pin, gpio_value);
#endif
                std::this_thread::sleep_for(std::chrono::milliseconds(1100));

                msg_result.resize(2);
                msg_result[0] = static_cast<uint32_t>(BLADE_MSG_ID::ENABLE_AMP);
                msg_result[1] = static_cast<uint32_t>(tmp_enable);
                break;

            case static_cast<uint32_t>(BLADE_MSG_ID::ENABLE_TX):
                recieve = false;
                tmp_enable = (bool)command[1];

                //transmit = enable_channel(dev, tx, tmp_enable, transmit);
                {
                    std::lock_guard<std::mutex> lock(tx_mutex);
                    transmit = tmp_enable;
                }
                if(transmit == true)
                    tx_cv.notify_one();

                std::cout << info << "Transmit: " << transmit << std::endl;
                data_log << info << "Transmit: " << transmit << std::endl;

                msg_result.resize(2);
                msg_result[0] = static_cast<uint32_t>(BLADE_MSG_ID::ENABLE_TX);
                msg_result[1] = static_cast<uint32_t>(transmit);
                break;

            case static_cast<uint32_t>(BLADE_MSG_ID::ENABLE_SCAN):
                tmp_enable = (bool)command[1];

                {
                    std::lock_guard<std::mutex> lock(scan_mutex);
                    scan = tmp_enable;
                }

                if (scan == true)
                {
                    scan_cv.notify_one();
                }
                else
                {
                    blade_status = bladerf_set_gain(dev, tx, tx_gain);
                }

                msg_result.resize(2);
                msg_result[0] = static_cast<uint32_t>(BLADE_MSG_ID::ENABLE_SCAN);
                msg_result[1] = static_cast<uint32_t>(scan);

                std::cout << info << "Scan: " << scan << std::endl;
                data_log << info << "Scan: " << scan << std::endl;
                break;

            case static_cast<uint32_t>(BLADE_MSG_ID::GET_IQ_FILES):
                msg_result.resize(2);
                msg_result[0] = static_cast<uint32_t>(BLADE_MSG_ID::GET_IQ_FILES);
                msg_result[1] = 1;

                // rescan the list in case things have changed
                iq_file_list.clear();
                iq_file_list = directory_listing(iq_file_path, ".sc16");
                if (iq_file_list.empty() == false)
                {
                    iq_filename = iq_file_path + iq_file_list[0];
                }

                // the number of iq files in the supplied directory = iq_file_list.size()
                // data order:
                // - number of files (uint32_t)
                // - file 1 length (uint32_t)
                // - file 1 name (uint32_t) per letter
                // - file 2 length (uint32_t)
                // - file 2 name (uint32_t) per letter
                // - etc
                msg_result.push_back((uint32_t)iq_file_list.size());
                for (idx = 0; idx < iq_file_list.size(); ++idx)
                {
                    msg_result.push_back((uint32_t)iq_file_list[idx].size());

                    for (jdx = 0; jdx < iq_file_list[idx].size(); ++jdx)
                    {
                        msg_result.push_back(iq_file_list[idx][jdx]);
                    }
                }

                break;

            case static_cast<uint32_t>(BLADE_MSG_ID::LOAD_IQ_FILE):
                current_tx_status = transmit;
                transmit = false;

                // if there is a filename try to load the data
                if (message_data.size() > 0)
                {
                    iq_filename.clear();
                    iq_filename.resize(message_data.size());
                    std::copy(message_data.begin(), message_data.end(), iq_filename.begin());
                    std::cout << info << "IQ filename: " << iq_file_path + iq_filename << std::endl;
                    data_log << info << "IQ filename: " << iq_file_path + iq_filename << std::endl;

                    samples = read_iq_data<int16_t>(iq_file_path + iq_filename);
                    num_tx_samples = samples.size();
                    std::cout << info << "num_tx_samples: " << num_tx_samples << std::endl;
                    data_log << info << "num_tx_samples: " << num_tx_samples << std::endl;
                }

                // return transmit to its former status
                transmit = current_tx_status;
            
                msg_result.resize(2);
                msg_result[0] = static_cast<uint32_t>(BLADE_MSG_ID::LOAD_IQ_FILE);
                msg_result[1] = 1;
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

        transmit_thread_running = false;
        recieve_thread_running = false;
        transmit = false;
        recieve = false;
        scan = false;

        tx_thread.join();
        pub_thread.join();
        sc_thread.join();

#if defined(WITH_RPI)
        // close the gpio line
        rf_gpio_line.set_value(rf_ctrl_pin, gpio_off);
        rf_gpio_line.release();
        gpio_chip.close();

        std::cout << info << "Closing GPIO..." << std::endl;
        data_log << info << "Closing GPIO..." << std::endl;
#endif

        // close the server
        std::cout << std::endl << info << "Closing the SDR Server..." << std::endl;
        data_log << std::endl << info << "Closing the SDR Server..." << std::endl;
        close_server(bladerf_context, { &bladerf_socket, &bladerf_pub_socket });

        // disable the tx channel RF frontend
        blade_status = bladerf_enable_module(dev, BLADERF_TX, false);
        blade_status = bladerf_enable_module(dev, BLADERF_RX, false);
        std::cout << info << "Disabled BladeRF frontend..." << std::endl;
        data_log << info << "Disabled BladeRF frontend..." << std::endl;

    }
    catch (std::exception e)
    {
        std::cout << error(__FILE__, __LINE__) << "Error: " << e.what() << std::endl;
        data_log << error(__FILE__, __LINE__) << "Error: " << e.what() << std::endl;
#if defined(WITH_RPI)
        // close the gpio line
        rf_gpio_line.set_value(rf_ctrl_pin, gpio_off);
        rf_gpio_line.release();
        gpio_chip.close();

        std::cout << info << "Closing GPIO..." << std::endl;
        data_log << info << "Closing GPIO..." << std::endl;
#endif

    }

    std::cout << info << "Closing BladeRF..." << std::endl;
    data_log << info << "Closing BladeRF..." << std::endl;
    bladerf_close(dev);

    // close the data logger file
    data_log.close();

    std::cout << info << "Press Enter to close..." << std::endl;
    //std::cin.ignore();    
    
    return 0;
    
}   // end of main
