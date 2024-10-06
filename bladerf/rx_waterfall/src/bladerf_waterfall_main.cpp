// ----------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)

#elif defined(__linux__)

#endif

#include <cstdint>
#include <csignal>
#include <iostream>
#include <sstream>
#include <vector>
#include <thread>
#include <complex>
#include <algorithm>
#include <atomic>
#include <mutex>
#include <condition_variable>

// bladeRF includes
#include <libbladeRF.h>
#include <bladeRF2.h>

// Custom Includes
#include "num2string.h"
#include "get_current_time.h"
//#include "file_parser.h"
//#include "file_ops.h"

// Project Includes
#include <bladerf_common.h>


// ----------------------------------------------------------------------------
// Globals
std::atomic<bool> rx_run(false);
std::atomic<bool> rx_complete = false;

const uint32_t buffer_size = 1024 * 4;        // must be a multiple of 1024
const uint32_t timeout_ms = 10000;
uint32_t N = 200;

std::vector<std::complex<int16_t>> samples;

uint32_t read_signal = 0;

std::mutex read_data_mutex;
std::condition_variable data_cv;

// ----------------------------------------------------------------------------
inline void get_samples(struct bladerf* dev, uint64_t num_samples)
{
    int blade_status;

    uint64_t mem_address;

    std::cout << "RX Started..." << std::endl;
    while (rx_run)
    {        
        //std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Simulate work
        //rx_complete = false;

        mem_address = read_signal * num_samples;
        blade_status = bladerf_sync_rx(dev, (void*)(samples.data() + mem_address), num_samples, NULL, timeout_ms);
        if (blade_status != 0)
        {
            std::cout << "Unable to get the required number of samples: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            //rx_run = false;
        }
        std::lock_guard<std::mutex> lock{read_data_mutex};

        read_signal = (read_signal + 1) & 0x01;

        rx_complete = true;
        std::cout << "rx complete" <<  std::endl;

        //lock.unlock();
        data_cv.notify_one();
    }
}

//-----------------------------------------------------------------------------
void sig_handler(int signo)
{
    if (signo == SIGINT)
    {
        fprintf(stderr, "received SIGINT\n");
        rx_run = false;
        //fprintf(stderr, "received another SIGINT, aborting\n");
        //abort();

    }
}


// ----------------------------------------------------------------------------
int main(int argc, char** argv)
{

    uint32_t idx;
    
    // bladeRF variable
    struct bladerf_devinfo *device_list = NULL;
    struct bladerf_devinfo dev_info;
    struct bladerf* dev;
    int bladerf_num;
    int blade_status;
    bladerf_channel rx = BLADERF_CHANNEL_RX(0);
    bladerf_channel tx = BLADERF_CHANNEL_TX(0);
    bladerf_frequency rx_freq = 137000000; //162425000;
    bladerf_frequency rx_freqa = 0;
    bladerf_sample_rate sample_rate = 624000;
    bladerf_bandwidth rx_bw = 624000;
    bladerf_gain rx1_gain = 64;
    int64_t span = 624000;

    uint64_t num_samples = buffer_size * N;
    //uint32_t timeout_ms = 10000;
    const uint32_t num_buffers = 16;
    const uint32_t num_transfers = 8;
    double t;

    if (argc == 2)
    {
        std::string param_filename = argv[1];

        read_bladerf_params(param_filename, rx_freq, sample_rate, rx_bw, rx1_gain);
    }

    int num_devices = bladerf_get_device_list(&device_list);

    bladerf_num = select_bladerf(num_devices, device_list);

    if (bladerf_num < 0)
    {
        std::cout << "could not detect any bladeRF devices..." << std::endl;
        std::cin.ignore();
        return 0;
    }

    std::cout << std::endl;

    try{
      
        blade_status = bladerf_open(&dev, ("*:serial=" +  std::string(device_list[bladerf_num].serial)).c_str());
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
        std::cout << std::endl << dev_info << std::endl;

        // set the frequency, sample_rate and bandwidth
        blade_status = bladerf_set_frequency(dev, rx, rx_freq);
        blade_status = bladerf_get_frequency(dev, rx, &rx_freqa);
        blade_status = bladerf_set_sample_rate(dev, rx, sample_rate, &sample_rate);
        blade_status = bladerf_set_bandwidth(dev, rx, rx_bw, &rx_bw);

        const bladerf_range* range;
        blade_status = bladerf_get_gain_range(dev, rx, &range);

        // configure the sync to receive/transmit data
        blade_status = bladerf_sync_config(dev, BLADERF_RX_X1, BLADERF_FORMAT_SC16_Q11, num_buffers, buffer_size, num_transfers, timeout_ms);

        // enable the rx channel RF frontend
        blade_status = bladerf_enable_module(dev, BLADERF_RX, true);

        // the gain must be set after the module has been enabled
        blade_status = bladerf_set_gain_mode(dev, rx, BLADERF_GAIN_MANUAL);
        blade_status = bladerf_set_gain(dev, rx, rx1_gain);
        blade_status = bladerf_get_gain(dev, rx, &rx1_gain);

        // receive the samples 
        // the *2 is because one sample consists of one I and one Q.  The data should be packed IQIQIQIQIQIQ...
        samples.resize(num_samples*2, std::complex<int16_t>(0,0));

        span = sample_rate;
        double freq_step = (sample_rate)/(double)num_samples;

        double f_min = (rx_freq - (span>>1)) * 1.0e-6;
        double f_max = ((rx_freq-1) + (span>>1)) * 1.0e-6;

        uint32_t sp = (uint32_t)((sample_rate - span) / (2.0 * freq_step));
        uint32_t sp2 = (uint32_t)(span / freq_step);

        // print out the specifics
        std::cout << std::endl << "------------------------------------------------------------------" << std::endl;
        std::cout << "sample_rate: " << sample_rate << std::endl;
        std::cout << "rx_freq:     " << rx_freq << std::endl;
        std::cout << "rx_bw:       " << rx_bw << std::endl;
        std::cout << "rx1_gain:    " << rx1_gain << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl << std::endl;

        rx_run = true;
        //rx_complete = false;

        if (signal(SIGINT, sig_handler) == SIG_ERR)
        {
            std::cerr << "Unable to catch SIGINT signals" << std::endl;
        }

        // start the rx thread
        std::cout << "Starting RX thread..." << std::endl;
        std::thread rx_thread(get_samples, dev, num_samples);

        uint64_t index = 0;

        while(rx_run == true)
        {
            rx_complete = false;
            std::cout << "index: " << ++index << std::endl; 

            std::unique_lock<std::mutex> lock{read_data_mutex};

            //while(rx_complete == false)
            //    std::cout << rx_complete << std::endl;

            data_cv.wait(lock, [] { return rx_complete == true; });
            //data_cv.wait(lock);

            std::cout << "read_signal = " << read_signal << std::endl;
            uint64_t mem_address = read_signal * num_samples;

            //std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Simulate work

            //for (idx = 0; idx < num_samples; ++idx)
            //{
            //    std::cout << samples[idx+ mem_address] << ", ";
            //}
            //std::cout<< std::endl;

            //lock.unlock();
            //data_cv.notify_one();
        }

        rx_thread.join();

        // disable the rx channel RF frontend
        blade_status = bladerf_enable_module(dev, BLADERF_RX, false);

        bladerf_close(dev);

    }
    catch(std::exception e)
    {
        std::cout << "error: " << e.what() << std::endl;
    }

    std::cout << "Press enter to close..." << std::endl;
    std::cin.ignore();
    return 0;
    
}   // end of main
