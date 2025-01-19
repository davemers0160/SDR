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

// bladeRF includes
#include <libbladeRF.h>
#include <bladeRF2.h>

// Custom Includes
#include "num2string.h"
#include "get_current_time.h"
#include "file_parser.h"
//#include "file_ops.h"
#include "sleep_ms.h"
#include "iq_utils.h"

// Project Includes
#include "bladerf_common.h"
#include "parse_hop_input.h"

//-----------------------------------------------------------------------------
// Globals
const double pi = 3.14159265358979323846;
const double pi2 = 2 * 3.14159265358979323846;

//bladerf_frequency hop_step = 25000;
//const bladerf_frequency start_freq = 47000000;
//const bladerf_frequency stop_freq = start_freq + 5000000;
bool is_running = false;

//-----------------------------------------------------------------------------
void sig_handler(int signo)
{
    if (signo == SIGINT) 
    {
        fprintf(stderr, "received SIGINT\n");
        is_running = false;
        //fprintf(stderr, "received another SIGINT, aborting\n");
        //abort();

    }
}



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
    const uint32_t buffer_size = 1024*40;        // must be a multiple of 1024
    const uint32_t num_transfers = 128;
    uint32_t timeout_ms = 10000;
    uint32_t hop_index;
    uint16_t hop_type;

    std::vector<std::complex<int16_t>> samples;
    std::string iq_filename;

    std::vector<hop_params> hops;

    struct bladerf_metadata meta;
    memset(&meta, 0, sizeof(meta));

    //std::ofstream data_log_stream;

    std::string sdate, stime;
    std::string logfile_name = "../results/hop_logfile_";

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

    // ----------------------------------------------------------------------------
    // read in the data
    read_iq_data(iq_filename, samples);

    // the number of IQ samples is the number of samples divided by 2
    num_samples = samples.size();
    uint64_t sample_duration = (num_samples * 1e6) / (double)sample_rate;

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

        //if (start_freq > stop_freq)
        //    hop_step *= -1;

        //// create the hop sequence
        //uint32_t num_hops = (uint32_t)std::abs(std::floor(((double)stop_freq - (double)start_freq) / (double)hop_step));

        //if (num_hops == 0)
        //    ++num_hops;

        std::cout << "number of hops: " << num_hops << std::endl << std::endl;

        for (idx = 0; idx < num_hops; ++idx)
        {

            //auto f = start_freq + (idx * hop_step);
            hop_params p;
            //p.f = start_freq + (idx * hop_step);
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
        blade_status = bladerf_sync_config(dev, BLADERF_TX_X1, BLADERF_FORMAT_SC16_Q11, num_buffers, buffer_size, num_transfers, timeout_ms);

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
        blade_status = bladerf_set_gain_mode(dev, tx, BLADERF_GAIN_MANUAL);
        blade_status = bladerf_set_gain(dev, tx, tx1_gain);
        blade_status = bladerf_get_gain(dev, tx, &tx1_gain);
        if (blade_status != 0)
        {
            std::cout << "Error setting TX gain - error: " << std::string(bladerf_strerror(blade_status)) << std::endl;
        }

        blade_status = bladerf_get_timestamp(dev, BLADERF_TX, &meta.timestamp);
        if (blade_status != 0) 
        {
            fprintf(stderr, "Failed to get initial timestamp: %s\n", bladerf_strerror(blade_status));
            //goto out;
        }

        // Add some initial startup delay
        meta.timestamp += (sample_rate / 50.0);
        meta.flags = BLADERF_META_FLAG_TX_BURST_START | BLADERF_META_FLAG_TX_BURST_END;

        // start up the data logger
        get_current_time(sdate, stime);
        logfile_name = logfile_name + sdate + "_" + stime + ".bin";

        //std::cout << "Log File: " << (logfile_name) << std::endl << std::endl;
        //data_log_stream.open(logfile_name, ios::out | ios::binary);

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

        //data_log_stream << (uint8_t)hop_index;

        is_running = true;

        if (signal(SIGINT, sig_handler) == SIG_ERR) 
        {
            std::cerr << "Unable to catch SIGINT signals" << std::endl;
        }
        
        std::cout << std::endl << "Sending signals..." << std::endl << std::endl;


        while (is_running)
        {
            //start_time = std::chrono::high_resolution_clock::now();

            //do
            //{
                //std::cout << "hop_index: " << hop_index << std::endl;
  
                //printf("nios_profile = %u, rffe_profile = %u\n",freqs[hopseq[f]].qt.nios_profile, freqs[hopseq[f]].qt.rffe_profile);
                //for(idx=0; idx<2;++idx)
                //{
                    blade_status = bladerf_sync_tx(dev, (int16_t*)samples.data(), num_samples, NULL, timeout_ms);
                    if (blade_status != 0)
                    {
                        std::cout << "Unable to send the required number of samples: " << std::string(bladerf_strerror(blade_status)) << std::endl;
                    }
                //}

                switch (hop_type)
                {
                case 0:
                    hop_index = (hop_index + 1) % num_hops;
                    break;
                case 1:
                    hop_index = (uint32_t)(rand() % num_hops);
                    break;
                default:
                    hop_index = (hop_index + 1) % num_hops;
                    break;
                }

                // normal tuning
                blade_status = bladerf_set_frequency(dev, tx, hops[hop_index].f);
                if (blade_status != 0)
                {
                    std::cout << "Failed to perform frequency change to: " << hops[hop_index].f << " -- blade error: " << std::string(bladerf_strerror(blade_status)) << std::endl;
                }

                // quicktune
                //blade_status = bladerf_schedule_retune(dev, tx, BLADERF_RETUNE_NOW, hops[hop_index].f, &hops[hop_index].qt);
                //if (blade_status != 0)
                //{
                //    std::cout << "Failed to perform quick tune to index: " << hop_index << ". blade error: " << std::string(bladerf_strerror(blade_status)) << std::endl;
                //}

            //    stop_time = std::chrono::high_resolution_clock::now();
            //    duration = std::chrono::duration_cast<chrono::nanoseconds>(stop_time - start_time).count();

            //} while (duration < on_time);


            // wait for the off_time
            //start_time = std::chrono::high_resolution_clock::now();

            //do
            //{
            //sleep_ms(1);
                //std::this_thread::sleep_for(std::chrono::microseconds(200));
                //std::this_thread::sleep_for(std::chrono::milliseconds(5));
                //    stop_time = std::chrono::high_resolution_clock::now();
            //    duration = std::chrono::duration_cast<chrono::nanoseconds>(stop_time - start_time).count();

            //} while (duration < off_time);


            //data_log_stream << (uint8_t)hop_index;

            // Update timestamp for next transmission
            //meta.timestamp += num_samples;

            //sleep_ms(10);
        }

        std::cout << "Done sending signals.  Closing BladeRF..." << std::endl;

        // close the logger
        //data_log_stream.close();

        // disable the rx channel RF frontend
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
