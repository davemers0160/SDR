// ----------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS

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

const double pi = 3.14159265358979323846;
const double pi2 = 2 * 3.14159265358979323846;

const bladerf_frequency hop_step = 30000;
const bladerf_frequency start_freq = 314000000;
const bladerf_frequency stop_freq = start_freq + 2370000;


//-----------------------------------------------------------------------------
void sig_handler(int signo)
{
    if (signo == SIGINT) {
        fprintf(stderr, "received SIGINT\n");
        fprintf(stderr, "received another SIGINT, aborting\n");
        abort();

    }
}


//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{

    uint32_t idx, jdx;
    
    // bladeRF variable
    struct bladerf_devinfo *device_list = NULL;
    struct bladerf_devinfo dev_info;
    struct bladerf* dev;
    int bladerf_num;
    int blade_status;
    bladerf_sample_rate sample_rate = 20e6;     // 10 MHz
    bladerf_channel tx = BLADERF_CHANNEL_TX(0);
    bladerf_frequency tx_freq = 1500000000;// 314300000;
    bladerf_bandwidth tx_bw = 20000000;
    bladerf_gain tx1_gain = 10;

    uint64_t num_samples;
    const uint32_t num_buffers = 16;
    const uint32_t buffer_size = 1024*8;        // must be a multiple of 1024
    const uint32_t num_transfers = 8;
    uint32_t timeout_ms = 10000;
    uint32_t hop_index;

    std::vector<std::complex<int16_t>> samples;
    std::string filename;

    std::vector<hop_params> hops;

    struct bladerf_metadata meta;
    memset(&meta, 0, sizeof(meta));

    if (argc < 2)
    {
        std::cout << "supply a file name for the IQ data." << std::endl;
        std::cin.ignore();
        return -1;
    }

    // initialize a random number generator
    srand((unsigned)time(NULL));

    filename = std::string(argv[1]);

    // ----------------------------------------------------------------------------
    // read in the data
    read_iq_data(filename, samples);

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

        // create the hop sequence
        uint32_t num_hops = (uint32_t)std::floor((stop_freq - start_freq) / (double)hop_step);
        for (idx = 0; idx < num_hops; ++idx)
        {

            auto f = start_freq + (idx * hop_step);
            hop_params p;
            p.f = start_freq + (idx * hop_step);

            hops.push_back(p);

            blade_status = bladerf_set_frequency(dev, tx, hops[idx].f);
            if (blade_status != 0)
            {
                std::cout << "Failed to set frequency: " << hops[idx].f << "Hz.   error: " << std::string(bladerf_strerror(blade_status)) << std::endl;
                std::cin.ignore();
            }

            blade_status = bladerf_get_quick_tune(dev, tx, &hops[idx].qt);
            if (blade_status != 0)
            {
                std::cout << "Failed to get quick tune: " << hops[idx].f << "Hz.   error: " << std::string(bladerf_strerror(blade_status)) << std::endl;
                std::cin.ignore();
            }

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

        // set initial frequency
        hop_index = (uint32_t)(rand() % num_hops);
        blade_status = bladerf_set_frequency(dev, tx, hops[hop_index].f);

        if (signal(SIGINT, sig_handler) == SIG_ERR) 
        {
            std::cerr << "Unable to catch SIGINT signals" << std::endl;
        }

        while (1)
        {
            hop_index = (uint32_t)(rand() % num_hops);

            //printf("nios_profile = %u, rffe_profile = %u\n",freqs[hopseq[f]].qt.nios_profile, freqs[hopseq[f]].qt.rffe_profile);
            blade_status = bladerf_sync_tx(dev, (int16_t*)samples.data(), num_samples, NULL, timeout_ms);
            if (blade_status != 0)
            {
                std::cout << "Unable to send the required number of samples: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            }

            blade_status = bladerf_schedule_retune(dev, tx, BLADERF_RETUNE_NOW, hops[hop_index].f, &hops[hop_index].qt);
            if (blade_status != 0) 
            {
                std::cout << "Failed to perform quick tune to index: " << hop_index << ". blade error: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            }

            // Update timestamp for next transmission
            //meta.timestamp += num_samples;

            //sleep_ms(10);
        }

        std::cout << "Done sending signals.  Closing BladeRF..." << std::endl;

        // disable the rx channel RF frontend
        blade_status = bladerf_enable_module(dev, BLADERF_TX, false);

        bladerf_close(dev);

        std::cout << "Press Enter to close..." << std::endl;

        std::cin.ignore();
    }
    catch (std::exception e)
    {
        std::cout << "error: " << e.what() << std::endl;
        std::cin.ignore();
    }
    
    int bp = 1;
    
    return 0;
    
}   // end of main
