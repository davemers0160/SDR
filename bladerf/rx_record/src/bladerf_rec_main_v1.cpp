// ----------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)

#elif defined(__linux__)

#endif

#include <cstdint>
#include <iostream>
#include <sstream>

// bladeRF includes
#include <libbladeRF.h>
#include <bladeRF2.h>

// Custom Includes
#include "num2string.h"
#include "get_current_time.h"
#include "file_parser.h"
#include "file_ops.h"

// Project Includes
#include <bladerf_common.h>

// ----------------------------------------------------------------------------
int main(int argc, char** argv)
{

    uint32_t idx;
    
    // bladeRF variable
    int num_devices = 0;
    struct bladerf_devinfo* device_list = NULL;
    struct bladerf_devinfo dev_info;
    struct bladerf* dev;
    int bladerf_num;
    int blade_status;
    bladerf_channel rx = BLADERF_CHANNEL_RX(0);
    bladerf_channel tx = BLADERF_CHANNEL_TX(0);
    bladerf_frequency rx_freq = 137800000;// 96600000; //162400000;
    bladerf_sample_rate fs = 624000;
    bladerf_bandwidth rx_bw = 624000;
    bladerf_gain rx1_gain = 24;
    //int64_t span = 1000000;
    double t;         // number of seconds to record

    std::string filename = "../recordings/162M425_test.bin";
    std::ofstream data_file;

    std::vector<int16_t> samples;
    uint32_t num_samples;
    uint32_t timeout_ms = 10000;
    const uint32_t num_buffers = 16;
    const uint32_t buffer_size = 1024 * 4* 8;        // must be a multiple of 1024
    const uint32_t num_transfers = 8;

    try{    
        
        if (argc == 2)
        {
        //    filename = argv[1];
        //}
        //else if (argc == 3)
        //{
        //    filename = argv[1];

            std::string param_filename = argv[1];
            read_bladerf_params(param_filename, rx_freq, fs, rx_bw, rx1_gain, &t, &filename);
        }

        num_samples = (uint32_t)(fs * t);

        num_devices = bladerf_get_device_list(&device_list);

        bladerf_num = select_bladerf(num_devices, device_list);

        if (bladerf_num < 0)
        {
            std::cout << "could not detect any bladeRF devices..." << std::endl;
            return 0;
        }

        std::cout << std::endl;

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
        blade_status = bladerf_get_frequency(dev, rx, &rx_freq);
        blade_status = bladerf_set_sample_rate(dev, rx, fs, &fs);
        blade_status = bladerf_set_bandwidth(dev, rx, rx_bw, &rx_bw);

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
        samples.resize(num_samples*2);

        //double freq_step = (sample_rate)/(double)num_samples;

        //double f_min = (rx_freq - (span>>1)) * 1.0e-6;
        //double f_max = (rx_freq + (span>>1)) * 1.0e-6;

        //uint32_t sp = (uint32_t)((sample_rate - span) / (2.0 * freq_step));
        //uint32_t sp2 = (uint32_t)(span / freq_step);

        //double scale = 1.0 / (double)(num_samples);
                
        // print out the specifics
        std::cout << std::endl << "------------------------------------------------------------------" << std::endl;
        std::cout << "fs:       " << fs << std::endl;
        std::cout << "rx_freq:  " << rx_freq << std::endl;
        std::cout << "rx_bw:    " << rx_bw << std::endl;
        std::cout << "rx1_gain: " << rx1_gain << std::endl;
        std::cout << "time:     " << t << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl << std::endl;

        data_file.open(filename, ios::out | ios::binary);

        if (!data_file.is_open())
        {
            std::cout << "Could not save data. Closing... " << std::endl;
            std::cin.ignore(); 
            return 0;
        }

        std::cout << "Ready to record: " << filename << std::endl;
        std::cout << "Press enter to start:" << std::endl;
        std::cin.ignore();

        // collect some dummy samples
        blade_status = bladerf_sync_rx(dev, (void*)samples.data(), buffer_size, NULL, timeout_ms);

        std::cout << "Recording started..." << std::endl << std::endl;

        // collect the samples
        blade_status = bladerf_sync_rx(dev, (void*)samples.data(), num_samples, NULL, timeout_ms);
        if (blade_status != 0)
        {
            std::cout << "Unable to get the required number of samples: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            return blade_status;
        }

        std::cout << "Capture complete!  Saving data..." << std::endl;
        data_file.write(reinterpret_cast<const char*>(samples.data()), samples.size() * sizeof(int16_t));

        data_file.close();

        std::cout << "Data save complete!" << std::endl;

        // disable the rx channel RF frontend
        blade_status = bladerf_enable_module(dev, BLADERF_RX, false);

        // close the bladerf device
        bladerf_close(dev);

        //std::cout << "Recording complete!" << std::endl;
        std::cout << "Press Enter to close..." << std::endl;
        std::cin.ignore();

    }
    catch(std::exception e)
    {
        std::cout << "error: " << e.what() << std::endl;
        std::cin.ignore();

    }


    return 0;
    
}   // end of main
