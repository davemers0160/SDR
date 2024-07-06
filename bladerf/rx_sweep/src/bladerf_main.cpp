// ----------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS

#include <ryml_all.hpp>

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)

#elif defined(__linux__)

#endif

#include <cstdint>
#include <iostream>
#include <sstream>
#include <vector>
#include <utility>

// bladeRF includes
#include <libbladeRF.h>
#include <bladeRF2.h>

// Custom Includes
#include "num2string.h"
#include "get_current_time.h"

// Project Includes
#include "bladerf_common.h"
#include "parse_blade_input.h"


std::string convert_metric_prefix(double num)
{
    uint32_t idx;
    std::vector<std::pair<double, std::string>> prefix = { std::make_pair(1e-12, "p"), std::make_pair(1e-9, "n"), std::make_pair(1e-6, "u"), std::make_pair(1e-3, "m"), std::make_pair(1, "N"), std::make_pair(1e3, "K"), std::make_pair(1e6, "M"), std::make_pair(1e9, "G"), std::make_pair(1e12, "T")};

    double p = 0.0;
    double s = 0.0;
    double v = 0.0;


    // find which prefix returns a whole number
    for (idx = 0; idx < prefix.size(); ++idx)
    {
        v = num/prefix[idx].first;
        
        if (abs(v) < 1.0)
        {
            break;
        }
    }

    if(idx > 0)
        --idx;

    v = (num / prefix[idx].first);

    if(v < 0)
        p = ceil(num / prefix[idx].first);
    else
        p = floor(num / prefix[idx].first);

    s = abs(floor((v-p)*1000 + 0.5));

    std::string mp = num2str((int64_t)p, "%d") + prefix[idx].second + num2str((int64_t)s, "%03d");

    return mp;

}

// ----------------------------------------------------------------------------
int main(int argc, char** argv)
{

    uint32_t idx;
    std::string sdate, stime;

    // bladeRF variable
    struct bladerf_devinfo *device_list = NULL;
    struct bladerf_devinfo dev_info;
    struct bladerf* dev;
    int bladerf_num;
    int blade_status;

    bladerf_channel rx = BLADERF_CHANNEL_RX(0);
    bladerf_frequency rx_freq = 137000000;
    bladerf_sample_rate sample_rate;
    bladerf_bandwidth rx_bw;
    bladerf_gain rx1_gain = 20;
    
    uint32_t timeout_ms = 10000;
    const uint32_t num_buffers = 16;
    const uint32_t buffer_size = 1024 * 4;        // must be a multiple of 1024
    const uint32_t num_transfers = 8;

    double duration;
    uint32_t num_samples;

    std::vector<bladerf_frequency> rx_freq_range;
    std::vector<int16_t> samples;

    std::string param_filename;
    std::string save_location;
    std::string file_name;

    std::ofstream data_file;

    if (argc < 2)
    {
        std::cout << "You must supply a yaml parameter file.  Press Enter to close." << std::endl;
        std::cin.ignore();
        return 0;
    }

    param_filename = argv[1];
    parse_input(param_filename, rx_freq_range, sample_rate, duration, rx1_gain, save_location);

    num_samples = (uint64_t)(sample_rate * duration);
    rx_bw = sample_rate;

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
        get_current_time(sdate, stime);

      
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
        blade_status = bladerf_set_frequency(dev, rx, rx_freq_range[0]);
        blade_status = bladerf_get_frequency(dev, rx, &rx_freq);
        blade_status = bladerf_set_sample_rate(dev, rx, sample_rate, &sample_rate);
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

        // print out the specifics
        std::cout << std::endl << "------------------------------------------------------------------" << std::endl;
        std::cout << "sample_rate: " << sample_rate << std::endl;
        std::cout << "rx_freq:     " << rx_freq_range[0] << std::endl;
        std::cout << "rx_bw:       " << rx_bw << std::endl;
        std::cout << "rx1_gain:    " << rx1_gain << std::endl;
        std::cout << "time:        " << duration << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl << std::endl;

        // collect some dummy samples
        blade_status = bladerf_sync_rx(dev, (int16_t*)samples.data(), 2 * buffer_size, NULL, timeout_ms);

        std::cout << "Ready to record.  Press enter to start:" << std::endl;
        std::cin.ignore();

        for (idx = 0; idx < rx_freq_range.size(); ++idx)
        {
            blade_status = bladerf_set_frequency(dev, rx, rx_freq_range[idx]);
            blade_status = bladerf_get_frequency(dev, rx, &rx_freq);

            std::cout << "Starting capture: " << idx << " - " << num2str((double)(rx_freq / 1000000.0), "%5.4f") << "MHz" << std::endl;
            
            //file_name = "blade_F" + num2str((double)(rx_freq / 1000000.0), "%5.3f") + "M_SR" + num2str((double)(sample_rate / 1000000.0), "%05.3f") + "M_" + sdate + "_" + stime + ".sc16";
            file_name = "blade_F" + convert_metric_prefix((double)(rx_freq)) + "_SR" + convert_metric_prefix((double)(sample_rate)) + "_" + sdate + "_" + stime + ".sc16";

            data_file.open(save_location + file_name, ios::out | ios::binary);

            if (!data_file.is_open())
            {
                std::cout << "Could not save data. Closing... " << std::endl;
                std::cin.ignore();
                return 0;
            }

            // collect the samples
            blade_status = bladerf_sync_rx(dev, (int16_t*)samples.data(), num_samples, NULL, timeout_ms);
            if (blade_status != 0)
            {
                std::cout << "Unable to get the required number of samples: " << std::string(bladerf_strerror(blade_status)) << std::endl;
                return blade_status;
            }

            std::cout << "Capture complete!  Saving data..." << std::endl << std::endl;
            data_file.write(reinterpret_cast<const char*>(samples.data()), samples.size() * sizeof(int16_t));

            data_file.close();

        }

        // disable the rx channel RF frontend
        blade_status = bladerf_enable_module(dev, BLADERF_RX, false);

        bladerf_close(dev);
    }
    catch(std::exception e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }

    std::cout << "Complete! Press enter to close... " << std::endl;
    std::cin.ignore();

    return 0;
    
}   // end of main
