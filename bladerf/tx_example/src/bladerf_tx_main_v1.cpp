// ----------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)

#elif defined(__linux__)

#endif
// ArrayFire Includes
//#include <arrayfire.h>

#include <cstdint>
#include <iostream>
#include <sstream>
#include <complex>
#include <cmath>

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

const double pi = 3.14159265358979323846;
const double pi2 = 3.14159265358979323846*2;

// ----------------------------------------------------------------------------
int main(int argc, char** argv)
{

    uint32_t idx, jdx;
    
    // bladeRF variable
    struct bladerf_devinfo *device_list = NULL;
    struct bladerf_devinfo dev_info;
    struct bladerf* dev;
    int bladerf_num;
    int blade_status;
    bladerf_sample_rate sample_rate = 10000000;     // 10 MHz
    bladerf_channel rx = BLADERF_CHANNEL_RX(0);
    bladerf_channel tx = BLADERF_CHANNEL_TX(0);
    bladerf_frequency tx_freq = 1694000000;// 314300000;
    bladerf_bandwidth tx_bw = 2000000;
    bladerf_gain tx1_gain = 00;

    std::vector<int16_t> samples;
    uint64_t num_samples;
    const uint32_t num_buffers = 16;
    const uint32_t buffer_size = 1024*8;        // must be a multiple of 1024
    const uint32_t num_transfers = 8;
    uint32_t timeout_ms = 10000;

    std::vector<uint8_t> data;
    std::vector<int16_t> iq_data;

    std::string message = "Hello!";

    // generate the data that will be transmitted
    for (idx = 0; idx < 19; ++idx)
    {
        data.push_back(1);
    }
    data.push_back(0);

    for (idx = 0; idx < message.length(); ++idx)
    {
        for (jdx = 0; jdx < 8; ++jdx)
        {
            data.push_back((message[idx] >> (8 - jdx)) & 0x01);
        }

    }
        
    // ----------------------------------------------------------------------------
    // create the bits in RF
    // the number of samples per bit
    uint32_t bit_samples = 50000;

    // the frequency offset for the FSK modulation - 200kHz
    double freq_offset = 100000;

    // vectors to store the IQ representation of a 0 and a 1
    std::vector<int16_t> IQ_1, IQ_0;

    // generate the samples - consists of one I and one Q.  The data should be packed IQIQIQIQIQIQ...
    for (idx = 0; idx < bit_samples; ++idx)
    {
        IQ_1.push_back((int16_t)(1200 * (cos((-2.0 * pi * freq_offset * idx) / (double)sample_rate))));
        IQ_1.push_back((int16_t)(1200 * (sin((-2.0 * pi * freq_offset * idx) / (double)sample_rate))));
        IQ_0.push_back((int16_t)(1200 * (cos((2.0 * pi * freq_offset * idx) / (double)sample_rate))));
        IQ_0.push_back((int16_t)(1200 * (sin((2.0 * pi * freq_offset * idx) / (double)sample_rate))));
    }

    // run through each data bit and map the bit_samples of IQ data
    for (idx = 0; idx < data.size(); ++idx)
    {
        if (data[idx] == 1)
        {
            iq_data.insert(iq_data.end(), IQ_1.begin(), IQ_1.end());
        }
        else
        {
            iq_data.insert(iq_data.end(), IQ_0.begin(), IQ_0.end());
        }
    }

    // the number of IQ samples is the number of samples divided by 2
    num_samples = iq_data.size() >> 1;

    // ----------------------------------------------------------------------------
    int num_devices = bladerf_get_device_list(&device_list);

    bladerf_num = select_bladerf(num_devices, device_list);

    if (bladerf_num < 0)
    {
        std::cout << "could not detect any bladeRF devices..." << std::endl;
        return 0;
    }

    std::cout << std::endl;

    try{

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
        blade_status = bladerf_set_frequency(dev, tx, tx_freq);
        blade_status = bladerf_set_sample_rate(dev, tx, sample_rate, &sample_rate);
        blade_status = bladerf_set_bandwidth(dev, tx, tx_bw, &tx_bw);

        // the gain 
        //blade_status = bladerf_set_gain_mode(dev, tx, BLADERF_GAIN_MANUAL);
        //blade_status = bladerf_set_gain(dev, tx, tx1_gain);

        // configure the sync to receive/transmit data
        blade_status = bladerf_sync_config(dev, BLADERF_TX_X1, BLADERF_FORMAT_SC16_Q11, num_buffers, buffer_size, num_transfers, timeout_ms);
        
        if (blade_status != 0) 
        {
            std::cout << "Failed to configure TX sync interface: " << bladerf_strerror(blade_status) << std::endl;
        }

        // enable the TX channel RF frontend
        blade_status = bladerf_enable_module(dev, BLADERF_TX, true);

        idx = 0;

        while (idx<500)
        {
            blade_status = bladerf_sync_tx(dev, (int16_t*)iq_data.data(), num_samples, NULL, timeout_ms);

            if (blade_status != 0)
            {
                std::cout << "Unable to get the required number of samples: " << std::string(bladerf_strerror(blade_status)) << std::endl;
                return blade_status;
            }

            std::cout << "Sending signal #" << idx << std::endl;
            ++idx;
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
