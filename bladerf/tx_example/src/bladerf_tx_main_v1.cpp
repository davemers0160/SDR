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
#include "iq_utils.h"

// Project Includes
#include <bladerf_common.h>

const double pi = 3.14159265358979323846;
const double pi2 = 3.14159265358979323846*2;

const std::complex<double> j(0, 1);

// ----------------------------------------------------------------------------
template<typename T>
inline std::vector<std::complex<T>> generate_bpsk_iq(std::vector<T> s, T amplitude)
{
    uint64_t idx;

    std::vector<std::complex<T>> data(s.size(), std::complex<T>(0, 0));

    for (idx = 0; idx < s.size(); ++idx)
    {
        data[idx] = std::complex<T>(amplitude * (2 * s[idx] - 1), 0);
    }

    return data;
}

// ----------------------------------------------------------------------------
template<typename T>
inline std::vector<T> maximal_length_sequence(uint16_t N, uint16_t rep, std::vector<uint16_t> taps = { 0, (uint16_t)(N - 1) })
{
    uint64_t idx, jdx;
    uint16_t tmp;
    std::vector<T> sr;

    //std::vector<uint16_t> taps = { 0, (uint16_t)(N - 1) };

    // initialize the register
    std::deque<uint8_t> r(N, 0);
    r[0] = 1;

    // shift register 
    uint64_t sr_size = (1 << N) - 1;

    for (idx = 0; idx < sr_size; ++idx)
    {
        //        sr.insert(sr.end(), rep, amplitude * (2 * r[N - 1] - 1));
        sr.insert(sr.end(), rep, r[N - 1]);

        tmp = 0;
        for (jdx = 0; jdx < taps.size(); ++jdx)
        {
            tmp += r[taps[jdx]];
        }
        tmp = tmp % 2;

        r.push_front(tmp);
        r.pop_back();
    }

    return sr;
}   // end of maximal_length_sequence


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
    bladerf_sample_rate sample_rate = 1000000;     // 10 MHz
    bladerf_channel rx = BLADERF_CHANNEL_RX(0);
    bladerf_channel tx = BLADERF_CHANNEL_TX(0);
    bladerf_frequency tx_freq = 2500188000;// 314300000;
    //bladerf_frequency tx_freq = 2375188000;// 314300000;
    //bladerf_frequency tx_freq = 2250188000;// 314300000;
    bladerf_bandwidth tx_bw = 1000000;
    bladerf_gain tx1_gain = 60000;

    std::vector<int16_t> samples;
    uint64_t num_samples;
    const uint32_t num_buffers = 16;
    const uint32_t buffer_size = 1024*4;        // must be a multiple of 1024
    const uint32_t num_transfers = 8;
    uint32_t timeout_ms = 5000;

    //std::vector<uint8_t> data;
    //std::vector<int16_t> iq_data;
    std::vector<std::complex<int16_t>> iq_data;
    std::complex<double> tmp_val;

    //generate IQ samples - simple FSK
    uint64_t data = 0xAB42F58C15ACFE37;     // random 64-bit data: 0xAB42F58C15ACFE37
    uint64_t bit_mask = 1;

    // the number of samples per bit
    double bit_length = 1e-4;
    uint32_t bit_samples = (uint32_t)(sample_rate * bit_length);

    // the frequency offset for the FSK modulation, normalized by the sample rate
    double freq_offset = 200000.0 / sample_rate;

    double amplitude = 2000;

    for (idx = 0; idx < sizeof(data) * 8; ++idx)
    {
        // if true this is a one
        if (data & bit_mask)
        {
            for (jdx = 0; jdx < bit_samples; ++jdx)
            {
                tmp_val = amplitude * (std::exp(j * pi2 * freq_offset * (double)jdx));
                iq_data.push_back(std::complex<int16_t>(tmp_val.real(), tmp_val.imag()));
            }
        }
        // if not this is a zero
        else
        {
            for (jdx = 0; jdx < bit_samples; ++jdx)
            {
                tmp_val = amplitude * (std::exp(j * pi2 * (-freq_offset) * (double)jdx));
                iq_data.push_back(std::complex<int16_t>(tmp_val.real(), tmp_val.imag()));
            }
        }

        bit_mask <<= 1;

    }




    //std::string filename = "D:/data/fm_test_1M.sc16";

    //read_iq_data(filename, iq_data);


    // the number of IQ samples is the number of samples divided by 2
    num_samples = iq_data.size();

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

        while (idx<50000)
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
