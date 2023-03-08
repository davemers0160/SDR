// ----------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)

#elif defined(__linux__)

#endif
// ArrayFire Includes
//#include <arrayfire.h>

#include <cstdint>
#include <csignal>
#include <cmath>
#include <iostream>
#include <sstream>
#include <complex>

// bladeRF includes
#include <libbladeRF.h>
#include <bladeRF2.h>

// Custom Includes
#include "num2string.h"
#include "get_current_time.h"
#include "file_parser.h"
#include "file_ops.h"
#include "dsp/dsp_windows.h"


// Project Includes
#include <sdr_functions.h>
#include <bladerf_common.h>

const double pi = 3.14159265358979323846;
const double pi2 = 3.14159265358979323846*2;

static bool is_running;

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


// ----------------------------------------------------------------------------
int main(int argc, char** argv)
{

    uint64_t idx, jdx;
    
    // bladeRF variable
    struct bladerf_devinfo *device_list = NULL;
    struct bladerf_devinfo dev_info;
    struct bladerf* dev;
    int bladerf_num;
    int blade_status;
    bladerf_sample_rate sample_rate = 20000000;     // 10 MHz
    bladerf_channel rx = BLADERF_CHANNEL_RX(0);
    bladerf_channel tx = BLADERF_CHANNEL_TX(0);
    bladerf_frequency tx_freq = 915000000;// 314300000;
    bladerf_bandwidth tx_bw = 50000000;
    bladerf_gain tx1_gain = 55000;

    std::vector<int16_t> samples;
    uint64_t num_samples;
    const uint32_t num_buffers = 16;
    const uint32_t buffer_size = 1024*3;        // must be a multiple of 1024
    const uint32_t num_transfers = 8;
    uint32_t timeout_ms = 10000;

    int16_t amplitude = 1500;

    // ----------------------------------------------------------------------------
    // generate a sequence of 1's and 0's
    std::vector<int16_t> seq = maximal_length_sequence<int16_t>(7, { 0, 3, 4, 6 });

    // take the MLS sequence and create an unfiltered BPSK IQ signal
    std::vector<std::complex<double>> bpsk_iq = generate_bpsk_iq<double>(seq);

    // at this point the BPSK signal has no relation to the SDR or samplerate used to transmit the signal
    // this is the number of seconds that each "bit" should last
    double bit_time = 1e-6;
    // the number of samples per bit
    uint32_t samples_per_bit = (uint32_t)(20000000 * bit_time);
    
    // expand the data based on the bit time and samplerate
    std::vector<std::complex<double>> iq_data;
    iq_data.reserve(samples_per_bit * bpsk_iq.size());
    for (idx = 0; idx < bpsk_iq.size(); ++idx)
    {
        iq_data.insert(iq_data.end(), samples_per_bit, bpsk_iq[idx]);
    }

    // pad the iq data with zeros for sending the 
    uint32_t buffer_diff = (iq_data.size() > buffer_size) ? iq_data.size() % buffer_size : buffer_size - iq_data.size();
    iq_data.insert(iq_data.end(), buffer_diff, std::complex<double>(0,0));

    // ----------------------------------------------------------------------------
    // window size
    int64_t n_taps = 301;

    // filter cutoff frequency
    float fc = 2.0e6 / (float)sample_rate;

    // create the low pass filter
    std::vector<double> lpf = DSP::create_fir_filter<double>(n_taps, fc, &DSP::nuttall_window);

    //----------------------------------------------------------------------------
    // take the base data and scale to the desired amplitude
    std::vector<std::complex<int16_t>> x0(iq_data.size(), std::complex<int16_t>(0, 0));
    for (idx = 0; idx < iq_data.size(); ++idx)
    {
        x0[idx] = amplitude * std::complex<int16_t>(iq_data[idx]);
    }

    //----------------------------------------------------------------------------
    // apply filter
    uint64_t offset = n_taps >> 1;

    std::vector<std::complex<double>> tmp_data = iq_data;
    tmp_data.insert(tmp_data.begin(), offset, 0);
    tmp_data.insert(tmp_data.end(), offset, 0);

    std::complex<double> accum;
    std::vector<std::complex<int16_t>> x1(iq_data.size(), std::complex<int16_t>(0,0));

    // apply the filter using convolution
    // assumes that the filter is odd and symetric, i.e. same valules forward and reverse
    for (idx = offset; idx < (tmp_data.size() - offset); ++idx)
    {
        accum = 0.0;

        for (jdx = 0; jdx < n_taps; ++jdx)
        {
            std::complex<double> t1 = std::complex<double>(lpf[jdx],0);
            std::complex<double> t2 = tmp_data[idx + jdx - offset];
            accum +=  t1 * t2;
        }

        x1[idx - offset] = (double)amplitude * accum;
    }

    //----------------------------------------------------------------------------
    // frequency rotation
    double f_offset = 2e6;
    std::vector<std::complex<double>> f_rot = create_freq_rotation(iq_data.size(), (f_offset / (double)sample_rate));

    // apply frequency rotation
    std::vector<std::complex<int16_t>> x1_r(x1.size(), std::complex<int16_t>(0, 0));
    for (idx = 0; idx < x1.size(); ++idx)
    {
        std::complex<double> tmp = std::complex<double>(x1[idx].real(), x1[idx].imag());
        x1_r[idx] = std::complex<int16_t>(f_rot[idx] * tmp);
    }

    // the number of IQ samples is the number of complex samples
    num_samples = x0.size();

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

        is_running = true;

        if (signal(SIGINT, sig_handler) == SIG_ERR)
        {
            std::cerr << "Unable to catch SIGINT signals" << std::endl;
        }

        
        uint32_t result;

        std::string console_input;
        std::cout << "Select waveform: " << std::endl;
        std::cout << "0 - Unfiltered BPSK" << std::endl;
        std::cout << "1 - Filtered BPSK" << std::endl;
        std::cout << "2 - Rotated & Filtered BPSK" << std::endl;

        while(is_running)
        {

            std::getline(std::cin, console_input);
            result = std::stoi(console_input);

            std::vector<std::complex<int16_t>> x;
            switch (result)
            {
            case 0:
                std::copy(x0.begin(), x0.end(), back_inserter(x));
                break;

            case 1:
                std::copy(x1.begin(), x1.end(), back_inserter(x));
                break;

            case 2:
                std::copy(x1_r.begin(), x1_r.end(), back_inserter(x));
                break;

            default:
                std::copy(x0.begin(), x0.end(), back_inserter(x));
                break;
            }

            idx = 0;
            while (idx < 10000)
            {
                blade_status = bladerf_sync_tx(dev, (int16_t*)x.data(), (uint32_t)x.size(), NULL, timeout_ms);
                //blade_status = bladerf_sync_tx(dev, (int16_t*)xn.data(), (uint32_t)xn.size(), NULL, timeout_ms);

                if (blade_status != 0)
                {
                    std::cout << "Unable to get the required number of samples: " << std::string(bladerf_strerror(blade_status)) << std::endl;
                    return blade_status;
                }

                //std::cout << "Sending signal #" << idx << std::endl;
                ++idx;
            }

            std::cout << "Done sending signals..." << std::endl;

        }   // end of while(is_running)

        std::cout << std::endl << "Closing BladeRF..." << std::endl;

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
