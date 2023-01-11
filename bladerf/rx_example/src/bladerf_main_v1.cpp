// ----------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)

#elif defined(__linux__)

#endif
// ArrayFire Includes
#include <arrayfire.h>

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
    struct bladerf_devinfo *device_list = NULL;
    struct bladerf_devinfo dev_info;
    struct bladerf* dev;
    int bladerf_num;
    int blade_status;
    bladerf_channel rx = BLADERF_CHANNEL_RX(0);
    bladerf_channel tx = BLADERF_CHANNEL_TX(0);
    bladerf_frequency rx_freq = 137000000; //162425000;
    bladerf_frequency rx_freqa = 0;
    bladerf_sample_rate fs = 624000;
    bladerf_bandwidth rx_bw = 624000;
    bladerf_gain rx1_gain = 64;
    int64_t span = 624000;

    std::vector<int16_t> samples;
    uint32_t num_samples = 65536*2;
    uint32_t timeout_ms = 10000;
    const uint32_t num_buffers = 16;
    const uint32_t buffer_size = 1024 * 4 * 8;        // must be a multiple of 1024
    const uint32_t num_transfers = 8;
    double t;

    if (argc == 2)
    {
        std::string param_filename = argv[1];

        read_bladerf_params(param_filename, rx_freq, fs, rx_bw, rx1_gain);
    }

#ifdef USE_ARRAYFIRE
    af::setBackend(AF_BACKEND_CPU);
    af::info();

    std::cout << std::endl << std::endl;

    // array fire variables
    af::array raw_data, fft_data;

#endif // USE_ARRAYFIRE

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
        blade_status = bladerf_set_sample_rate(dev, rx, fs, &fs);
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
        samples.resize(num_samples*2);

        span = fs;
        double freq_step = (fs)/(double)num_samples;

        double f_min = (rx_freq - (span>>1)) * 1.0e-6;
        double f_max = ((rx_freq-1) + (span>>1)) * 1.0e-6;

        uint32_t sp = (uint32_t)((fs - span) / (2.0 * freq_step));
        uint32_t sp2 = (uint32_t)(span / freq_step);

        double scale = 1.0 / (double)(num_samples);


        // print out the specifics
        std::cout << std::endl << "------------------------------------------------------------------" << std::endl;
        std::cout << "fs:       " << fs << std::endl;
        std::cout << "rx_freq:  " << rx_freqa << std::endl;
        std::cout << "rx_bw:    " << rx_bw << std::endl;
        std::cout << "rx1_gain: " << rx1_gain << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl << std::endl;

#ifdef USE_ARRAYFIRE

        af::Window myWindow(800, 800, "FFT example: ArrayFire");
        //af::array X = af::seq(0, num_samples - 1, 1);
        af::array X = af::seq(sp+1, (sp+sp2), 1);
        //auto x2d = X.dims(0);

        //af::array f = af::seq(f_min, f_max - (freq_step * 1.0e-6), (freq_step * 1.0e-6));
        af::array f = af::seq(f_min, f_max, (freq_step * 1.0e-6));

        myWindow.setAxesLimits(f_min, f_max - (freq_step * 1.0e-6), -120, -20, true);
        myWindow.setAxesTitles("Frequency (MHz)", "Power (dBm)");

        while (!myWindow.close())

#else

        while(1)
#endif

        {
            blade_status = bladerf_sync_rx(dev, (void*)samples.data(), num_samples, NULL, timeout_ms);
            if (blade_status != 0)
            {
                std::cout << "Unable to get the required number of samples: " << std::string(bladerf_strerror(blade_status)) << std::endl;
                return blade_status;
            }

            // try to compute the fft using the arrayfire library
            std::vector<std::complex<float>> c_samples(num_samples);
            int index = 0;
            for (idx = 0; idx < samples.size(); idx += 2)
            {
                c_samples[index++] = std::complex<float>((float)samples[idx], (float)samples[idx + 1]);
            }

#ifdef USE_ARRAYFIRE
            raw_data = af::array(num_samples, (af::cfloat*)c_samples.data())*(1/2048.0);

            af::fftInPlace(raw_data, scale);
            //auto a1 = af::abs(raw_data).host<float>();

            fft_data = 20 * af::log10(af::shift(af::abs(raw_data), (num_samples >> 1)));

            auto x_dim = X.dims();
            auto f_dim = f.dims();
            auto fft_dims = fft_data.dims();

            // show the results of the FFT in the window
            myWindow.plot(f, fft_data(X));
            //myWindow.show();
#endif
        }

        // disable the rx channel RF frontend
        blade_status = bladerf_enable_module(dev, BLADERF_RX, false);

        bladerf_close(dev);
    }

#ifdef USE_ARRAYFIRE
    catch (af::exception e)
    {
        std::cout << "error: " << e.what() << std::endl;
        std::cin.ignore();
    }
#else
    catch(std::exception e)
    {
        std::cout << "error: " << e.what() << std::endl;
    }
#endif

    
    return 0;
    
}   // end of main
