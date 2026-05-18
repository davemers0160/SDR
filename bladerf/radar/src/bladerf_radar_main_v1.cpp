// ----------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
#define _USE_MATH_DEFINES

#elif defined(__linux__)

#endif


#include <csignal>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <deque>
#include <atomic>
#include <thread>
#include <complex>

// bladeRF includes
#include <libbladeRF.h>
#include <bladeRF2.h>

// Custom Includes
#include "num2string.h"
#include "get_current_time.h"
#include "file_parser.h"
#include "file_ops.h"

#include <dsp/dsp_filtering.h>

// Project Includes
#include <bladerf_common.h>

// ----------------------------------------------------------------------------
// Globals
std::atomic<bool> is_running(false);
std::atomic<bool> rx_run(false);
std::atomic<bool> rx_complete(false);

uint32_t timeout_ms = 10000;

//-----------------------------------------------------------------------------
void signal_handler(int sig_num)
{
    if ((sig_num == SIGINT) || (sig_num == SIGTERM))
    {
        //fprintf(stderr, "received SIGINT\n");
        std::cout << std::endl << "Received SIGINT/SIGTERM: " << sig_num << std::endl << "Shutting down..." << std::endl;
        is_running = false;
        rx_run = false;
    }
}   // end of signal_handler

// ----------------------------------------------------------------------------
template<typename T>
void save_complex_data(std::string filename, std::vector<std::complex<T>> data)
{
    std::ofstream data_file;

    //T r, q;

    data_file.open(filename, ios::out | ios::binary);

    if (!data_file.is_open())
    {
        std::cout << "Could not save data. Closing... " << std::endl;
        //std::cin.ignore();
        return;
    }

    data_file.write(reinterpret_cast<const char*>(data.data()), 2 * data.size() * sizeof(T));
    data_file.close();
}

// ----------------------------------------------------------------------------
std::vector<std::complex<int16_t>> generate_lfm_chirp(int64_t f_start, int64_t f_stop, uint64_t fs, double signal_length, int16_t amplitude)
{
    uint64_t idx;

    uint64_t N = (uint64_t)(fs * signal_length);
    std::vector<std::complex<int16_t>> iq(N, std::complex<int16_t>(0, 0));
    std::complex<double> tmp;
    double t = 1.0 / fs;

    for (idx = 0; idx < N; ++idx)
    {
        tmp = exp(1i * 2.0 * M_PI * (f_start * idx * t + (f_stop - f_start) * 0.5 * idx * idx * t * t / signal_length));
        iq[idx] = std::complex<int16_t>(amplitude * tmp.real(), amplitude * tmp.imag());
    }

    return iq;
}

// ----------------------------------------------------------------------------
template<typename T>
inline std::vector<T> maximal_length_sequence(uint16_t N, uint16_t rep, std::vector<uint16_t> taps)
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
template<typename T>
inline std::vector<std::complex<T>> generate_bpsk_iq(std::vector<T> s, T amplitude)
{
    uint64_t idx;

    std::vector<std::complex<T>> data(s.size(), std::complex<T>(0,0));

    for (idx = 0; idx < s.size(); ++idx)
    {
        data[idx] = std::complex<T>(amplitude * (2 * s[idx] - 1), 0);
    }

    return data;
}

// ----------------------------------------------------------------------------
template<typename T>
inline std::vector<std::complex<T>> generate_cw_pulse(uint64_t sample_rate, double pw, double pri, T amplitude)
{
    uint64_t idx;
    uint64_t pw_samples = floor(sample_rate * pw + 0.5);
    uint64_t pri_samples = floor(sample_rate * pri + 0.5);
    
    std::vector<std::complex<T>> iq(pri_samples, std::complex<T>(0,0));

    for (idx = 0; idx < pw_samples; ++idx)
    {
        iq[idx] = std::complex<T>(amplitude, 0);
    }

    return iq;

}

// ----------------------------------------------------------------------------
inline void RX(struct bladerf* dev, std::vector<std::complex<int16_t>> &samples)
{
    int blade_status;
    uint32_t num_samples = samples.size();

    while (rx_run)
    {
        if (rx_complete == false)
        {
            std::cout << "RX Started..." << std::endl;
            blade_status = bladerf_sync_rx(dev, (void*)samples.data(), num_samples, NULL, timeout_ms);
            if (blade_status != 0)
            {
                std::cout << "Unable to get the required number of samples: " << std::string(bladerf_strerror(blade_status)) << std::endl;
                rx_run = false;
            }

            rx_complete = true;
        }

    }
}

// ----------------------------------------------------------------------------
inline void TX(struct bladerf* dev, std::vector<std::complex<int16_t>>& samples)
{
    uint32_t num_samples = samples.size() * 2;

    int blade_status = bladerf_sync_tx(dev, (int16_t*)samples.data(), num_samples, NULL, timeout_ms);

    if (blade_status != 0)
    {
        std::cout << "Unable to get the required number of samples: " << std::string(bladerf_strerror(blade_status)) << std::endl;
        //return blade_status;
    }
}

// ----------------------------------------------------------------------------
int main(int argc, char** argv)
{

    uint32_t idx;
    int32_t bp = 0;
    
    // bladeRF variable
    struct bladerf_devinfo *device_list = NULL;
    struct bladerf_devinfo dev_info;
    struct bladerf* dev;
    int bladerf_num;
    int blade_status;
    bladerf_channel rx = BLADERF_CHANNEL_RX(0);
    bladerf_channel tx = BLADERF_CHANNEL_TX(0);
    bladerf_frequency rx_freq = 2500000000; //162425000;
    bladerf_frequency tx_freq = 2500000000; //162425000;
    bladerf_sample_rate rx_fs = 20000000;
    bladerf_sample_rate tx_fs = 20000000;
    bladerf_bandwidth rx_bw = 5000000;
    bladerf_bandwidth tx_bw = 5000000;
    bladerf_gain rx1_gain = 60;
    bladerf_gain tx1_gain = 60;

    const uint32_t num_buffers = 16;
    const uint32_t buffer_size = 1024;        // must be a multiple of 1024
    const uint32_t num_transfers = 8;
    double t = 0.02;
    int16_t amplitude = 2040;

    if (argc != 2)
    {
        std::cout << "enter parameter file..." << std::endl;
        std::cin.ignore();
        return 0;
    }

    std::string param_filename = argv[1];
    //read_bladerf_params(param_filename, rx_freq, rx_fs, rx_bw, rx1_gain, &t);

    uint64_t num_rx_samples = (uint64_t)floor(rx_fs * t);
    std::vector<std::complex<int16_t>> rx_samples(num_rx_samples);

    int num_devices = bladerf_get_device_list(&device_list);

    bladerf_num = select_bladerf(num_devices, device_list);

    if (bladerf_num < 0)
    {
        std::cout << "could not detect any bladeRF devices..." << std::endl;
        std::cin.ignore();
        return 0;
    }

    std::cout << std::endl;

    try
    {
      
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
        std::cout << std::endl << dev_info;
        //std::cout << "FPGA: " << 

        // set the frequency, sample_rate and bandwidth
        blade_status = bladerf_set_frequency(dev, rx, rx_freq);
        blade_status = bladerf_get_frequency(dev, rx, &rx_freq);
        blade_status = bladerf_set_frequency(dev, tx, tx_freq);
        blade_status = bladerf_get_frequency(dev, tx, &tx_freq);

        // configure the sync to receive/transmit data
        blade_status = bladerf_sync_config(dev, BLADERF_RX_X1, BLADERF_FORMAT_SC16_Q11, num_buffers, buffer_size, num_transfers, timeout_ms);
        blade_status = bladerf_sync_config(dev, BLADERF_TX_X1, BLADERF_FORMAT_SC16_Q11, num_buffers, buffer_size, num_transfers, timeout_ms);

        blade_status = bladerf_set_sample_rate(dev, rx, rx_fs, &rx_fs);
        blade_status = bladerf_set_bandwidth(dev, rx, rx_bw, &rx_bw); 
        blade_status = bladerf_set_sample_rate(dev, tx, tx_fs, &tx_fs);
        blade_status = bladerf_set_bandwidth(dev, tx, tx_bw, &tx_bw);  
        
        // enable the rx & tx channel RF frontend
        blade_status = bladerf_enable_module(dev, BLADERF_RX, true);
        blade_status = bladerf_enable_module(dev, BLADERF_TX, true);

        // the gain must be set after the module has been enabled
        blade_status = bladerf_set_gain_mode(dev, rx, BLADERF_GAIN_MGC);
        blade_status = bladerf_set_gain(dev, rx, rx1_gain);
        blade_status = bladerf_get_gain(dev, rx, &rx1_gain);
        blade_status = bladerf_set_gain_mode(dev, tx, BLADERF_GAIN_MGC);
        blade_status = bladerf_set_gain(dev, tx, tx1_gain);
        blade_status = bladerf_get_gain(dev, tx, &tx1_gain);

        // setup signal handler
        is_running = true;

        //-----------------------------------------------------------------------------
        // initialize a signal handler for graceful exit
        if (signal(SIGINT, signal_handler) == SIG_ERR)
        {
            std::cerr << "Unable to catch SIGINT signals" << std::endl;
        }
        // handle SIGTERM signals
        if (signal(SIGTERM, signal_handler) == SIG_ERR)
        {
            std::cerr << "Unable to catch SIGTERM signals" << std::endl;
        }
        
        //-----------------------------------------------------------------------------
        // create the transmit samples
        // c = 299792458 m/s ==> 1/c = 3.3356409519815204957557671447492e-9 s/m
        //
        // Radar Minimum detectable distance equation
        // R_min = ((pw + tr)*c)/2
        // where pw is the pulse width, tr is the time to switch between tx and rx,
        // 
        // Radar maximum unambiguous detection range
        // R_max = (pri * c)/2 
        //-----------------------------------------------------------------------------

        //-----------------------------------------------------------------------------
        // build a simple CW pulse with a given pulse width and pulse repetition interval
        // minimum detectable range: 14.9896229 m
        // maximum unambiguous detection range: 149896.229 m
        double pw = 0.1e-6;
        double pri = 1.0e-3;
        
        // create a single pulse
        std::vector<std::complex<int16_t>> iq_pulse = generate_cw_pulse(tx_fs, pw, pri, amplitude);
        std::vector<double> iq_pulse_d(iq_pulse.size());

        for (idx = 0; idx < iq_pulse.size(); ++idx)
        {
            iq_pulse_d[idx] = (double)(iq_pulse[idx].real());
        }
        
        // create a multiple pulse group
        int32_t num_pulses = 10;
        std::vector<std::complex<int16_t>> tx_pulses;
        
        // Pre-allocate space for all copies
        tx_pulses.reserve(iq_pulse.size() * num_pulses);
        
        // Loop and insert copies
        for(idx = 0; idx < num_pulses; ++idx) 
        {
            tx_pulses.insert(tx_pulses.end(), iq_pulse.begin(), iq_pulse.end());
        }

        //-----------------------------------------------------------------------------
        rx_run = true;
        rx_complete = true;

        // start the rx thread
        std::thread rx_thread(RX, dev, std::ref(rx_samples));

        // collect some junk samples to clear the buffer
        rx_complete = false;
        while (rx_complete == false);

        // print out the specifics
        std::cout << std::endl << "------------------------------------------------------------------" << std::endl;
        std::cout << "Receiver: " << std::endl;
        std::cout << "  freq:     " << rx_freq << std::endl;
        std::cout << "  fs:       " << rx_fs << std::endl;
        std::cout << "  bw:       " << rx_bw << std::endl;
        std::cout << "  rx1_gain: " << rx1_gain << std::endl;
        std::cout << "Transmitter: " << std::endl;
        std::cout << "  freq:     " << tx_freq << std::endl;
        std::cout << "  fs:       " << tx_fs << std::endl;
        std::cout << "  bw:       " << tx_bw << std::endl;
        std::cout << "  tx1_gain: " << tx1_gain << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl << std::endl;


        while(is_running == true)
        {
            // start collecting samples
            rx_complete = false;

            //for (uint32_t jdx = 0; jdx < 30; ++jdx)
            TX(dev, tx_pulses);

            // wait for the rx cycle to complete
            while (rx_complete == false)
                std::cout << ".";

            std::cout << std::endl;

            std::cout << "Press enter to continue...";
            std::cin.ignore();
            
            // start processing samples
            //std::vector<std::complex<int16_t>> filtered_rx_iq = DSP::apply_fir_filter(rx_samples, iq_pulse_d);
            
            bp = 3;

        }

        rx_run = false;
        rx_thread.join();

        // disable the rx channel RF frontend
        blade_status = bladerf_enable_module(dev, BLADERF_RX, false);
        blade_status = bladerf_enable_module(dev, BLADERF_TX, false);

        bladerf_close(dev);
    }
    catch(std::exception e)
    {
        std::cout << "error: " << e.what() << std::endl;
        std::cin.ignore();
    }
    
    return 0;
    
}   // end of main
