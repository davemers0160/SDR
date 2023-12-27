// ----------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#include <ryml_all.hpp>

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)

#elif defined(__linux__)

#endif

#include <cstdint>
#include <cmath>
#include <csignal>
#include <iostream>
#include <sstream>
#include <complex>
#include <deque>


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

//const std::complex<double> j(0, 1);

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
void parse_input(std::string param_filename,
    uint64_t& center_freq,
    int64_t& freq_sep,
    uint32_t& sample_rate,
    uint32_t& bw,
    int32_t& tx1_gain,
    uint8_t &type,
    std::string& iq_file
)
{

    uint32_t idx;

    uint64_t start, stop, step;

    try {
        std::ifstream tmp_stream(param_filename);
        std::stringstream buffer;
        buffer << tmp_stream.rdbuf();
        std::string contents = buffer.str();
        tmp_stream.close();

        ryml::Tree config = ryml::parse_in_arena(ryml::to_csubstr(contents));

        // frequency step plan
        config["center_freq"] >> center_freq;
        config["freq_sep"] >> freq_sep;

        // sample rate
        config["sample_rate"] >> sample_rate;

        config["bandwidth"] >> bw;

        config["tx1_gain"] >> tx1_gain;

        config["signal_type"] >> type;

        config["iq_file"] >> iq_file;

    }
    catch (std::exception& e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }
}

// ----------------------------------------------------------------------------
template<typename T>
inline std::vector<std::complex<int16_t>> generate_bpsk(std::vector<T> s, int16_t amplitude)
{
    uint64_t idx, jdx;

    std::vector<std::complex<int16_t>> data(s.size(), std::complex<T>(0, 0));

    for (idx = 0; idx < s.size(); ++idx)
    {
        data[idx] = std::complex<int16_t>(amplitude * (2 * s[idx] - 1), 0);
    }

    return data;
}

// ----------------------------------------------------------------------------
inline std::vector<std::complex<int16_t>> generate_lfm_chirp(int64_t f_start, int64_t f_stop, uint64_t fs, double signal_length, int16_t amplitude)
{
    uint64_t idx;

    uint64_t N = (uint64_t)(fs * signal_length);
    std::vector<std::complex<int16_t>> iq(N, std::complex<int16_t>(0, 0));
    std::complex<double> tmp, v;

    double t = 1.0 / fs;

    for (idx = 0; idx < N; ++idx)
    {
        v = 1i * 2.0 * M_PI * (f_start * idx * t + (f_stop - f_start) * 0.5 * idx * idx * t * t / signal_length);
        tmp = std::exp(v);
        iq[idx] = std::complex<int16_t>(amplitude * tmp.real(), amplitude * tmp.imag());
    }

    return iq;
}

// ----------------------------------------------------------------------------
//function[iq] = generate_fsk(data, amplitude, sample_rate, bit_length, center_freq, freq_separation)
template<typename T>
inline std::vector<std::complex<int16_t>> generate_fsk(std::vector<T> data, double amplitude, uint64_t sample_rate, double bit_length, int64_t center_freq, int64_t freq_separation)
{
    uint32_t idx, jdx;
    std::complex<double> tmp_val, v;

    uint32_t samples_per_bit = floor(sample_rate * bit_length + 0.5);

    double f1 = (center_freq - freq_separation) / (double)sample_rate;
    double f2 = (center_freq + freq_separation) / (double)sample_rate;

    std::vector<std::complex<int16_t>> iq_data;

    for (idx = 0; idx < data.size(); ++idx)
    {
        if (data[idx] == 0)
        {
            for (jdx = 0; jdx < samples_per_bit; ++jdx)
            {
                v = 1i * M_PI * f1 * (double)jdx;
                tmp_val = amplitude * std::exp(v);
                iq_data.push_back(std::complex<int16_t>(tmp_val.real(), tmp_val.imag()));
            }
        }
        else
        {
            for (jdx = 0; jdx < samples_per_bit; ++jdx)
            {
                v = 1i * M_PI * f2 * (double)jdx;
                tmp_val = amplitude * std::exp(v);
                iq_data.push_back(std::complex<int16_t>(tmp_val.real(), tmp_val.imag()));
            }
        }

    }

    return iq_data;
}   // end of generate_fsk

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
    uint32_t timeout_ms = 10000;
    int64_t freq_separation;

    uint8_t signal_type;
    std::string iq_filename;

    std::vector<uint8_t> data;

    std::vector<std::complex<int16_t>> iq_data;
    
    if (argc != 2)
    {
        std::cout << "enter parameter file..." << std::endl;
        std::cin.ignore();
        return 0;
    }

    std::string param_filename = argv[1];
    parse_input(param_filename, tx_freq, freq_separation, sample_rate, tx_bw, tx1_gain, signal_type, iq_filename);

    //generate IQ samples - simple FSK
    uint64_t d = 0x2F58C15ACFE37AAA;     // random 64-bit data: 0xAB42F58C15ACFE37
    uint64_t bit_mask = 1;

    double amplitude = 2000;

    for (idx = 0; idx < sizeof(d)*8; ++idx)
    {
        data.push_back(d & bit_mask ? 1 : 0);
        bit_mask <<= 1;
    }

    double bit_length = 1e-4;
    auto seq = maximal_length_sequence<int16_t>(7, 5, { 0, 2 });

    switch (signal_type)
    {
    // fsk
    case 0:

        iq_data = generate_fsk(data, amplitude, sample_rate, bit_length, 0, freq_separation);
        break;

    case 1:
        iq_data = generate_lfm_chirp(-freq_separation, freq_separation, sample_rate, bit_length, amplitude);
        break;

    case 2:
        iq_data = generate_bpsk(seq, amplitude);
        break;

    case 3:
        read_iq_data(iq_filename, iq_data);
        break;

    default:
        break;
    }
    
    uint32_t num_buff = ceil(iq_data.size() / (double)buffer_size) + 2;

    iq_data.insert(iq_data.end(), num_buff * buffer_size - iq_data.size(), std::complex<int16_t>(0, 0));


    // the number of IQ samples is the number of samples divided by 2
    num_samples = iq_data.size();

    // get the libbladeRF API version
    struct bladerf_version version = { 0,0,0," " };
    bladerf_version(&version);
    std::cout << "libbladerf API version: " << std::string(version.describe) << std::endl;

    // ----------------------------------------------------------------------------
    int num_devices = bladerf_get_device_list(&device_list);

    bladerf_num = select_bladerf(num_devices, device_list);

    if (bladerf_num < 0)
    {
        std::cout << "could not detect any bladeRF devices..." << std::endl;
        cin.ignore();
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
        std::cout << dev_info;

        version;
        bladerf_fpga_version(dev, &version);
        std::cout << "  FPGA version:  " << std::string(version.describe) << std::endl;

        // set the frequency, sample_rate and bandwidth
        blade_status = bladerf_set_frequency(dev, tx, tx_freq);
        blade_status = bladerf_set_sample_rate(dev, tx, sample_rate, &sample_rate);
        blade_status = bladerf_set_bandwidth(dev, tx, tx_bw, &tx_bw);

        // configure the sync to receive/transmit data
        blade_status = bladerf_sync_config(dev, BLADERF_TX_X1, BLADERF_FORMAT_SC16_Q11, num_buffers, buffer_size, num_transfers, timeout_ms);
        if (blade_status != 0) 
        {
            std::cout << "Failed to configure TX sync interface: " << bladerf_strerror(blade_status) << std::endl;
        }

        // enable the TX channel RF frontend
        blade_status = bladerf_enable_module(dev, BLADERF_TX, true);

        // the gain must be set after the module has been enabled
        //blade_status = bladerf_set_gain_mode(dev, tx, BLADERF_GAIN_MGC);
        blade_status = bladerf_set_gain(dev, tx, tx1_gain);
        blade_status = bladerf_get_gain(dev, tx, &tx1_gain);

        // print out the specifics
        std::cout << std::endl << "------------------------------------------------------------------" << std::endl;
        std::cout << "sample_rate: " << sample_rate << std::endl;
        std::cout << "tx_freq:     " << tx_freq << std::endl;
        std::cout << "tx_bw:       " << tx_bw << std::endl;
        std::cout << "tx1_gain:    " << tx1_gain << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl << std::endl;

        idx = 0;

        is_running = true;

        if (signal(SIGINT, sig_handler) == SIG_ERR)
        {
            std::cerr << "Unable to catch SIGINT signals" << std::endl;
        }

        while (is_running)
        {
            std::cout << "Sending signal #" << idx << std::endl;
            blade_status = bladerf_sync_tx(dev, (int16_t*)iq_data.data(), num_samples, NULL, timeout_ms);

            if (blade_status != 0)
            {
                std::cout << "Unable to get the required number of samples: " << std::string(bladerf_strerror(blade_status)) << std::endl;
                return blade_status;
            }
          
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
