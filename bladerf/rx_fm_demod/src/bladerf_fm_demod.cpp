// ----------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
#define _USE_MATH_DEFINES
#define NOMINMAX

//#include <mmsystem.h>
//#pragma comment(lib, "winmm.lib")

#elif defined(__linux__)

#endif

// ArrayFire Includes
#ifdef USE_ARRAYFIRE
#include <arrayfire.h>
#endif

#include <cstdint>
#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>

#include <complex>

// bladeRF includes
//#include <libbladeRF.h>
//#include <bladeRF2.h>

#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")

// Custom Includes
#include "num2string.h"
#include "get_current_time.h"
#include "file_parser.h"
#include "file_ops.h"
#include "sleep_ms.h"
#include "dsp/dsp_windows.h"

#define BUILD_BLADERF

// Project Includes
#include <bladerf_common.h>
#include "bladerf_sdr.h"


const std::complex<double> j = std::complex<double>(0, 1);
const double pi = 3.14159265358979;

//-----------------------------------------------------------------------------
template<class T, class U>
inline std::complex<T> complex_cast(const std::complex<U>& c) {
//    return { reinterpret_cast<T>(c.real()), reinterpret_cast<T>(c.imag()) };
    //return { (T)(c.real()), (T)(c.imag()) };
    return std::complex<T>(std::real(c), std::imag(c));
}


std::unique_ptr<SDR_BASE> SDR_BASE::build()
{
#ifdef BUILD_BLADERF 
    std::unique_ptr<BLADERF_SDR> bladerf_dev = BLADERF_SDR::open();

    // initialize the 
    bladerf_dev->init_rx();

    bladerf_dev->set_rx_frequency(96700000);
    bladerf_dev->set_rx_samplerate(1000000);
    bladerf_dev->set_rx_gain(30, BLADERF_GAIN_MANUAL);
    bladerf_dev->set_rx_bandwidth(1000000);

    //bladerf_dev->setSamplePublisher(std::move(config.Bladerf.samplePublisher));
    return std::unique_ptr<SDR_BASE>(bladerf_dev.release());
#endif

}


//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{

    uint32_t idx;
    
    // bladeRF variable
    struct bladerf_devinfo *device_list = NULL;
    //struct bladerf_devinfo dev_info;
    //struct bladerf* dev;
    int bladerf_num;
    //int blade_status;
    bladerf_channel rx = BLADERF_CHANNEL_RX(0);
    bladerf_channel tx = BLADERF_CHANNEL_TX(0);
    bladerf_frequency rx_freq;
    bladerf_sample_rate fs;
    bladerf_bandwidth rx_bw;
    bladerf_gain rx1_gain = 32;
    int64_t span = 100000;

    int64_t f_offset;           // offset from the tuned frequency (Hz)
    int64_t channel_bw;         // bandwidth of the signal (Hz)
    uint64_t n_taps;            // number of taps used in the first FIR filter
    int64_t audio_freq;         // optimal audio frequency sample rate

    uint32_t num_samples = 65536*2;
    uint32_t timeout_ms = 10000;
    const uint32_t num_buffers = 16;
    const uint32_t buffer_size = 1024 * 64;        // must be a multiple of 1024
    const uint32_t num_transfers = 8;
    const float scale = (1.0 / 2048.0);
    std::vector<std::complex<int16_t>> samples(num_samples);

    //std::string filename = "../../rx_record/recordings/096M600_1M__10s_test.bin"; 
    //std::ifstream input_data(filename, std::ios::binary);

    uint8_t test_case = 1;

    switch (test_case)
    {
    // NOAA radio
    case 0:
        fs = 1000000;
        rx_freq = 162400000;
        rx_bw = 500000;
        f_offset = 50000;
        channel_bw = 40000;
        audio_freq = 20000;
        n_taps = 80;
        break;

    // FM radio station
    case 1:
        fs = 1000000;
        rx_freq = 96600000;
        rx_bw = 1000000;
        f_offset = 100000;
        channel_bw = 200000;
        audio_freq = 44100;
        n_taps = 100;
        break;
        // FM radio station
    case 2:
        fs = 1400000;
        rx_freq = 137500000;
        rx_bw = 1400000;
        f_offset = 120000;
        channel_bw = 48000;
        audio_freq = 4800;
        n_taps = 100;
        break;

    }

#ifdef USE_ARRAYFIRE
    
    af::setBackend(AF_BACKEND_CPU);
    af::info();

    std::cout << std::endl << std::endl;

    // array fire variables
    //af::array raw_data, fft_data, raw_data2;

    af::array x2, x3, x4, x5, x6, x7, x8;

#endif // USE_ARRAYFIRE

    //input_data.seekg(0, std::ios::end);
    //size_t filesize = input_data.tellg();
    //input_data.seekg(0, std::ios::beg);

    //std::vector<int16_t> buffer(filesize / sizeof(int16_t));

    //input_data.read((char*)buffer.data(), filesize);

    //num_samples = buffer.size() >> 1;


    //int num_devices = bladerf_get_device_list(&device_list);

    //bladerf_num = select_bladerf(num_devices, device_list);

    //if (bladerf_num < 0)
    //{
    //    std::cout << "could not detect any bladeRF devices..." << std::endl;
    //    //return 0;
    //}

    std::cout << std::endl;

    try{

        std::unique_ptr<SDR_BASE> sdr = SDR_BASE::build();

        // decimation rate
        int64_t dec_rate = (int64_t)(fs / (float)channel_bw);

        // calculate the new sampling rate based on the original and the decimated sample rate
        float fs_d = fs / (float)dec_rate;

        // build the decimation sequence
        af::array dec_seq = af::seq(0, num_samples, dec_rate);

        // low pass filter coefficients
        std::vector<float> lpf = DSP::create_fir_filter<float>(n_taps, (channel_bw / 2.0) / (float)fs, &DSP::hann_window);
        // create the low pass filter from the filter coefficients
        af::array af_lpf = af::array(lpf.size(), (float*)lpf.data());

        // find a decimation rate to achieve audio sampling rate between for 10 kHz
        int64_t dec_audio = (int64_t)(fs_d / (float)audio_freq);
        float fs_audio = fs_d / (float)dec_audio;

        // scaling for tangent
        float phasor_scale = 1 / ((2 * M_PI) / (fs_d / channel_bw));

        // audio decimation sequence
        af::array seq_audio = af::seq(0, dec_seq.dims(0), dec_audio);

        std::vector<float> lpf_de = DSP::create_fir_filter<float>(64, 1.0f/(float)(fs_d * 75e-6), &DSP::rectangular_window);
        af::array af_lpf_de = af::array(lpf_de.size(), (float*)lpf_de.data());

        std::vector<float> lpf_a = DSP::create_fir_filter<float>(n_taps, (audio_freq / 2.0) / (float)fs_d, &DSP::hann_window);
        af::array af_lpf_a = af::array(lpf_a.size(), (float*)lpf_a.data());

        // A*exp(j*3*pi*t) = A*cos(3*pi*t) + j*sin(3*pi*t)
        // generate the frequency rotation vector to center the offset frequency 
        std::vector<std::complex<float>> fc_rot(num_samples, 0);
        for (idx = 0; idx < num_samples; ++idx)
        {
            fc_rot[idx] = std::exp(-2.0 * 1i * M_PI * (f_offset / (double)fs) * (double)idx);
        }

        double freq_step = (fs)/(double)num_samples;

        double f_min = (rx_freq - (span>>1)) * 1.0e-6;
        double f_max = (rx_freq + (span>>1)) * 1.0e-6;

        uint32_t sp = (uint32_t)((fs - span) / (2.0 * freq_step));
        uint32_t sp2 = (uint32_t)(span / freq_step);

        double fft_scale = 1.0 / (double)(num_samples);

        std::complex<float> cf_scale(scale, 0.0f);

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)

        // audio setup for windows only
        HWAVEOUT hWaveOut = 0;
        WAVEFORMATEX wfx = { WAVE_FORMAT_PCM, 1, (DWORD)fs_audio, (DWORD)fs_audio, 1, 8, 0 };
        waveOutOpen(&hWaveOut, WAVE_MAPPER, &wfx, 0, 0, CALLBACK_NULL);

#endif

        // print out the specifics
        std::cout << std::endl << "------------------------------------------------------------------" << std::endl;
        std::cout << "fs:         " << fs << std::endl;
        std::cout << "rx_freq:    " << rx_freq << std::endl;
        std::cout << "f_offset:   " << f_offset << std::endl;
        std::cout << "channel_bw: " << channel_bw << std::endl;
        std::cout << "dec_rate:   " << dec_rate << std::endl;
        std::cout << "fs_d:       " << fs_d << std::endl;
        std::cout << "audio_freq: " << audio_freq << std::endl;
        std::cout << "dec_audio:  " << dec_audio << std::endl;
        std::cout << "fs_audio:   " << fs_audio << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl << std::endl;

        std::vector<std::complex<float>> cf_samples(num_samples);

        sdr->start(cf_samples);

        while(1)
        {
            //sdr->start_single(cf_samples, num_samples);
            sdr->wait_for_samples();

#ifdef USE_ARRAYFIRE
            // take the complex float vector data and put it into an af::array container 
            x2 = af::array(num_samples, (af::cfloat*)cf_samples.data());

            // apply low pass filter to the rotated signal 
            x3 = af::fir(af_lpf, x2);

            // decimate the signal
            x4 = x3(dec_seq);

            // polar discriminator - x4(2:end).*conj(x4(1:end - 1));
            x5 = x4(af::seq(1, af::end, 1)) * af::conjg(x4(af::seq(0, -2, 1)));
            x5 = af::atan2(af::imag(x5), af::real(x5)) * phasor_scale;// .as(f32);

            // run the audio through the low pass de-emphasis filter
            x6 = af::fir(af_lpf_de, x5);

            // run the audio through a second low pass filter before decimation
            x6 = af::fir(af_lpf_a, x6);

            // decimate the audio sequence
            x7 = x6(seq_audio);

            // scale the audio from -1 to 1
            x7 = (x7 * (1.0 / (af::max<float>(af::abs(x7)))));

            // shift to 0 to 2 and then scale by 60
            x7 = ((x7+1) * 30).as(af::dtype::u8);

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
            // place the data into the header for windows audio data
            WAVEHDR header = { (LPSTR)x7.host<uint8_t>(), x7.dims(0) * sizeof(uint8_t), 0, 0, 0, 0, 0, 0 };

            waveOutPrepareHeader(hWaveOut, &header, sizeof(WAVEHDR));
            waveOutWrite(hWaveOut, &header, sizeof(WAVEHDR));
#endif

            // show the results of the FFT in the window
            //fft_data = 20 * af::log10(af::shift(af::abs(af::fft(x3)*fft_scale), (num_samples >> 1)))-10;
            //myWindow.plot(f, fft_data(X));

            //myWindow.scatter(af::real(x4), af::imag(x4));
            //myWindow.show();

            //sleep_ms(10000);

#endif
        }

        sdr->stop();

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
        waveOutClose(hWaveOut);
#endif

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
