// ----------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
#define _USE_MATH_DEFINES
#define NOMINMAX

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

// #include <mmsystem.h>
// #pragma comment(lib, "winmm.lib")

// Custom Includes
#include "num2string.h"
#include "get_current_time.h"
#include "file_parser.h"
#include "file_ops.h"
#include "sleep_ms.h"
#include "dsp/dsp_windows.h"
#include "iq_utils.h"

// OpenCV Includes
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp> 


#define BUILD_BLADERF

// Project Includes
//#include <bladerf_common.h>
#include "bladerf_sdr.h"


const std::complex<double> j = std::complex<double>(0, 1);
const double pi = 3.14159265358979;

//-----------------------------------------------------------------------------
template<class T, class U>
inline std::complex<T> complex_cast(const std::complex<U>& c) {

    return std::complex<T>(std::real(c), std::imag(c));
}


std::unique_ptr<SDR_BASE> SDR_BASE::build()
{
#ifdef BUILD_BLADERF 
    std::unique_ptr<BLADERF_SDR> bladerf_dev = BLADERF_SDR::open();

    // Use sample rate if set, otherwise default to 2.4MSPS.

    bladerf_dev->init_rx();

    //if (config.Bladerf.sampleRate != 0) {
    //    bladerf_dev->setSampleRate(config.Bladerf.sampleRate);
    //}
    //else {
    //    bladerf_dev->setSampleRate(2400000);
    //}

    bladerf_dev->set_rx_frequency(96700000);
    bladerf_dev->set_rx_samplerate(624000);
    bladerf_dev->set_rx_gain(30, BLADERF_GAIN_MANUAL);
    bladerf_dev->set_rx_bandwidth(1000000);

    //bladerf_dev->setSamplePublisher(std::move(config.Bladerf.samplePublisher));
    return std::unique_ptr<SDR_BASE>(bladerf_dev.release());
#endif

};

//-----------------------------------------------------------------------------
template <typename T>
std::vector<T> vec_ptws_mul(std::vector<T>& v1, std::vector<T>& v2)
{
    if (v1.size() != v2.size())
        std::cout << "vectors need to be the same size:" << std::endl;

    std::vector<T> res(v1.size());

    auto v1_itr = v1.begin();
    auto v1_end = v1.end();
    auto v2_itr = v2.begin();
    auto res_itr = res.begin();

    for (; v1_itr != v1_end; ++v1_itr, ++v2_itr, ++res_itr)
    {
        *res_itr = (*v1_itr) * (*v2_itr);
    }

    return res;
}   // end of vec_ptws_mul

//-----------------------------------------------------------------------------
template <typename T>
std::vector<T> decimate_vec(std::vector<T>& v1, double rate)
{
    uint64_t num = (uint64_t)std::ceil(v1.size() / rate);

    std::vector<T> res(num);

    auto v1_itr = v1.begin();
    auto res_itr = res.begin();
    auto res_end = res.end();

    double index = 0.0;

    for (uint64_t idx = 0; idx < num; ++idx)
    {
        res[idx] = v1[floor(index)];
        index += rate;
    }

    //for (; res_itr != res_end; ++res_itr)
    //{
    //    std::advance(v1_itr, rate);
    //    *res_itr = *v1_itr;
    //    //index += rate;
    //}

    return res;
}   // 

//-----------------------------------------------------------------------------
//polar discriminator - x(2:end).*conj(x(1:end - 1));
template <typename T>
std::vector<T> polar_discriminator(std::vector<std::complex<T>>& v1, float scale)
{
    std::complex<T> tmp;
    std::vector<T> res(v1.size()-1);

    auto v1_itr0 = v1.begin();
    auto v1_itr1 = (v1.begin() + 1);
    auto v1_end = v1.end();
    auto res_itr = res.begin();
    auto res_end = res.end();

    for (; v1_itr1 != v1_end; ++v1_itr0, ++v1_itr1, ++res_itr)
    {
        tmp = (*v1_itr0) * std::conj(*v1_itr1);

        *res_itr = scale * std::atan2f(tmp.imag(), tmp.real());
    }

    return res;
}

//-----------------------------------------------------------------------------
template <typename T, typename U>
std::vector<T> filter_vec(std::vector<T>& v1, std::vector<U>& h)
{
    uint64_t idx, jdx, kdx;
    uint64_t start;
    
    uint64_t v1_size = v1.size();
    uint64_t h_size = h.size();
    uint64_t h2_size = h.size() >> 1;

    std::vector<T> res(v1_size, 0);

    for (idx = 0; idx < v1_size; ++idx)
    {
        // v1_size = 20; f_size = 7
        uint64_t jmn = (idx >= h2_size) ? 0 : (h2_size - idx);  // idx = 0,3,18 => jmn=0,0,0
        uint64_t jmx = (idx < v1_size - h2_size) ? h_size - 1 : v1_size - idx;           // idx = 0,3,18 => jmx=0,3,18

        res[idx] = T(0);
        kdx = (uint64_t)max(int64_t(0), (int64_t)idx - (int64_t)h2_size);
        for (jdx = jmn; jdx <= jmx; ++jdx, ++kdx) 
        {
            res[idx] += (v1[kdx] * h[h_size-jdx-1]);
        }
    }
    
    return res;
}   // end of filter_vec

//-----------------------------------------------------------------------------
template <typename T>
std::vector<T> scale_vec(std::vector<T>& v1, T scale)
{
    std::vector<T> res(v1.size());

    auto v1_itr = v1.begin();
    auto v1_end = v1.end();
    auto res_itr = res.begin();

    for (; v1_itr != v1_end; ++v1_itr, ++res_itr)
    {
        *res_itr = scale * (*v1_itr);
    }

    return res;
}

//-----------------------------------------------------------------------------
template <typename T>
std::vector<T> am_demod(std::vector<T>& v1, T scale)
{
    uint64_t idx;
    std::vector<T> res(v1.size()-1);

    auto v1_itr = v1.begin();
    auto v1_end = v1.end();
    auto res_itr = res.begin();

    for (idx=1; idx< v1.size(); ++idx)
    {
        res[idx-1] = std::sqrt(v1[idx]*v1[idx] + v1[idx-1]*v1[idx-1] - scale * v1[idx] * v1[idx - 1]);
    }

    return res;
}   // end of am_demod



//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{

    uint64_t idx, blk_idx;
    
    uint64_t num_samples;

    // number of samples per second
    uint64_t sample_rate = 624000;
    uint64_t block_size = 624000;


    // number of taps to create a low pass filters
    uint64_t rf_taps = 128;
    uint64_t fm_taps = 64;
    uint64_t audio_taps = 200;

    // offset from the center where we want to demodulate(Hz)
    int64_t f_offset = 115750;

    // rf frequency filter cutoff
    int64_t fc_rf = 62400;

    // the FM broadcast signal has a bandwidth(Hz)
    int64_t desired_rf_sample_rate = fc_rf * 2;

    // audio sample rate ==> 5 times the data bit rate
    int64_t bit_rate = 4160;
    int64_t desired_audio_sample_rate = 5 * bit_rate;

    // audio filter cutoff frequency(Hz)
    int64_t fc_audio = 2400;

    std::complex<float> cf_scale(1.0 / 2048.0f, 0.0f);

    std::vector<float> sync_pulse = { -128, -128, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, -128, -128, -128, -128, -128, -128, -128 };

    try{

        // test code
        //std::vector<float> v1 = { 1,2,1,2,1,2,1,2 };
        //std::vector<float> h = { 1, -2, 1 };
        //std::vector<float> x1 = filter_vec(v1, h);




        //std::unique_ptr<SDR_BASE> sdr = SDR_BASE::build();

        //sample_rate = sdr->get_rx_samplerate();
        // number of samples is equal to the number seconds to record times the samplerate
        num_samples = 15 * 3600 * sample_rate;

        std::vector<complex<int16_t>> samples;
        std::string filename = "../../rx_record/recordings/137M800_0M624__640s_test4.bin";
        read_iq_data(filename, samples);

        num_samples = samples.size();

        //-----------------------------------------------------------------------------
        // setup all of the filters and rotations
        //-----------------------------------------------------------------------------

        // decimation factor
        int64_t rf_decimation_factor = (int64_t)(sample_rate / (float)desired_rf_sample_rate);

        // calculate the new sampling rate based on the original and the decimated sample rate
        float decimated_sample_rate = sample_rate / (float)rf_decimation_factor;

        // RF low pass filter coefficients
        std::vector<float> lpf_rf = DSP::create_fir_filter<float>(rf_taps, (desired_rf_sample_rate / 2.0) / (float)sample_rate, &DSP::hann_window);
        af::array af_lpf_rf = af::array(lpf_rf.size(), (float*)lpf_rf.data());

        // find a decimation rate to achieve audio sampling rate
        int64_t audio_decimation_factor = (int64_t)(decimated_sample_rate / (float)desired_audio_sample_rate);
        float decimated_audio_sample_rate = decimated_sample_rate / (float)audio_decimation_factor;

        // scaling for FM demodulation
        float phasor_scale = 1 / ((2 * M_PI) / (decimated_sample_rate / desired_rf_sample_rate));

        std::vector<float> lpf_fm = DSP::create_fir_filter<float>(fm_taps, 1.0/(float)(decimated_sample_rate * 75e-6), &DSP::rectangular_window);
        af::array af_lpf_fm = af::array(lpf_fm.size(), (float*)lpf_fm.data());

        // Audio low pass filter coefficients
        std::vector<float> lpf_audio = DSP::create_fir_filter<float>(audio_taps, (fc_audio / 2.0) / (float)decimated_sample_rate, &DSP::hann_window);
        af::array af_lpf_audio = af::array(lpf_audio.size(), (float*)lpf_audio.data());

        // A*exp(j*3*pi*t) = A*cos(3*pi*t) + j*sin(3*pi*t)
        // generate the frequency rotation vector to center the offset frequency 
        //std::vector<std::complex<float>> fc_rot(num_samples, 0);
        //for (idx = 0; idx < num_samples; ++idx)
        //{
        //    fc_rot[idx] = std::exp(-2.0 * 1i * M_PI * (f_offset / (double)sample_rate) * (double)idx);
        //}

        // print out the specifics
        std::cout << std::endl << "------------------------------------------------------------------" << std::endl;
        std::cout << "fs:         " << sample_rate << std::endl;
        //std::cout << "rx_freq:    " << sdr->get_rx_frequency() << std::endl;
        std::cout << "f_offset:   " << f_offset << std::endl;
        std::cout << "channel_bw: " << desired_rf_sample_rate << std::endl;
        std::cout << "dec_rate:   " << rf_decimation_factor << std::endl;
        std::cout << "fs_d:       " << decimated_sample_rate << std::endl;
        std::cout << "audio_freq: " << desired_audio_sample_rate << std::endl;
        std::cout << "dec_audio:  " << audio_decimation_factor << std::endl;
        std::cout << "fs_audio:   " << decimated_audio_sample_rate << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl << std::endl;
              
        //sdr->start_single(cf_samples, num_samples);
        //sdr->wait_for_samples();

        //-----------------------------------------------------------------------------
        // start the demodulation process
        //-----------------------------------------------------------------------------


        // take the complex float vector data and rotate it
        //cv::Mat cv_fc_rot(1, fc_rot.size(), CV_64FC2, fc_rot.data());
        //cv::Mat cv_samples(1, cf_samples.size(), CV_64FC2, cf_samples.data());
        //cv::Mat x2 = mul_cmplx(cv_samples, cv_fc_rot);
        //std::vector<complex<float>> x2 = vec_ptws_mul(cf_samples, fc_rot);
        std::vector<std::complex<float>> cf_samples(num_samples);
        for (idx = 0; idx < num_samples; ++idx)
        {
            cf_samples[idx] = cf_scale * std::complex<float>(std::real(samples[idx]), std::imag(samples[idx])) * (complex<float>)std::exp(-2.0 * 1i * M_PI * (f_offset / (double)sample_rate) * (double)idx);
        }

        //for (idx = 0; idx < num_samples; ++idx)
        //{
        //    cf_samples[idx] = cf_samples[idx] * (complex<float>)std::exp(-2.0 * 1i * M_PI * (f_offset / (double)sample_rate) * (double)idx);
        //}

        samples.clear();


        // apply low pass filter to the rotated signal 
        //x3 = af::fir(af_lpf, x2);
        //std::vector<complex<float>> x3 = filter_vec(cf_samples, lpf_rf);
        cv::Mat cv_cf(1, num_samples, CV_32FC2, cf_samples.data());
        cv::Mat cv_lpf_rf(1, lpf_rf.size(), CV_32FC1, lpf_rf.data());
        cv::Mat cv_x3;
        cv::filter2D(cv_cf, cv_x3, CV_64FC2, cv_lpf_rf, cv::Point(-1, -1), cv::BORDER_REFLECT_101);
        std::vector<complex<float>> x3;
        x3.assign(cv_x3.data, cv_x3.data + cv_x3.total() * cv_x3.channels());


        // decimate the signal
        //x4 = x3(dec_seq);
        //std::vector<complex<float>> x4 = decimate_vec(cf_samples, rf_decimation_factor);
        std::vector<complex<float>> x4 = decimate_vec(x3, rf_decimation_factor);

        // polar discriminator - x4(2:end).*conj(x4(1:end - 1));
        //x5 = x4(af::seq(1, af::end, 1)) * af::conjg(x4(af::seq(0, -2, 1)));
        //x5 = af::atan2(af::imag(x5), af::real(x5)) * phasor_scale;
        std::vector<float> x6 = polar_discriminator(x4, phasor_scale);
        //std::vector<float> x5 = polar_discriminator(x4, phasor_scale);

        // run the audio through the low pass de-emphasis filter
        //x6 = af::fir(af_lpf_de, x5);
        //std::vector<float> x6 = filter_vec(x5, lpf_audio);

        // run the audio through a second low pass filter before decimation
        //x6 = af::fir(af_lpf_a, x6);

        // decimate the audio sequence
        //x7 = x6(seq_audio);
        std::vector<float> x7 = decimate_vec(x6, audio_decimation_factor);

        // scale the audio from -1 to 1
        //x7 = (x7 * (1.0 / (af::max<float>(af::abs(x7)))));

        // shift to 0 to 2 and then scale by 60
        //x7 = ((x7+1) * 40).as(af::dtype::u8);
        std::vector<float> x8 = scale_vec(x7, 5.0f);

        std::vector<float> x9 = am_demod(x8, 2.0f * std::cosf(2.0 * pi * fc_audio / (float)decimated_audio_sample_rate));

        auto x_min = *std::min_element(x9.begin(), x9.end());
        auto x_max = *std::max_element(x9.begin(), x9.end());


        //sdr->stop();


    }
    catch(std::exception e)
    {
        std::cout << "error: " << e.what() << std::endl;
    }

    
    return 0;
    
}   // end of main
