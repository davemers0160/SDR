// ----------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
#define _USE_MATH_DEFINES
#define NOMINMAX

#elif defined(__linux__)

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

#include "opencv_complex_functions.h"

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
//    std::vector<T> res(v1.size() - 1);
    std::vector<T> res(v1.size());

    auto v1_itr = v1.begin();
    auto v1_end = v1.end();
    auto res_itr = res.begin();

    for (idx=0; idx< v1.size(); ++idx)
    {
        //res[idx-1] = std::sqrt(v1[idx]*v1[idx] + v1[idx-1]*v1[idx-1] - scale * v1[idx] * v1[idx - 1]);

        res[idx - 1] = abs(v1[idx]);
    }

    return res;
}   // end of am_demod



//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{
    int bp = 0;
    int64_t idx, blk_idx;
    
    uint64_t num_samples;
    uint64_t block_size = 624000;



    // number of samples per second
    uint64_t sample_rate = 624000;


    // number of taps to create a low pass filters
    uint64_t rf_taps = 201;
    uint64_t fm_taps = 101;
    uint64_t audio_taps = 201;



    // offset from the center where we want to demodulate(Hz)
    int64_t rf_freq_offset = 0; // 115750;

    // rf frequency filter cutoff
    int64_t fc_rf = 40000;

    // decimation factor
    int64_t rf_decimation_factor = 3;

    // the FM broadcast signal has a bandwidth(Hz)
    int64_t desired_rf_sample_rate = (int64_t)(sample_rate / (double)rf_decimation_factor);

    // FM cutoff frequency
    int64_t fc_fm = 20000;

    // audio sample rate ==> 5 times the data bit rate
    int64_t am_sample_rate = 4160;
    int64_t desired_audio_sample_rate = 5 * am_sample_rate;

    // audio filter cutoff frequency(Hz)
    int64_t fc_am = 2400;
    int64_t am_offset = 2400;

    uint32_t min_distance = 2000;

    double x_min, x_max, delta;

    std::complex<double> cf_scale(1.0 / 2048.0, 0.0);

    //std::vector<float> sync_pulse = { -128, -128, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, -128, -128, -128, -128, -128, -128, -128 };

    cv::Mat sync_pulse = (cv::Mat_<double>(1,39) << -128, -128, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, -128, -128, -128, -128, -128, -128, -128);

    try{

        // test code

        //std::unique_ptr<SDR_BASE> sdr = SDR_BASE::build();

        //sample_rate = sdr->get_rx_samplerate();
        // number of samples is equal to the number seconds to record times the samplerate
        num_samples = 15 * 3600 * sample_rate;

        std::vector<complex<int16_t>> samples;
        //std::string filename = "../../rx_record/recordings/137M800_0M624__640s_test4.bin";
        //std::string filename = "../../rx_record/recordings/137M000_1M000__600s_20221120_0955.bin";
        std::string filename = "D:/data/RF/20240224/blade_F137.912M_SR0.624M_20240224_222353.sc16";
        read_iq_data(filename, samples);

        num_samples = samples.size();
        //num_samples = sample_rate;

        //-----------------------------------------------------------------------------
        // setup all of the filters and rotations
        //-----------------------------------------------------------------------------

        // decimation factor
        int64_t rf_decimation_factor = (int64_t)(sample_rate / (double)desired_rf_sample_rate);

        // calculate the new sampling rate based on the original and the decimated sample rate
        double decimated_sample_rate = sample_rate / (double)rf_decimation_factor;

        // RF low pass filter
        std::vector<double> lpf_rf = DSP::create_fir_filter<double>(rf_taps, (desired_rf_sample_rate / 2.0) / (double)sample_rate, &DSP::hann_window);
        cv::Mat cv_lpf_rf(1, lpf_rf.size(), CV_64FC1, lpf_rf.data());

        std::vector<double> lpf_fm = DSP::create_fir_filter<double>(fm_taps, fc_fm / decimated_sample_rate, &DSP::hann_window);
        cv::Mat cv_lpf_fm(1, lpf_fm.size(), CV_64FC1, lpf_fm.data());

        // find a decimation rate to achieve audio sampling rate
        int64_t audio_decimation_factor = (int64_t)(decimated_sample_rate / (double)am_sample_rate);
        double decimated_audio_sample_rate = decimated_sample_rate / (double)audio_decimation_factor;

        // scaling for FM demodulation
        double phasor_scale = 1 / ((2 * M_PI) / (decimated_sample_rate / desired_rf_sample_rate));

        // FM low pass de-emphasis filter
        //std::vector<double> lpf_fm = DSP::create_fir_filter<double>(fm_taps, 1.0/(double)(decimated_sample_rate * 75e-6), &DSP::rectangular_window);
        //cv::Mat cv_lpf_fm(1, lpf_fm.size(), CV_64FC1, lpf_fm.data());

        // Audio low pass filter coefficients
        std::vector<double> lpf_am = DSP::create_fir_filter<double>(audio_taps, (fc_am) / (double)decimated_sample_rate, &DSP::hann_window);
        cv::Mat cv_lpf_am(1, lpf_am.size(), CV_64FC1, lpf_am.data());


        // print out the specifics
        std::cout << std::endl << "------------------------------------------------------------------" << std::endl;
        std::cout << "fs:         " << sample_rate << std::endl;
        //std::cout << "rx_freq:    " << sdr->get_rx_frequency() << std::endl;
        std::cout << "f_offset:   " << rf_freq_offset << std::endl;
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
        std::vector<std::complex<double>> cf_samples(num_samples);
        for (idx = 0; idx < num_samples; ++idx)
        {
            cf_samples[idx] = cf_scale * std::complex<double>(std::real(samples[idx]), std::imag(samples[idx]));
            //cf_samples[idx] = cf_scale * std::complex<double>(std::real(samples[idx]), std::imag(samples[idx])) * (complex<double>)std::exp(-2.0 * 1i * M_PI * (f_offset / (double)sample_rate) * (double)idx);

        }
        cv::Mat cv_x1(1, num_samples, CV_64FC2, cf_samples.data());

        samples.clear();


        // apply low pass filter to the signal 
        cv::Mat cv_x3;
        cv::filter2D(cv_x1, cv_x3, CV_64FC2, cv_lpf_rf, cv::Point(-1, -1), cv::BORDER_REFLECT_101);



        // decimate the signal
        //x4 = x3(dec_seq);
        //std::vector<complex<float>> x4 = decimate_vec(cf_samples, rf_decimation_factor);
        //std::vector<complex<float>> x4 = decimate_vec(x3, rf_decimation_factor);
        cv::Mat cv_x4;
        cv_cmplx_decimate(cv_x3, cv_x4, (double)rf_decimation_factor);


        cv::Mat cv_x5;
        cv::filter2D(cv_x4, cv_x5, CV_64FC2, cv_lpf_fm, cv::Point(-1, -1), cv::BORDER_REFLECT_101);


        // polar discriminator - x4(2:end).*conj(x4(1:end - 1));
        //x5 = x4(af::seq(1, af::end, 1)) * af::conjg(x4(af::seq(0, -2, 1)));
        //x5 = af::atan2(af::imag(x5), af::real(x5)) * phasor_scale;
        //std::vector<float> x6 = polar_discriminator(x4, phasor_scale);
        //std::vector<float> x5 = polar_discriminator(x4, phasor_scale);
        cv::Mat cv_x6;
        cv_polar_discriminator(cv_x5, cv_x6, phasor_scale);

        // run the audio through the low pass de-emphasis filter
        //x6 = af::fir(af_lpf_de, x5);
        //std::vector<float> x6 = filter_vec(x5, lpf_audio);

        // run the audio through a second low pass filter before decimation
        //x6 = af::fir(af_lpf_a, x6);

        cv::Mat cv_x7;
        cv_x7 = cv_frequency_rotate(cv_x6, (double)am_offset / (double)decimated_sample_rate);



        cv::Mat cv_x8;
        cv::filter2D(cv_x7, cv_x8, CV_64FC2, cv_lpf_am, cv::Point(-1, -1), cv::BORDER_REFLECT_101);


        // decimate the audio sequence
        //x7 = x6(seq_audio);
        //std::vector<float> x7 = decimate_vec(x6, audio_decimation_factor);
        //cv_decimate(cv_x6, cv_x7, audio_decimation_factor);


        // scale the audio from -1 to 1
        //x7 = (x7 * (1.0 / (af::max<float>(af::abs(x7)))));

        // shift to 0 to 2 and then scale by 60
        //x7 = ((x7+1) * 40).as(af::dtype::u8);
        //std::vector<float> x8 = scale_vec(x7, 5.0f);

        //std::vector<float> x9 = am_demod(x8, 2.0f * std::cosf(2.0 * pi * fc_audio / (float)decimated_audio_sample_rate));

        //cv::Mat cv_x9 = 5.0 * cv_x7;

        cv::Mat cv_x9 = cv_cmplx_abs(cv_x8);

        // downsample the AM
        cv::Mat cv_x10;
        cv_decimate(cv_x9, cv_x10, audio_decimation_factor);

        cv::minMaxIdx(cv_x10, &x_min, &x_max);


        //auto result = std::minmax_element(x9.begin(), x9.end());
        //auto x_max = *std::max_element(x9.begin(), x9.end());
        //float x_min = *result.first;
        //float x_max = *result.second;

        delta = x_max - x_min;

        // Normalize the signal to px luminance values, discretize
        //std::vector<float> x11(x9.size());
        cv::Mat cv_x11;

        cv_x11 = (255.0 / delta) * (cv_x10 - x_min);

        cv_x11 = clamp(cv_x11, 0, 255);

        cv::Mat cv_x12;

        cv_x11.convertTo(cv_x12, CV_8UC1);

        cv_x11 -= 128.0;

        //float tmp;
        //
        //for(idx = 0; idx < x9.size(); ++idx)
        //{
        //    tmp = floor((127 * (x9[idx] - x_min) / delta) + 0.5);
        //    x11[idx] = tmp < -128 ? -128 : (tmp > 127 ? 127 : tmp);
        //}

        std::vector<std::pair<int64_t, float>> peaks = { {0, 0 } };
        cv::Rect r(0, 0, sync_pulse.total(), 1);

        idx = 0;
        while (idx <= cv_x11.total() - sync_pulse.total())
        {

            r.x = idx;

            auto corr = cv_x11(r).dot(sync_pulse) / (double)sync_pulse.total();

            //c5d(idx) = dot(sync, d5s(idx:idx + numel(sync) - 1)) / numel(sync);
            //corr = c5d(idx);
            auto t2 = (*(peaks.end()-1)).first;

            // If previous peak is too far, we keep it but add this value as new
            if ((idx - peaks[peaks.size() - 1].first) > min_distance)
            {
                //peaks(end + 1, :) = [idx, corr];
                peaks.push_back(std::make_pair(idx, corr));
            }
            //% idx = idx + ceil(mindistance / 4);
            else if (corr > peaks[peaks.size() - 1].second)
            {
                //peaks(end, :) = [idx, corr];
                peaks[peaks.size() - 1] = std::make_pair(idx, corr);
            }
            //end
            //    idx = idx + 1;
            //end
            ++idx;
        }

        cv::Mat img = cv::Mat::zeros(peaks.size(), 2080, CV_8UC1);

        //for idx = 1:(size(peaks, 1) - 2)
        //    img = cat(1, img, d5(peaks(idx, 1) :peaks(idx, 1) + 2079)');
        //end
        //int64_t index;
        r.width = 2080;
        for (idx = 0; idx < peaks.size()-1; ++idx)
        {
            r.x = peaks[idx].first;
            cv_x12(r).copyTo(img(cv::Rect(0, idx, 2080, 1)));

        }

        bp = 1;
        //sdr->stop();
        cv::namedWindow("test", cv::WINDOW_NORMAL);
        cv::imshow("test", img);
        cv::waitKey(0);

    }
    catch(std::exception e)
    {
        std::cout << "error: " << e.what() << std::endl;
        std::cin.ignore();
    }

    
    return 0;
    
}   // end of main
