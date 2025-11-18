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
#include <mutex>
#include <thread>
#include <condition_variable>
#include <complex>
#include <stdexcept>

// bladeRF includes
//#include <libbladeRF.h>
//#include <bladeRF2.h>

// #include <mmsystem.h>
// #pragma comment(lib, "winmm.lib")

// Custom Includes
#include "num2string.h"
#include "get_current_time.h"
//#include "file_parser.h"
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
#include <bladerf_common.h>
#include "bladerf_sdr.h"

//-----------------------------------------------------------------------------
const std::complex<double> j = std::complex<double>(0, 1);
const double pi = 3.14159265358979;

std::vector<std::vector<std::complex<int16_t>>> iq_data_queue(2);
std::mutex read_mutex;
std::condition_variable data_ready_cv;
volatile std::vector<bool> data_ready(2, false);
volatile int32_t read_index = 0;
volatile int32_t write_index = 0;
volatile bool run = false;



//-----------------------------------------------------------------------------
// thread function to simulate data capture
inline void temp_get_data(bladerf_sample_rate sample_rate)
{
    int32_t blade_status = 0;

    double capture_time = 1.0;
    uint64_t block_size = floor(sample_rate * capture_time + 0.5);

    iq_data_queue[0].resize(block_size);
    iq_data_queue[1].resize(block_size);

    // data to read in
    std::string filename = "D:/data/RF/20240224/blade_F137.912M_SR0.624M_20240224_222353.sc16";
    std::vector<std::complex<int16_t>> samples;
    read_iq_data(filename, samples);
    uint64_t num_samples = samples.size();

    // delete some extra samples to match the capture buffers and size
    uint64_t rem = num_samples % (2 * block_size);
    if (rem > 0)
    {
        samples.erase(samples.end() - rem, samples.end());
    }

    write_index = 0;
    uint64_t sample_index = 0;
    while (run == true)
    {
        // take the samples and place them into the right buffer
        std::copy(samples.begin() + sample_index, // Start iterator for the source range
            samples.begin() + sample_index + block_size, // End iterator (exclusive) for the source range
            std::back_inserter(iq_data_queue[write_index]));

        // sleep to simulate the data capture of the SDR
        std::this_thread::sleep_for(std::chrono::milliseconds((uint32_t)(capture_time*999)));

        write_index = (write_index+1) & 0x01;
        sample_index += block_size;
        if (sample_index >= samples.size())
        {
            sample_index = 0;
        }
    }

}


//-----------------------------------------------------------------------------
void get_data(uint32_t sample_rate, struct bladerf* dev)
{
    int blade_status;
    uint32_t timeout_ms = 10000;

    double capture_time = 2.0;
    uint64_t block_size = floor(sample_rate * capture_time + 0.5);

    iq_data_queue[0].resize(block_size);
    iq_data_queue[1].resize(block_size);

    while (run)
    {
        
        blade_status = bladerf_sync_rx(dev, (void*)iq_data_queue[write_index].data(), block_size, NULL, timeout_ms);
        if (blade_status != 0)
        {
            std::cout << "Unable to get the required number of samples: " << std::string(bladerf_strerror(blade_status)) << std::endl;
        }
        //data_ready[write_index] = true;

        write_index = (write_index + 1) & 0x01;

    }

}

std::vector<std::complex<double>> polyphase_decimate(const std::vector<std::complex<double>>& x, int32_t decimation_factor, const std::vector<double>& h)
{
    uint64_t idx, jdx, kdx;
    int64_t index;

    if (decimation_factor <= 0)
        throw std::invalid_argument("Decimation factor M must be positive.");

    int32_t filter_size = h.size();
    if (filter_size % decimation_factor != 0)
        throw std::invalid_argument("Filter length must be a multiple of M.");

    int32_t tap_per_phase = filter_size / decimation_factor;   // taps per phase

    // --- Build polyphase components ---
    // E[k][p] = h[p*M + k],  (k = phase, p = tap index)
    //N = floor(fm_taps / rf_decimation_factor);
    //p = zeros(rf_decimation_factor, N);

    //for k = 1:decimation_factor
    //    index = k : decimation_factor : fm_taps;
    //p(k, :) = lpf_fm(index);
    //end
    std::vector<std::vector<double>> E(decimation_factor, std::vector<double>(tap_per_phase));
    for (idx = 0; idx < decimation_factor; ++idx)
    {
        for (jdx = 0; jdx < tap_per_phase; ++jdx)
        {
            E[idx][jdx] = h[jdx * decimation_factor + idx];
        }
    }

    // --- Output size ---
    int64_t output_length = x.size() / decimation_factor;
    std::vector<std::complex<double>> output(output_length, std::complex<double>(0,0));

    // --- Perform polyphase filtering + decimation ---
    for (idx= 0; idx < output_length; ++idx)
    {
        std::complex<double> sum(0.0, 0.0);
        std::complex<double> sum2(0.0, 0.0);

        // Process each polyphase branch
        for (jdx = 0; jdx < decimation_factor; ++jdx)
        {
            // For output sample n, we process input starting at index n*M
            // Each branch filters different phases of the input
            for (kdx = 0; kdx < tap_per_phase; ++kdx)
            {
                // index = rf_decimation_factor * (idx - kdx) - jdx;  % 0-based index
                index = decimation_factor * (idx - kdx) - jdx;    //idx * M - branch - k * M;

                // Handle boundary conditions (zero-pad before signal start)
                if (index >= 0 && index < x.size())
                {
                    sum += E[jdx][kdx] * x[index];
                }
            }
        }

        output[idx] = sum;
    }

    return output;
}

/*

template<typename T, typename U>
std::vector<std::complex<T>> polyphaseDecimate(const std::vector<std::complex<T>>& input, int32_t M, const std::vector<U>& filter)
{

    const size_t N = input.size();
    const size_t L = filter.size();               // total FIR length

    if (L % M != 0)
        throw std::invalid_argument("Filter length must be a multiple of decimation factor M");

    const size_t P = L / M;                       // taps per polyphase branch

    // --------------------------------------------------------------------
    // Build the polyphase sub-filters:  e_k[n] = h[n*M + k]   (k = 0..M-1)
    // --------------------------------------------------------------------
    std::vector<std::vector<T>> branches(M, std::vector<T>(P));
    for (int k = 0; k < M; ++k) 
    {
        for (size_t p = 0; p < P; ++p) 
        {
            size_t h_idx = p * M + k;              // causal index in original filter
            branches[k][p] = filter[h_idx];
        }
    }

    // --------------------------------------------------------------------
    // Output length: floor( (N + L - 1) / M )  (exact formula for FIR decimation)
    // --------------------------------------------------------------------
    const size_t outLen = (N + L - 1) / M;
    std::vector<std::complex<double>> output(outLen, std::complex<double>(0, 0));

    // --------------------------------------------------------------------
    // Process the input in blocks of M samples.
    // For each block we run the M polyphase branches in parallel.
    // --------------------------------------------------------------------
    for (size_t n = 0; n < N; ++n) 
    {
        int k = n % M;                                 // which branch receives this sample
        const std::complex<double> x = input[n];

        // Add contribution of this sample to its branch's delay line
        // (we keep a per-branch circular buffer of length P)
        static thread_local std::vector<std::complex<double>> delay(M); // one per branch
        if (delay.size() != P) 
        {
            for (int b = 0; b < M; ++b)
            {
                delay[b].assign(P, std::complex<double>(0, 0));
            }
        }

        // Insert new sample at the *end* of the delay line (causal FIR)
        delay[k].push_back(x);
        if (delay[k].size() > P)
        {
            delay[k].erase(delay[k].begin());
        }

        // If we have just processed the *last* sample of a decimation block
        // (i.e. k == M-1) we can emit one output sample.
        if (k == M - 1) 
        {
            size_t outIdx = n / M;                     // decimated time index
            std::complex<double> y(0, 0);

            for (int b = 0; b < M; ++b) 
            {
                // Multiply the branch delay line with its coefficients
                std::complex<double> branchSum(0, 0);
                for (size_t p = 0; p < P; ++p) 
                {
                    // delay[b][P-1-p] is the oldest sample for tap p
                    branchSum += delay[b][P - 1 - p] * std::complex<double>(branches[b][p], 0);
                }
                y += branchSum;
            }
            output[outIdx] = y;
        }
    }

    // --------------------------------------------------------------------
    // Handle the tail: remaining samples after the last full block
    // --------------------------------------------------------------------
    size_t samplesAfterLastBlock = N % M;
    if (samplesAfterLastBlock != 0) 
    {
        // We still have partial input; run the remaining branches with zeros
        // to flush the filter.
        size_t outIdx = N / M;
        for (int missing = samplesAfterLastBlock; missing < M; ++missing) 
        {
            int k = missing;
            // push a zero into the missing branch
            delay[k].push_back(std::complex<double>(0, 0));
            if (delay[k].size() > P) delay[k].erase(delay[k].begin());
        }

        std::complex<double> y(0, 0);
        for (int b = 0; b < M; ++b) 
        {
            std::complex<double> branchSum(0, 0);
            for (size_t p = 0; p < P; ++p) {
                branchSum += delay[b][P - 1 - p] * std::complex<double>(branches[b][p], 0);
            }
            y += branchSum;
        }
        output[outIdx] = y;
    }

    // Trim any excess slots that were allocated because of the (N+L-1)/M formula
    if (output.size() > outIdx + 1)
        output.resize(outIdx + 1);

    return output;
}
*/

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
std::vector<T> polar_discriminator(std::vector<std::complex<T>>& v1, double scale)
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
        tmp = (*v1_itr1) * std::conj(*v1_itr0);

        *res_itr = scale * std::atan2(tmp.imag(), tmp.real());
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
inline std::vector<std::complex<double>> frequency_shift(const std::vector<T>& data, double fr)
{
    double index = 0;
    std::vector<complex<double>> res(data.size());

    std::complex<double> frc = { 0, 2.0 * M_PI * fr };

    auto d_itr = data.begin();
    auto d_end = data.end();
    auto res_itr = res.begin();

    for (; d_itr != d_end; ++d_itr, ++res_itr)
    {
        *res_itr = (*d_itr) * std::exp(frc*index);
        index += 1;
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

        res[idx] = abs(v1[idx]);
    }

    return res;
}   // end of am_demod


//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{
    int bp = 0;
    int64_t idx, blk_idx;
    
    uint64_t num_samples;
    //uint64_t block_size = 624000;

    // timing variables
    auto start_time = std::chrono::high_resolution_clock::now();
    auto stop_time = std::chrono::high_resolution_clock::now();
    double duration = std::chrono::duration_cast<chrono::nanoseconds>(stop_time - start_time).count();

    // number of samples per second
    uint64_t sample_rate = 624000;

    // number of taps to create a low pass filters
    //uint64_t rf_taps = 201;
    uint64_t fm_taps = 200;
    uint64_t audio_taps = 195;

    // offset from the center where we want to demodulate(Hz)
    int64_t rf_freq_offset = 0; // 115750;

    // rf frequency filter cutoff
    //int64_t fc_rf = 40000;

    // decimation factor
    int64_t rf_decimation_factor = 10;

    // the FM broadcast signal has a bandwidth(Hz)
    double desired_rf_sample_rate = (int64_t)(sample_rate / (double)rf_decimation_factor);

    // FM cutoff frequency
    int64_t fc_fm = 20800;

    // audio sample rate ==> 5 times the data bit rate
    int64_t audio_decimation_factor = 15;
    //int64_t am_sample_rate = 4160;
    double desired_audio_sample_rate = desired_rf_sample_rate/(double)audio_decimation_factor;

    // audio filter cutoff frequency(Hz)
    int64_t fc_am = 2400;
    int64_t am_offset = 2400;

    uint32_t min_distance = 2000;

    double x_min, x_max, delta;

    std::complex<double> cf_scale(1.0 / 2048.0, 0.0);

    //std::vector<float> sync_pulse = { -128, -128, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, -128, -128, -128, -128, -128, -128, -128 };

    cv::Mat sync_pulse = (cv::Mat_<int16_t>(1,39) << -128, -128, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, -128, -128, -128, -128, -128, -128, -128);

    //cv::Mat cv_x3;
    //cv::Mat cv_x4;
    //cv::Mat cv_x5;
    //cv::Mat cv_x6;
    cv::Mat cv_x7;
    cv::Mat cv_x8;
    cv::Mat cv_x9;
    cv::Mat cv_x10;
    cv::Mat cv_x11;
    cv::Mat cv_x12;
    cv::Mat img;

    try{

        // test code

        //std::unique_ptr<SDR_BASE> sdr = SDR_BASE::build();

        //sample_rate = sdr->get_rx_samplerate();
        // number of samples is equal to the number seconds to record times the samplerate
        num_samples = 15 * 3600 * sample_rate;

        std::vector<std::complex<int16_t>> samples;
        //std::string filename = "../../rx_record/recordings/137M800_0M624__640s_test4.bin";
        //std::string filename = "../../rx_record/recordings/137M000_1M000__600s_20221120_0955.bin";
//        std::string filename = "D:/data/RF/20240224/blade_F137.620M_SR0.624M_20240224_194922.sc16";
        std::string filename = "D:/data/RF/20240224/blade_F137.912M_SR0.624M_20240224_222353.sc16";


        read_iq_data(filename, samples);

        num_samples = samples.size();

        //-----------------------------------------------------------------------------
        // setup all of the filters and rotations
        //-----------------------------------------------------------------------------

        // decimation factor
        //int64_t rf_decimation_factor = (int64_t)(sample_rate / (double)desired_rf_sample_rate);

        // calculate the new sampling rate based on the original and the decimated sample rate
        //double decimated_sample_rate = sample_rate / (double)rf_decimation_factor;

        // RF low pass filter
        //std::vector<double> lpf_rf = DSP::create_fir_filter<double>(rf_taps, (desired_rf_sample_rate / 2.0) / (double)sample_rate, &DSP::hann_window);
        //cv::Mat cv_lpf_rf(1, lpf_rf.size(), CV_64FC1, lpf_rf.data());

        std::vector<double> lpf_fm = DSP::create_fir_filter<double>(fm_taps, fc_fm / desired_rf_sample_rate, &DSP::hann_window);
        cv::Mat cv_lpf_fm(1, lpf_fm.size(), CV_64FC1, lpf_fm.data());

        // find a decimation rate to achieve audio sampling rate
        //int64_t audio_decimation_factor = (int64_t)(decimated_sample_rate / (double)am_sample_rate);
        //double decimated_audio_sample_rate = decimated_sample_rate / (double)audio_decimation_factor;

        // scaling for FM demodulation
        double phasor_scale = 1.0 / (2.0 * M_PI);

        // FM low pass de-emphasis filter
        //std::vector<double> lpf_fm = DSP::create_fir_filter<double>(fm_taps, 1.0/(double)(decimated_sample_rate * 75e-6), &DSP::rectangular_window);
        //cv::Mat cv_lpf_fm(1, lpf_fm.size(), CV_64FC1, lpf_fm.data());

        // Audio low pass filter coefficients
        std::vector<double> lpf_am = DSP::create_fir_filter<double>(audio_taps, (fc_am) / (double)desired_rf_sample_rate, &DSP::hann_window);
        cv::Mat cv_lpf_am(1, lpf_am.size(), CV_64FC1, lpf_am.data());


        // print out the specifics
        std::cout << std::endl << "------------------------------------------------------------------" << std::endl;
        std::cout << "fs:         " << sample_rate << std::endl;
        //std::cout << "rx_freq:    " << sdr->get_rx_frequency() << std::endl;
        std::cout << "f_offset:   " << rf_freq_offset << std::endl;
        std::cout << "dec_rate:   " << rf_decimation_factor << std::endl;
        std::cout << "channel_bw: " << desired_rf_sample_rate << std::endl;
        //std::cout << "fs_d:       " << decimated_sample_rate << std::endl;
        std::cout << "dec_audio:  " << audio_decimation_factor << std::endl;
        std::cout << "fs_audio:   " << desired_audio_sample_rate << std::endl;
        //std::cout << "fs_audio:   " << decimated_audio_sample_rate << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl << std::endl;
              
        //sdr->start_single(cf_samples, num_samples);
        //sdr->wait_for_samples();

        /*
        Step 1: start a thread to grab the data and fill an alternating buffer.  This thread will run forever.  This thread needs to indicate
        when a capture is complete and ready to be processed

        Step 2: A read index variable will be used to track which buffer index to read from.  Process the the buffer
        - scale data to +/- 1
        - assume that we no longer need to capture data at an offset therefore no frequency rotation
        - low pass filter the data
        - decimate the data (look at combining the filtering and decimation if needed)
        - concatenate 3 decimated blocks with a rolloing FIFO object that will advance by 80% of a block
        - take the data in std::XXXX form and convert to cv::XXX structure
        - 
        
        
        */


        // start the thread to capture data
        run = true;
        //std::thread data_capture_thread = std::thread(temp_get_data, sample_rate);


        //-----------------------------------------------------------------------------
        // start the demodulation process
        //-----------------------------------------------------------------------------


        //// take the complex float vector data and rotate it
        ////cv::Mat cv_fc_rot(1, fc_rot.size(), CV_64FC2, fc_rot.data());
        ////cv::Mat cv_samples(1, cf_samples.size(), CV_64FC2, cf_samples.data());
        ////cv::Mat x2 = mul_cmplx(cv_samples, cv_fc_rot);
        ////std::vector<complex<float>> x2 = vec_ptws_mul(cf_samples, fc_rot);
        std::vector<std::complex<double>> cf_samples(num_samples);
        for (idx = 0; idx < num_samples; ++idx)
        {
            cf_samples[idx] = cf_scale * std::complex<double>(std::real(samples[idx]), std::imag(samples[idx]));
            //cf_samples[idx] = cf_scale * std::complex<double>(std::real(samples[idx]), std::imag(samples[idx])) * (complex<double>)std::exp(-2.0 * 1i * M_PI * (f_offset / (double)sample_rate) * (double)idx);

        }
        //cv::Mat cv_x1(1, num_samples, CV_64FC2, cf_samples.data());

        ////samples.clear();


        //// apply low pass filter to the signal 
        ////cv::filter2D(cv_x1, cv_x3, CV_64FC2, cv_lpf_rf, cv::Point(-1, -1), cv::BORDER_REFLECT_101);
        //cv::filter2D(cv_x1, cv_x3, CV_64FC2, cv_lpf_fm, cv::Point(-1, -1), cv::BORDER_REFLECT_101);


        // decimate the signal
        //x4 = x3(dec_seq);
        //std::vector<complex<float>> x4 = decimate_vec(cf_samples, rf_decimation_factor);
        //std::vector<complex<float>> x4 = decimate_vec(x3, rf_decimation_factor);
        //cv_cmplx_decimate(cv_x3, cv_x4, (double)rf_decimation_factor);

        // look at block processing like we would do with the SDR input
        double capture_time = 2.0;
        uint64_t block_size = floor(sample_rate * capture_time + 0.5);

        uint64_t num_blocks = floor((num_samples) / (double)block_size);

        cv_x10 = cv::Mat(1, 1, CV_64FC1, cv::Scalar(0.0));
        std::cout << "number of blocks to process: " << num_blocks << std::endl;

        for (idx = 0; idx < num_samples; idx += block_size)
        {
            std::cout << "processing block: " << idx; // << std::endl;
                // timing variables
            start_time = std::chrono::high_resolution_clock::now();

            std::vector<std::complex<double>> tmp_samples;
            std::copy(cf_samples.begin() + idx, // Start iterator for the source range
                cf_samples.begin() + (idx + block_size), // End iterator (exclusive) for the source range
                std::back_inserter(tmp_samples));

            std::vector<std::complex<double>> x4 = polyphase_decimate(tmp_samples, rf_decimation_factor, lpf_fm);
            //cv::Mat cv_x4(1, v_x4.size(), CV_64FC2, v_x4.data());

            //cv::filter2D(cv_x4, cv_x5, CV_64FC2, cv_lpf_fm, cv::Point(-1, -1), cv::BORDER_REFLECT_101);
            //for(idx=0; idx<10; ++idx)
            //    std::cout << x4[idx] << std::endl;

            // polar discriminator - x4(2:end).*conj(x4(1:end - 1));
            //x5 = x4(af::seq(1, af::end, 1)) * af::conjg(x4(af::seq(0, -2, 1)));
            //x5 = af::atan2(af::imag(x5), af::real(x5)) * phasor_scale;
            std::vector<double> x6 = polar_discriminator(x4, phasor_scale);
            //std::vector<float> x5 = polar_discriminator(x4, phasor_scale);
            //cv_polar_discriminator(cv_x4, cv_x6, phasor_scale);
            //cv::Mat cv_x6(1, x6.size(), CV_64FC1, x6.data());

            //for (idx = 0; idx < 10; ++idx)
            //    std::cout << x6[idx] << std::endl;

            // run the audio through the low pass de-emphasis filter
            //x6 = af::fir(af_lpf_de, x5);
            //std::vector<float> x6 = filter_vec(x5, lpf_audio);

            // run the audio through a second low pass filter before decimation
            //x6 = af::fir(af_lpf_a, x6);
            std::vector<std::complex<double>> x7 = frequency_shift(x6, (double)am_offset / (double)desired_rf_sample_rate);

//            cv::Mat cv_x7(1, x7.size(), CV_64FC2, x7.data());

            //for (idx = 0; idx < 10; ++idx)
            //    std::cout << x7[idx] << std::endl;

            //cv_x7 = cv_frequency_rotate(cv_x6, (double)am_offset / (double)desired_rf_sample_rate);

            //cv::filter2D(cv_x7, cv_x8, CV_64FC1, cv_lpf_am, cv::Point(-1, -1), cv::BORDER_REFLECT_101);

            std::vector<std::complex<double>> x8 = polyphase_decimate(x7, audio_decimation_factor, lpf_am);

            cv::Mat cv_x8(1, x8.size(), CV_64FC2, x8.data());

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

            cv_x9 = cv_cmplx_abs(cv_x8);

            // downsample the AM
            //cv::Mat cv_x9a;
            //cv_decimate(cv_x9, cv_x9a, audio_decimation_factor);

            cv::hconcat(cv_x10, cv_x9, cv_x10);
            stop_time = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<chrono::milliseconds>(stop_time - start_time).count();

            std::cout << " - " << duration << " milliseconds" << std::endl;

        }

        cv_x10 = cv_x10.colRange(1, cv_x10.cols).clone();
        cv::minMaxIdx(cv_x10, &x_min, &x_max);


        //auto result = std::minmax_element(x9.begin(), x9.end());
        //auto x_max = *std::max_element(x9.begin(), x9.end());
        //float x_min = *result.first;
        //float x_max = *result.second;

        delta = x_max - x_min;

        // Normalize the signal to px luminance values, discretize
        //std::vector<float> x11(x9.size());

        cv_x11 = ((255.0 / delta) * (cv_x10 - x_min));

        //cv_x11 = clamp(cv_x11, 0, 255);


        cv_x11.convertTo(cv_x12, CV_16SC1, 1, -128);

        //cv_x11 -= 128.0;

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
        while (idx <= cv_x12.total() - sync_pulse.total())
        {

            r.x = idx;

            cv::Mat tmp = cv_x12(r);

            auto corr = tmp.dot(sync_pulse) / (double)sync_pulse.total();

            //c5d(idx) = dot(sync, d5s(idx:idx + numel(sync) - 1)) / numel(sync);
            //corr = c5d(idx);
            //auto t2 = (*(peaks.end()-1)).first;

            // If previous peak is too far, we keep it but add this value as new
            if ((idx - peaks[peaks.size() - 1].first) > min_distance)
            {
                //peaks(end + 1, :) = [idx, corr];
                peaks.push_back(std::make_pair(idx, corr));
                idx += 500;
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

        img = cv::Mat::zeros(peaks.size(), 2080, CV_8UC1);
        cv_x11.convertTo(cv_x11, CV_8UC1, 1, 0);

        //for idx = 1:(size(peaks, 1) - 2)
        //    img = cat(1, img, d5(peaks(idx, 1) :peaks(idx, 1) + 2079)');
        //end
        //int64_t index;
        r.width = 2080;
        for (idx = 0; idx < peaks.size()-1; ++idx)
        {
            r.x = peaks[idx].first;
    
            cv_x11(r).copyTo(img(cv::Rect(0, idx, 2080, 1)));

        }
        
        //sdr->stop();
        cv::namedWindow("test", cv::WINDOW_NORMAL);
        cv::imshow("test", img);
        cv::waitKey(0);
        bp = 1;

    }
    catch(std::exception e)
    {
        std::cout << "error: " << e.what() << std::endl;
        std::cin.ignore();
    }

    
    return 0;
    
}   // end of main
