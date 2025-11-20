// ----------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
#define _USE_MATH_DEFINES
#define NOMINMAX

#elif defined(__linux__)

#endif

#include <cstdint>
#include <cmath>
#include <csignal>

#include <iostream>
#include <sstream>
#include <fstream>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <complex>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <string>


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
#include "data_logger.h"

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
//volatile std::vector<bool> data_ready(2, false);
volatile int32_t read_index = 1;
volatile int32_t write_index = 0;
volatile bool is_running = false;
volatile bool data_ready = false;

const uint32_t timeout_ms = 10000;


//-----------------------------------------------------------------------------
void sig_handler(int sig_num)
{
    if ((sig_num == SIGINT) | (sig_num == SIGTERM))
    {
        //fprintf(stderr, "received SIGINT: %d\n", sig_num);
        std::cout << "Received SIGINT: " << sig_num << std::endl;
        is_running = false;
        //transmit_thread_running = false;
        //recieve_thread_running = false;
    }

}   // end of sig_handler

//-----------------------------------------------------------------------------
// thread function to simulate data capture
inline void temp_get_data(bladerf_sample_rate sample_rate, double capture_time)
{
    int32_t blade_status = 0;

    uint64_t block_size = floor(sample_rate * capture_time + 0.5);
    std::cout << info << "Starting thread..." << std::endl;

    iq_data_queue[0].resize(block_size);
    iq_data_queue[1].resize(block_size);

    // data to read in
    std::string filename = "D:/data/RF/20240224/blade_F137.912M_SR0.624M_20240224_222353.sc16";
    std::vector<std::complex<int16_t>> samples;

    std::cout << info << "Thread: Loading Data..." << std::endl;

    read_iq_data(filename, samples);
    uint64_t num_samples = samples.size();

    // delete some extra samples to match the capture buffers and size
    uint64_t rem = num_samples % (2 * block_size);
    if (rem > 0)
    {
        samples.erase(samples.end() - rem, samples.end());
    }

    std::cout << info << "Thread: Processing Data..." << std::endl;
    write_index = 0;
    read_index = 1;
    uint64_t sample_index = 0;
    while (is_running == true)
    {
        // take the samples and place them into the right buffer
        std::copy(samples.begin() + sample_index, // Start iterator for the source range
            samples.begin() + sample_index + block_size, // End iterator (exclusive) for the source range
            iq_data_queue[write_index].begin());

        // sleep to simulate the data capture of the SDR
        std::this_thread::sleep_for(std::chrono::milliseconds((uint32_t)(capture_time*999)));

        write_index = (write_index + 1) & 0x01;
        read_index = (read_index + 1) & 0x01;

        data_ready = true;
        // this just resets the index back to the beginning to forever cycle through the data
        sample_index += block_size;
        if (sample_index >= samples.size())
        {
            sample_index = 0;
            std::cout << info << "Thread: Resetting sample_index" << std::endl;

        }
    }

    std::cout << info << "Thread stopped!" << std::endl;

}


//-----------------------------------------------------------------------------
void get_data(struct bladerf* dev, bladerf_sample_rate sample_rate, double capture_time)
{
    int32_t blade_status;

    std::cout << info << "Starting thread..." << std::endl;

    uint64_t block_size = floor(sample_rate * capture_time + 0.5);

    iq_data_queue[0].resize(block_size);
    iq_data_queue[1].resize(block_size);

    while(is_running == true)
    {
        
        blade_status = bladerf_sync_rx(dev, (void*)iq_data_queue[write_index].data(), block_size, NULL, timeout_ms);
        if (blade_status != 0)
        {
            std::cout << warning << "Unable to get the required number of samples: " << std::string(bladerf_strerror(blade_status)) << std::endl;
        }

        write_index = (write_index + 1) & 0x01;
        read_index = (read_index + 1) & 0x01;
        data_ready = true;
    }

    std::cout << info << "Thread stopped!" << std::endl;
}

//-----------------------------------------------------------------------------
inline std::vector<std::pair<int64_t, double>> get_correlation_peaks(cv::Mat src, cv::Mat sync_pulse)
{
    uint64_t index = 0;
    std::vector<std::pair<int64_t, double>> peaks = { {0, 0 } };
    uint32_t min_distance = 2000;
    cv::Rect sp_rect(0, 0, sync_pulse.total(), 1);

    cv::Mat tmp;

    while (index <= src.total() - sync_pulse.total())
    {

        sp_rect.x = index;

        tmp = src(sp_rect);
        auto corr = tmp.dot(sync_pulse) / (double)sync_pulse.total();

        // If previous peak is too far, we keep it but add this value as new
        if ((index - peaks[peaks.size() - 1].first) > min_distance)
        {
            peaks.push_back(std::make_pair(index, corr));
            index += 500;
        }
        else if (corr > peaks[peaks.size() - 1].second)
        {
            peaks[peaks.size() - 1] = std::make_pair(index, corr);
        }

        ++index;
    }

    return peaks;
}

//-----------------------------------------------------------------------------
void add_line_to_image(cv::Mat& img, const cv::Mat& new_row) 
{
    // 1. Shift existing data up by one row
    // Create a Rect (Region of Interest) starting from the second row (y=1)
    // to the bottom, with the original width
    cv::Rect roi(0, 1, img.cols, img.rows - 1);
    cv::Mat src = img(roi);

    // Create a destination Rect starting from the top (y=0)
    // to the second-to-last row, with the original width
    cv::Rect dst_roi(0, 0, img.cols, img.rows - 1);
    cv::Mat dst = img(dst_roi);

    // Copy the source region to the destination region, effectively shifting content up
    src.copyTo(dst);

    // 2. Add the new row to the bottom
    // Define the ROI for the very last row
    cv::Rect bottom_row_roi(0, img.rows - 1, img.cols, 1);
    cv::Mat bottom_row = img(bottom_row_roi);

    // Copy the new row data into the bottom row ROI
    if (new_row.cols == img.cols && new_row.type() == img.type())
    {
        new_row.copyTo(bottom_row);
    }
    else 
    {
        std::cerr << warning << "Error: New row dimensions or type mismatch!" << std::endl;
    }
}   // end of add_line_to_image

//-----------------------------------------------------------------------------
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
//template <typename T>
//std::vector<T> scale_vec(std::vector<T>& v1, T scale)
//{
//    std::vector<T> res(v1.size());
//
//    auto v1_itr = v1.begin();
//    auto v1_end = v1.end();
//    auto res_itr = res.begin();
//
//    for (; v1_itr != v1_end; ++v1_itr, ++res_itr)
//    {
//        *res_itr = scale * (*v1_itr);
//    }
//
//    return res;
//}

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
int main(int argc, char** argv)
{
    int bp = 0;
    int64_t idx, jdx, blk_idx;
    
    uint64_t num_samples;
    //uint64_t block_size = 624000;

    // timing variables
    auto start_time = std::chrono::high_resolution_clock::now();
    auto stop_time = std::chrono::high_resolution_clock::now();
    double duration = std::chrono::duration_cast<chrono::nanoseconds>(stop_time - start_time).count();

    // bladeRF variable
    struct bladerf_devinfo* device_list = NULL;
    struct bladerf_devinfo dev_info;
    struct bladerf* dev;
    int bladerf_num;
    int blade_status;

    bladerf_channel rx = BLADERF_CHANNEL_RX(0);
    bladerf_frequency rx_freq = 137620000;
    bladerf_sample_rate sample_rate = 624000;
    bladerf_bandwidth rx_bw = 624000;
    bladerf_gain rx1_gain = 65;

    const uint32_t num_buffers = 16;
    const uint32_t buffer_size = 1024 * 4;        // must be a multiple of 1024
    const uint32_t num_transfers = 8;
    int32_t num_devices;

    // block capture time
    double capture_time = 2.0;

    // number of taps to create a low pass filters
    uint64_t fm_taps = 200;
    uint64_t audio_taps = 195;

    // decimation factor
    int64_t rf_decimation_factor = 10;
    double desired_rf_sample_rate = (int64_t)(sample_rate / (double)rf_decimation_factor);

    // FM cutoff frequency
    int64_t fc_fm = 20800;

    // audio sample rate ==> 5 times the data bit rate
    int64_t audio_decimation_factor = 15;
    double desired_audio_sample_rate = desired_rf_sample_rate/(double)audio_decimation_factor;

    // audio filter cutoff frequency(Hz)
    int64_t fc_am = 2400;
    int64_t am_offset = 2400;

    uint32_t min_distance = 2000;

    double x_min, x_max, delta;

    // sample scale value to bring samples to +/- 1
    std::complex<double> cf_scale(1.0 / 2048.0, 0.0);

    // scaling for FM demodulation
    double phasor_scale = 1.0 / (2.0 * M_PI);

    // apt sync pulse
    cv::Mat sync_pulse = (cv::Mat_<int16_t>(1,39) << -128, -128, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, 127, 127, -128, -128, -128, -128, -128, -128, -128, -128, -128);
    cv::Rect sp_rect(0, 0, sync_pulse.total(), 1);
    cv::Rect img_rect(0, 0, 2080, 1);

    //-----------------------------------------------------------------------------
    // setup all of the filters and rotations
    //-----------------------------------------------------------------------------
    // RF low pass filter
    std::vector<double> lpf_fm = DSP::create_fir_filter<double>(fm_taps, fc_fm / desired_rf_sample_rate, &DSP::hann_window);

    // Audio low pass filter coefficients
    std::vector<double> lpf_am = DSP::create_fir_filter<double>(audio_taps, (fc_am) / (double)desired_rf_sample_rate, &DSP::hann_window);
        
    std::vector<double> x10;
    cv::Mat cv_x10;
    cv::Mat cv_x11;
    cv::Mat cv_x12;
    cv::Mat img = cv::Mat::zeros(700, 2080, CV_8UC1);

    cv::namedWindow("test", cv::WINDOW_NORMAL);
    cv::imshow("test", img);
    cv::waitKey(10);

    try
    {
        //std::unique_ptr<SDR_BASE> sdr = SDR_BASE::build();
        if (argc < 2) 
        {
            std::cerr << warning << "Usage: " << argv[0] << " <Frequncy in MHz>" << std::endl;
            rx_freq = 137620000;
        }
        else
        {
            rx_freq = static_cast<bladerf_frequency>(std::stod(argv[1]) * 1e6);
        }

        num_devices = bladerf_get_device_list(&device_list);
        bladerf_num = select_bladerf(num_devices, device_list);

        //-----------------------------------------------------------------------------
        if (bladerf_num < 0)
        {
            std::cout << warning << "could not detect any bladeRF devices..." << std::endl;
            std::cin.ignore();
            return 0;
        }
        std::cout << std::endl;

        blade_status = bladerf_open(&dev, ("*:serial=" + std::string(device_list[bladerf_num].serial)).c_str());
        if (blade_status != 0)
        {
            std::cout << warning << "Unable to open device: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            return blade_status;
        }

        blade_status = bladerf_get_devinfo(dev, &dev_info);
        if (blade_status != 0)
        {
            std::cout << warning << "Unable to get the device info: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            return blade_status;
        }
        std::cout << std::endl << dev_info << std::endl;

        // set the frequency, sample_rate and bandwidth
        blade_status = bladerf_set_frequency(dev, rx, rx_freq);
        blade_status = bladerf_get_frequency(dev, rx, &rx_freq);
        blade_status = bladerf_set_sample_rate(dev, rx, sample_rate, &sample_rate);
        blade_status = bladerf_set_bandwidth(dev, rx, rx_bw, &rx_bw);

        // configure the sync to receive/transmit data
        blade_status = bladerf_sync_config(dev, BLADERF_RX_X1, BLADERF_FORMAT_SC16_Q11, num_buffers, buffer_size, num_transfers, timeout_ms);

        // enable the rx channel RF frontend
        blade_status = bladerf_enable_module(dev, BLADERF_RX, true);

        // the gain must be set after the module has been enabled
        blade_status = bladerf_set_gain_mode(dev, rx, BLADERF_GAIN_MANUAL);
        blade_status = bladerf_set_gain(dev, rx, rx1_gain);
        blade_status = bladerf_get_gain(dev, rx, &rx1_gain);

        // print out the specifics
        std::cout << std::endl << "------------------------------------------------------------------" << std::endl;
        std::cout << "sample_rate: " << sample_rate << std::endl;
        std::cout << "rx_freq:     " << rx_freq << std::endl;
        std::cout << "rx_bw:       " << rx_bw << std::endl;
        std::cout << "rx1_gain:    " << rx1_gain << std::endl;
        //std::cout << "time:        " << duration << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl << std::endl;

        // print out the specifics
        //std::cout << std::endl << "------------------------------------------------------------------" << std::endl;
        std::cout << "dec_rate:   " << rf_decimation_factor << std::endl;
        std::cout << "channel_bw: " << desired_rf_sample_rate << std::endl;
        std::cout << "dec_audio:  " << audio_decimation_factor << std::endl;
        std::cout << "fs_audio:   " << desired_audio_sample_rate << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl << std::endl;
              
        //sdr->start_single(cf_samples, num_samples);
        //sdr->wait_for_samples();

        /*
        Step 1: start a thread to grab the data and fill an alternating buffer.  This thread will run forever.  This thread needs to indicate
        when a capture is complete and ready to be processed

        Step 2: A read index variable will be used to track which buffer index to read from.  Process the the buffer
        - scale, filter, decimate, FM demod, filter, decimate, shift, AM demod, min/max scale 
                
        Step 3: find the correlation peaks and create image

        Step 4: delete processed samples from x10 based on last peak, but leave 2080 samples ahead of the last peak

        Step 5: repeat with new data
        */

        // handle SIGINT signals
        if (signal(SIGINT, sig_handler) == SIG_ERR)
        {
            std::cerr << warning << "Unable to catch SIGINT signals" << std::endl;
        }
        // handle SIGTERM signals
        if (signal(SIGTERM, sig_handler) == SIG_ERR)
        {
            std::cerr << warning << "Unable to catch SIGTERM signals" << std::endl;
        }

        // start the thread to capture data
        is_running = true;
        // file read data capture thread
        //std::thread data_capture_thread = std::thread(temp_get_data, sample_rate, capture_time);
        // bladerf data capture thread
        std::thread data_capture_thread = std::thread(get_data, std::ref(dev), sample_rate, capture_time);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));


        //-----------------------------------------------------------------------------
        // start the demodulation process
        //-----------------------------------------------------------------------------
        //std::vector<std::complex<double>> cf_samples(num_samples);
        //for (idx = 0; idx < num_samples; ++idx)
        //{
        //    cf_samples[idx] = cf_scale * std::complex<double>(std::real(samples[idx]), std::imag(samples[idx]));
        //    //cf_samples[idx] = cf_scale * std::complex<double>(std::real(samples[idx]), std::imag(samples[idx])) * (complex<double>)std::exp(-2.0 * 1i * M_PI * (f_offset / (double)sample_rate) * (double)idx);
        //}

        // look at block processing like we would do with the SDR input
        uint64_t block_size = floor(sample_rate * capture_time + 0.5);
        //uint64_t num_blocks = floor((num_samples) / (double)block_size);

        while (data_ready == false);

        //std::cout << "number of blocks to process: " << num_blocks << std::endl;
        std::cout << info << "processing block size: " << block_size << std::endl;

        //for (idx = 0; idx < num_samples; idx += block_size)
        while(is_running == true)
        {
            data_ready = false;
//            std::cout << "processing block: " << idx; // << std::endl;
            //std::cout << "processing block: " << block_size; // << std::endl;

            // timing variables
            start_time = std::chrono::high_resolution_clock::now();

            // scale the input vector - read_index
            std::vector<std::complex<double>> cf_samples(block_size);
            for (jdx = 0; jdx < block_size; ++jdx)
            {
                cf_samples[jdx] = cf_scale * std::complex<double>(std::real(iq_data_queue[read_index][jdx]), std::imag(iq_data_queue[read_index][jdx]));
            }

            std::vector<std::complex<double>> tmp_samples;
            //std::copy(cf_samples.begin() + idx, // Start iterator for the source range
            //    cf_samples.begin() + (idx + block_size), // End iterator (exclusive) for the source range
            //    std::back_inserter(tmp_samples));
            std::copy(cf_samples.begin(), cf_samples.end(), std::back_inserter(tmp_samples));

            std::vector<std::complex<double>> x4 = polyphase_decimate(tmp_samples, rf_decimation_factor, lpf_fm);

            std::vector<double> x6 = polar_discriminator(x4, phasor_scale);

            std::vector<std::complex<double>> x7 = frequency_shift(x6, (double)am_offset / (double)desired_rf_sample_rate);

            // decimate and filter the audio sequence
            std::vector<std::complex<double>> x8 = polyphase_decimate(x7, audio_decimation_factor, lpf_am);
            //cv::Mat cv_x8(1, x8.size(), CV_64FC2, x8.data());

            std::vector<double> x9(x8.size());
            for (jdx = 0; jdx < x8.size(); ++jdx)
            {
                x9[jdx] = std::abs(x8[jdx]);
            }
            
            x10.insert(x10.end(), x9.begin(), x9.end());

            auto minmax_pair = std::minmax_element(x10.begin(), x10.end());
            x_min = *minmax_pair.first;
            x_max = *minmax_pair.second;
            delta = x_max - x_min;

            cv::Mat cv_x10(1, x10.size(), CV_64FC1, x10.data());

            // Normalize the signal to px luminance values, discretize
            cv_x11 = ((255.0 / delta) * (cv_x10 - x_min));

            cv_x11.convertTo(cv_x12, CV_16SC1, 1, -128);

            //std::vector<std::pair<int64_t, float>> peaks = { {0, 0 } };
            std::vector<std::pair<int64_t, double>> peaks = get_correlation_peaks(cv_x12, sync_pulse);

            cv_x11.convertTo(cv_x11, CV_8UC1, 1, 0);

            for (jdx = 0; jdx < peaks.size() - 1; ++jdx)
            {
                if (peaks[jdx].first + img_rect.width >= cv_x11.cols)
                    continue;

                img_rect.x = peaks[jdx].first;
                //cv_x11(img_rect).copyTo(img(cv::Rect(0, idx, 2080, 1)));
                cv::Mat image_line = cv_x11(img_rect);
                add_line_to_image(img, image_line);

                cv::imshow("test", img);
                cv::waitKey(1);
            }

            // remove the processed samples
            uint32_t n = (*(peaks.end() - 1)).first;
            n = std::max(n - 500UL, 0UL);

            x10.erase(x10.begin(), x10.begin() + n);

            stop_time = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<chrono::milliseconds>(stop_time - start_time).count();

            std::cout << info << "Processing time: " << duration << " milliseconds" << std::endl;

            while (data_ready == false);
        }

        //auto minmax_pair = std::minmax_element(x10.begin(), x10.end());
        //x_min = *minmax_pair.first;
        //x_max = *minmax_pair.second;
        //delta = x_max - x_min;

        //cv::Mat cv_x10(1, x10.size(), CV_64FC1, x10.data());
        //
        //// Normalize the signal to px luminance values, discretize
        //cv_x11 = ((255.0 / delta) * (cv_x10 - x_min));

        //cv_x11.convertTo(cv_x12, CV_16SC1, 1, -128);

        ////std::vector<std::pair<int64_t, float>> peaks = { {0, 0 } };
        //std::vector<std::pair<int64_t, double>> peaks = get_correlation_peaks(cv_x12, sync_pulse);

        ////img = cv::Mat::zeros(peaks.size(), 2080, CV_8UC1);
        //cv_x11.convertTo(cv_x11, CV_8UC1, 1, 0);

        //// use the peak index locations to crop the data and assign to the image row
        //for (idx = 0; idx < peaks.size()-1; ++idx)
        //{
        //    img_rect.x = peaks[idx].first;   
        //    cv_x11(img_rect).copyTo(img(cv::Rect(0, idx, 2080, 1)));
        //}
        
        //sdr->stop();
        std::cout << info << "Press any key to close!" << std::endl;

        cv::imshow("test", img);
        cv::waitKey(0);
        bp = 1;

        data_capture_thread.join();

        //char key = 0;
        //while (key != 'q')
        //{
        //    cv::Mat tmp(1, 2080, CV_8UC1);
        //    cv::randu(tmp, cv::Scalar(0), cv::Scalar(255));

        //    start_time = std::chrono::high_resolution_clock::now();

        //    scrollImageUp(img, tmp);

        //    stop_time = std::chrono::high_resolution_clock::now();
        //    duration = std::chrono::duration_cast<chrono::microseconds>(stop_time - start_time).count();
        //    std::cout << " - " << duration << " microseconds" << std::endl;

        //    cv::imshow("test", img);
        //    key = cv::waitKey(10);

        //}

    }
    catch(std::exception e)
    {
        std::cout << "error: " << e.what() << std::endl;
        std::cin.ignore();
    }

    cv::destroyAllWindows();
    return 0;
    
}   // end of main
