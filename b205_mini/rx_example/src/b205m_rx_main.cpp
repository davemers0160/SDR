// ----------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
#include <Windows.h>
#elif defined(__linux__)

#endif

#include <cstdint>
#include <ctime>
#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <complex>

// UHD includes
// will need to add the following to the environment PATH
// PATH=C:\local\boost_1_81_0\lib64-msvc-14.2;C:\Program Files\UHD\bin;%PATH%
#include <uhd.h>
#include <uhd/usrp/multi_usrp.hpp>
//#include <uhd/types/device_addr.hpp>
#include <uhd/utils/thread.hpp>

// Custom Includes
#include <iq_utils.h>
//#include <hackrf_common.h>

// Project Includes

//-----------------------------------------------------------------------------
inline void sleep_ms(uint32_t value)
{

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
    Sleep(value);
#else
    const timespec delay[] = { 0, (uint32_t)(value * 1000000) };
    nanosleep(delay, NULL);
#endif

}   // end of sleep_ms

//-----------------------------------------------------------------------------
void receive_data(uhd::usrp::multi_usrp::sptr usrp,
    const size_t& channel,
    size_t samps_per_buff,
    unsigned long long num_requested_samples,
    std::vector<std::complex<int16_t>> &samples
)
{
    uint64_t num_total_samps = 0;
    std::string cpu_format = "sc16";        //"";        // complex<int16_t>
    std::string wire_format = "sc16";       // Q16 I16
    bool enable_size_map = false;
    bool continue_on_bad_packet = false;

    try
    {
        // create a receive streamer
        uhd::stream_args_t stream_args(wire_format);
        std::vector<size_t> channel_nums;
        channel_nums.push_back(channel);
        stream_args.channels = channel_nums;
        uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

        samps_per_buff = rx_stream->get_max_num_samps();
        std::cout << "Max number of samples per buffer: " << samps_per_buff << std::endl;
        
        uhd::rx_metadata_t md;
        std::vector<std::complex<int16_t>> buff(samps_per_buff);

        //bool overflow_message = true;

        // setup streaming
        uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);

        stream_cmd.num_samps = size_t(num_requested_samples);
        stream_cmd.stream_now = true;
        stream_cmd.time_spec = uhd::time_spec_t();
        rx_stream->issue_stream_cmd(stream_cmd);

        //typedef std::map<size_t, size_t> SizeMap;
        //SizeMap mapSizes;
        //const auto start_time = std::chrono::steady_clock::now();
        //const auto stop_time = start_time + std::chrono::milliseconds(int64_t(1000));

        // Track time and samps between updating the BW summary
        //auto last_update = start_time;
        //unsigned long long last_update_samps = 0;

        samples.clear();

        // Run this loop until either time expired (if a duration was given), until
        // the requested number of samples were collected (if such a number was
        // given), or until Ctrl-C was pressed.
        while (num_total_samps < num_requested_samples)
        {
            //const auto now = std::chrono::steady_clock::now();

            size_t num_rx_samps = rx_stream->recv(&buff.front(), buff.size(), md, 3.0, enable_size_map);

            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
                std::cout << "Timeout while streaming" << std::endl;
                break;
            }

            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW)
            {
                std::string error_str = "Got an overflow indication. Please consider the following:\n";
                error_str += "  Your write medium must sustain a rate of " + std::to_string((usrp->get_rx_rate(channel) * sizeof(std::complex<int16_t>) / 1.0e6)) + "MB / s.\n";
                error_str += "  Dropped samples will not be written to the file.\n";

                std::cerr << error_str << std::endl;
                continue;
            }

            if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
                std::string error_str = "Receiver error: " + md.strerror();

                if (continue_on_bad_packet) {
                    std::cerr << error_str << std::endl;
                    continue;
                }
                else
                    throw std::runtime_error(error_str);
            }

            num_total_samps += num_rx_samps;

            std::copy(buff.begin(), buff.end(), std::back_inserter(samples));
        }
        //const auto actual_stop_time = std::chrono::steady_clock::now();

        stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
        rx_stream->issue_stream_cmd(stream_cmd);
    }
    catch (std::exception& e)
    {
        std::string error_string = "Error: " + std::string(e.what()) + "\n";
        error_string += "File: " + std::string(__FILE__) + ", Function: " + std::string(__FUNCTION__) + ", Line #: " + std::to_string(__LINE__);
        std::cout << error_string << std::endl;
    }

}

//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{

    uint32_t idx = 0;
    
    std::string args;

    double sample_rate = 4000000;

    uint32_t channel = 0;
    double freq = 314500000;
    double rx_gain = 20;
    double bw;
    double lo_offset = 0.0;

    int32_t rv;


    // determine how many blocks to capture
    uint64_t num_samples = (uint64_t)(1.0 * sample_rate);

    // allocate the memory for the samples, but do not actually init the container
    static std::vector<std::complex<int16_t>> samples;
    samples.reserve(num_samples);

    try
    {
        // declare a UHD address type and put the usrp type in it as a string
        uhd::device_addr_t dev_addr("B205mini");

        // create an object for the usrp sdr
        uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(dev_addr);
        //std::cout << "Using Device: " << usrp->get_pp_string() << std::endl << std::endl;

        // set the samplerate
        usrp->set_rx_rate(sample_rate, channel);
        std::cout << "Actual RX Rate: " << (usrp->get_rx_rate(channel) / 1e6) << " MHz" << std::endl << std::endl;

        // set the center frequency and LO offset
        uhd::tune_request_t tune_request(freq, lo_offset);
        usrp->set_rx_freq(tune_request, channel);
        std::cout << "Actual RX Freq: " << (usrp->get_rx_freq(channel) / 1e6) << " MHz" << std::endl << std::endl;

        // set the rx gain
        usrp->set_rx_gain(rx_gain, channel);
        std::cout << "Actual RX Gain: " <<  usrp->get_rx_gain(channel) << " dB" << std::endl << std::endl;

        // set the antenna
        //usrp->set_rx_antenna(ant, channel);

        std::cout << std::endl << "Press enter to collect." << std::endl;
        std::cin.ignore();


        // create a receive streamer
        // linearly map channels (index0 = channel0, index1 = channel1, ...)
        //std::string format = "sc16";
        //uhd::stream_args_t stream_args(format); // complex floats


        // received the samples
        uint64_t samps_per_buff = 4096 * 8;
        receive_data(usrp, channel, samps_per_buff, num_samples, samples);

        // save the samples to a file
        std::cout << std::endl << "num samples captured: " << samples.size() << "/" << num_samples << std::endl;

        std::string save_filename = "../test_b205mini_save.bin";
        write_qi_data(save_filename, samples);

        int bp = 2;

    }
    catch (std::exception e)
    {
        std::cout << "error: " << e.what() << std::endl;
        std::cin.ignore();
    }


    std::cout << std::endl << "Program complete.  Press Enter to close..." << std::endl;
    std::cin.ignore();
    
}   // end of main
