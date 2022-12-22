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

// Custom Includes
#include <iq_utils.h>
//#include <hackrf_common.h>

// Project Includes

const uint64_t block_size = 262144;
static std::vector<uint8_t> samples;

static uint64_t blocks_captured;

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


//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{

    uint32_t idx = 0;
    
    std::string args;

    double sample_rate = 8000000;

    uint32_t channel = 0;
    double freq = 314500000;
    double rx_gain = 20;
    double bw;
    double total_time;
    double setup_time;
    double lo_offset = 0.0;

    int32_t rv;

    uint8_t board_id = 0;

    // determine how many blocks to capture
    uint64_t num_blocks = 50;

    // allocate the memory for the samples, but do not actually init the container
    samples.reserve(num_blocks * block_size);

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




        // save the samples to a file
        std::cout << std::endl << "num samples captured: " << blocks_captured* block_size << "/" << samples.size() << std::endl;
        std::string save_filename = "../test_hackrf_save.bin";
        write_iq_data(save_filename, samples);

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
