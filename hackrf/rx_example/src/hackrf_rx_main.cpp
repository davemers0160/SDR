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

// HackRF includes
#include <hackrf.h>

// Custom Includes
#include <iq_utils.h>
#include <hackrf_common.h>

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
int rx_callback(hackrf_transfer* transfer)
{
    size_t samples_captured;

    ++blocks_captured;
    samples_captured = transfer->valid_length;
    fprintf(stderr, "samples_captured: %d\n", samples_captured);

    std::vector<uint8_t> tmp(transfer->buffer, transfer->buffer + samples_captured);

    std::copy(tmp.begin(), tmp.end(), std::back_inserter(samples));

    int bp = 1;

    return 0;
}   // end of rx_callback

//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{

    uint32_t idx = 0;
    
    // hackrf specific structs
    hackrf_device* dev = NULL;

    double sample_rate = 8000000;
    uint64_t freq = 314500000;
    uint32_t rx_gain = 20;

    int32_t rv;

    uint8_t board_id = 0;

    // determine how many blocks to capture
    uint64_t num_blocks = 50;

    // allocate the memory for the samples, but do not actually init the container
    samples.reserve(num_blocks * block_size);

    try
    {
        rv = hackrf_init();

        rv = select_hackf(&dev);
        if (rv != HACKRF_SUCCESS)
        {
            std::cout << "error opening HackRF: " << std::string(hackrf_error_name((enum hackrf_error)rv)) << std::endl;
        }

        rv = hackrf_board_id_read(dev, &board_id);
        if (rv == HACKRF_SUCCESS)
        {
            std::cout << "HackRF board ID: " << std::string(hackrf_board_id_name((enum hackrf_board_id)board_id)) << std::endl;
        }
        else
        {
            std::cout << "error getting board id: " << std::string(hackrf_error_name((enum hackrf_error)rv)) << std::endl;
        }

        rv = hackrf_set_sample_rate(dev, sample_rate);
        rv |= hackrf_set_freq(dev, freq);
        rv |= hackrf_set_vga_gain(dev, 20);
        rv |= hackrf_set_lna_gain(dev, rx_gain);

        std::cout << std::endl << "Press enter to collect." << std::endl;
        std::cin.ignore();
        
        // set the counter for the number of blocks to zero
        blocks_captured = 0;

        // HackRF captures data in 262144 byte blocks
        rv = hackrf_start_rx(dev, rx_callback, NULL);

        // wait for the correct number of blocks to be collected
        while (blocks_captured < num_blocks)
        {
            sleep_ms(10);
        }

        // stop the receive callback
        rv = hackrf_stop_rx(dev);

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
    // initialize the hackrf - required before opening
    
    rv = hackrf_close(dev); 
    rv = hackrf_exit();

    std::cout << std::endl << "Program complete.  Press Enter to close..." << std::endl;
    std::cin.ignore();
    
}   // end of main
