// ----------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)

#elif defined(__linux__)

#endif

#include <cstdint>
#include <cmath>
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

const double pi = 3.14159265358979323846;
const double pi2 = 2.0 * 3.14159265358979323846;

const std::complex<double> j(0, 1);

const uint64_t block_size = 262144 >> 1;
static std::vector<std::complex<int8_t>> samples;

static uint64_t blocks_captured;

//-----------------------------------------------------------------------------
int tx_callback(hackrf_transfer* transfer)
{
    size_t bytes_to_read = transfer->buffer_length;

    

    return 0;
}   // end of rx_callback

//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{

    uint32_t idx = 0, jdx = 0;
    
    // hackrf specific structs
    hackrf_device* dev = NULL;

    double sample_rate = 20000000;
    uint64_t freq = 315000000;
    
    int32_t rv;

    uint8_t board_id = 0;

    // determine how many blocks to capture
    uint64_t num_blocks = 100;

    // allocate the memory for the samples, but do not actually init the container
    samples.reserve(num_blocks * block_size);

    //generate IQ samples - simple FSK
    uint32_t data = 0xAB42F58C;     // random 32-bit data
    uint32_t bit_mask = 1;

    // the number of samples per bit
    uint32_t bit_samples = 20000;

    // the frequency offset for the FSK modulation - 100kHz, normalized by the sample rate
    double freq_offset = 20000.0 / sample_rate;

    double amplitude = 120;

    for (idx = 0; idx < sizeof(data)*8; ++idx)
    {
        // if true this is a one
        if (data & bit_mask)
        {
            for (jdx = 0; jdx < bit_samples; ++jdx)
            {
                samples.push_back(static_cast<std::complex<int8_t>>(amplitude * std::exp(j * pi2 * freq_offset * (double)jdx)));
            }
        }
        // if not this is a zero
        else
        {
            for (jdx = 0; jdx < bit_samples; ++jdx)
            {
                samples.push_back(static_cast<std::complex<int8_t>>(amplitude * std::exp(-j * pi2 * freq_offset * (double)jdx)));
            }
        }

        bit_mask <<= 1;

    }

    std::string save_filename = "../test_hackrf_save.bin";
    write_iq_data(save_filename, samples);


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

        // set the counter for the number of blocks to zero
        blocks_captured = 0;

        // HackRF captures data in 262144 byte blocks
        rv = hackrf_start_tx(dev, tx_callback, NULL);

        // wait for the correct number of blocks to be collected
        while (blocks_captured < num_blocks);

        // stop the receive callback
        rv = hackrf_stop_tx(dev);

        // save the samples to a file
        //std::string save_filename = "../test_hackrf_save.bin";
        //write_iq_data(save_filename, samples);

        int bp = 2;

    }
    catch (std::exception e)
    {
        std::cout << "error: " << e.what() << std::endl;
    }
    // initialize the hackrf - required before opening
    
    rv = hackrf_close(dev); 
    rv = hackrf_exit();
    
}   // end of main
