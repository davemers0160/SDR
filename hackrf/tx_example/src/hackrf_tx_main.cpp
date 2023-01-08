// ----------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
#include <Windows.h>
#elif defined(__linux__)

#endif

#include <cstdint>
#include <ctime>
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

static std::vector<uint8_t> samples;

static uint64_t data_index;
static bool tx_complete;

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
int tx_callback(hackrf_transfer* transfer)
{
    size_t bytes_to_xfer = transfer->buffer_length;
    size_t bytes_remaining = samples.size() - data_index;
    //size_t bytes_to_read;
    //size_t bytes_read;
    uint64_t idx;

    if (tx_complete == true)
        return 0;

    // check the current index and the transfer size to see where we are at in the samples
    if (bytes_remaining >= bytes_to_xfer)
    {
        // start at index == 0
        // if the size of the samples buffer is larger than the transfer->buffer length then fill the transfer buffer
        // increment the index by the number of bytes_to_xsfer
        //std::copy(samples.begin()+data_index, samples.begin() + data_index+ bytes_to_xfer, transfer->buffer);
        for (idx = 0; idx < bytes_to_xfer; ++idx)
        {
            transfer->buffer[idx] = samples[data_index + idx];
        }
    
        transfer->valid_length = bytes_to_xfer;
        data_index += bytes_to_xfer;
        return 0;
    }
    else
    {
        // if the number of the remaining samples is less than the transfer buffer fill the buffer with just those samples
        //std::copy(samples.begin() + data_index, samples.begin() + data_index + bytes_remaining, transfer->buffer);
        for (idx = 0; idx < bytes_remaining; ++idx)
        {
            transfer->buffer[idx] = samples[data_index + idx];
        }

        // set the transfer valid length to the number of remaining bytes
        transfer->valid_length = bytes_remaining;
        data_index = 0;
        tx_complete = true;
        return 0;
    }
    

}   // end of tx_callback

//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{

    uint32_t idx = 0, jdx = 0;
    
    // hackrf specific structs
    hackrf_device* dev = NULL;

    double sample_rate = 10000000;
    uint64_t freq = 314500000;
    uint32_t tx_gain = 6;

    int32_t rv;
    int bp = 1;

    // determine how many blocks to capture
    uint64_t num_blocks = 100;

    std::complex<double> tmp_val;

    //generate IQ samples - simple FSK
    uint64_t data = 0xAB42F58C15ACFE37;     // random 64-bit data
    uint64_t bit_mask = 1;

    // the number of samples per bit
    double bit_length = 1e-2;
    uint32_t bit_samples = (uint32_t)(sample_rate * bit_length);

    // the frequency offset for the FSK modulation - 100kHz, normalized by the sample rate
    double freq_offset = 25000.0 / sample_rate;

    double amplitude = 120;

    for (idx = 0; idx < sizeof(data)*8; ++idx)
    {
        // if true this is a one
        if (data & bit_mask)
        {
            for (jdx = 0; jdx < bit_samples; ++jdx)
            {
                tmp_val = amplitude * (std::exp(j * pi2 * freq_offset * (double)jdx));
                samples.push_back((int8_t)(tmp_val.real()));
                samples.push_back((int8_t)(tmp_val.imag()));
            }
        }
        // if not this is a zero
        else
        {
            for (jdx = 0; jdx < bit_samples; ++jdx)
            {
                tmp_val = amplitude * (std::exp(j * pi2 * (-freq_offset) * (double)jdx));
                samples.push_back((int8_t)(tmp_val.real()));
                samples.push_back((int8_t)(tmp_val.imag()));
            }
        }

        bit_mask <<= 1;

    }

    //std::string save_filename = "../test_hackrf_save.bin";
    //write_iq_data(save_filename, samples);


    try
    {

        rv = hackrf_init();
        if (rv != HACKRF_SUCCESS)
        {
            std::cout << "error initializing HackRF: " << std::string(hackrf_error_name((enum hackrf_error)rv)) << std::endl;
            std::cout << "Press enter to close" << std::endl;
            std::cin.ignore();
            return -1;
        }

        rv = select_hackf(&dev);
        if (rv != HACKRF_SUCCESS)
        {
            std::cout << "error opening HackRF: " << std::string(hackrf_error_name((enum hackrf_error)rv)) << std::endl;
            std::cout << "Press enter to close" << std::endl;
            std::cin.ignore();
            return -1;
        }

        get_hack_info(dev);

        rv = hackrf_set_sample_rate(dev, sample_rate);
        rv |= hackrf_set_freq(dev, freq);
        rv |= hackrf_set_txvga_gain(dev, tx_gain);
        if (rv != HACKRF_SUCCESS)
        {
            fprintf(stderr, "hackrf failed: %s (%d)\n", hackrf_error_name((enum hackrf_error)rv), rv);
            //return EXIT_FAILURE;
        }

            
        for (idx = 0; idx < 20; ++idx)
        {
            data_index = 0;
            tx_complete = false;

            rv = hackrf_start_tx(dev, tx_callback, NULL);
            if (rv != HACKRF_SUCCESS) 
            {
                fprintf(stderr, "hackrf_start_tx() failed: %s (%d)\n", hackrf_error_name((enum hackrf_error)rv), rv);
                //return EXIT_FAILURE;
            }

            //while ((hackrf_is_streaming(dev) == HACKRF_TRUE));
            while (!tx_complete)
            {
                sleep_ms(50);
            }

            sleep_ms(20);

            // stop the transmit callback
            rv = hackrf_stop_tx(dev);
            if (rv != HACKRF_SUCCESS)
            {
                fprintf(stderr, "hackrf_stop_tx() failed: %s (%d)\n", hackrf_error_name((enum hackrf_error)rv), rv);
                //return EXIT_FAILURE;
            }

            sleep_ms(200);

            std::cout << "loop #: " << idx << std::endl;
        }

    }
    catch (std::exception e)
    {
        std::cout << "error: " << e.what() << std::endl;
    }
    
    rv = hackrf_close(dev); 
    rv = hackrf_exit();
    
    std::cout << std::endl << "Program complete!  Pressenter to close..." << std::endl;
    std::cin.ignore();

}   // end of main
