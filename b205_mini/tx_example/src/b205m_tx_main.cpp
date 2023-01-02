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
const double pi2 = 2.0 * 3.14159265358979323846;
const std::complex<double> j(0, 1);

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
void generate_fsk(double sample_rate, std::vector<std::complex<int16_t>> &samples, double freq_offset = 25000.0, double bit_length = 1e-2, double amplitude = 2000)
{
    uint64_t idx, jdx;

    //generate IQ samples - simple FSK
    uint64_t data = 0xAB42F58C15ACFE37;     // random 32-bit data
    uint64_t bit_mask = 1;
    std::complex<double> tmp_val;

    // clear out the samples
    samples.clear();

    // the number of samples per bit
    uint32_t bit_samples = (uint32_t)(sample_rate * bit_length);

    // the frequency offset for the FSK modulation - 100kHz, normalized by the sample rate
    freq_offset = freq_offset / sample_rate;

    for (idx = 0; idx < sizeof(data) * 8; ++idx)
    {
        // if true this is a one
        if (data & bit_mask)
        {
            for (jdx = 0; jdx < bit_samples; ++jdx)
            {
                tmp_val = amplitude * (std::exp(j * pi2 * freq_offset * (double)jdx));
                samples.push_back(std::complex<int16_t>((int16_t)tmp_val.imag(), (int16_t)tmp_val.real()));
            }
        }
        // if not this is a zero
        else
        {
            for (jdx = 0; jdx < bit_samples; ++jdx)
            {
                tmp_val = amplitude * (std::exp(j * pi2 * (-freq_offset) * (double)jdx));
                samples.push_back(std::complex<int16_t>((int16_t)tmp_val.imag(), (int16_t)tmp_val.real()));
            }
        }

        bit_mask <<= 1;
    }

}   // end of generate_fsk


//-----------------------------------------------------------------------------
void send_data(uhd::tx_streamer::sptr tx_stream, std::vector<std::complex<int16_t>>&samples, size_t samps_per_buff)
{
    uint64_t idx;
    //size_t bytes_to_xfer = samps_per_buff;
    size_t bytes_remaining = samples.size() - data_index;
    size_t num_tx_samps;
    size_t samples_sent;

    uhd::tx_metadata_t md;
    md.start_of_burst = false;
    md.end_of_burst = false;

    std::vector<std::complex<int16_t>> buffer(samps_per_buff);

    // loop until all of the data has been sent
    while (!md.end_of_burst) 
    {
        // check the current index and the transfer size to see where we are at in the samples
        if (bytes_remaining >= samps_per_buff)
        {
            // start at index == 0
            // if the size of the samples buffer is larger than the transfer->buffer length then fill the transfer buffer
            // increment the index by the number of bytes_to_xsfer
            for (idx = 0; idx < samps_per_buff; ++idx)
            {
                buffer[idx] = samples[data_index + idx];
            }

            samples_sent = tx_stream->send(&buffer.front(), samps_per_buff, md);

            data_index += samps_per_buff;
            bytes_remaining = samples.size() - data_index;
        }
        else
        {
            // if the number of the remaining samples is less than the transfer buffer fill the buffer with just those samples
            for (idx = 0; idx < bytes_remaining; ++idx)
            {
                buffer[idx] = samples[data_index + idx];
            }

            // set the transfer valid length to the number of remaining bytes
            samples_sent = tx_stream->send(&buffer.front(), bytes_remaining, md);

            data_index = 0;
            tx_complete = true;
            md.end_of_burst = true;
        }

        //if (samples_sent != num_tx_samps) {
        //    UHD_LOG_ERROR("TX-STREAM",
        //        "The tx_stream timed out sending " << num_tx_samps << " samples ("
        //        << samples_sent << " sent).");
        //    return;
        //}

    }

}   // end of send_data

//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{

    uint32_t idx = 0;
    
    std::string args;

    double sample_rate = 8000000;

    uint32_t channel = 0;
    double freq = 314500000;
    double tx_gain = 20;
    double bw;
    double lo_offset = 0.0;

    // initialize a random number generator
    srand((unsigned)time(NULL));
    double rng_offset = 5e4;
    int32_t range = 20;
    double freq_offset;


    for (idx = 0; idx < 20; ++idx)
    {
        freq_offset = (rng_offset * (rand() % range));
    }

    // determine how many blocks to capture
    uint64_t num_samples = (uint64_t)(1.0 * sample_rate);

    // allocate the memory for the samples, but do not actually init the container
    std::vector<std::complex<int16_t>> samples;
//    std::vector<std::complex<int16_t>> samples;
//    samples.reserve(num_samples);

    generate_fsk(sample_rate, samples, 5000);

    try
    {
        // declare a UHD address type and put the usrp type in it as a string
        uhd::device_addr_t dev_addr("B205mini");

        // create an object for the usrp sdr
        uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(dev_addr);
        //std::cout << "Using Device: " << usrp->get_pp_string() << std::endl << std::endl;

        // set the samplerate
        usrp->set_tx_rate(sample_rate, channel);
        std::cout << "Actual TX Samplerate: " << (usrp->get_tx_rate(channel) / 1e6) << " MHz" << std::endl << std::endl;

        // set the center frequency and LO offset
        uhd::tune_request_t tune_request(freq, lo_offset);
        usrp->set_tx_freq(tune_request, channel);
        std::cout << "Actual TX Freq: " << (usrp->get_tx_freq(channel) / 1e6) << " MHz" << std::endl << std::endl;

        // set the tx gain
        usrp->set_tx_gain(tx_gain, channel);
        std::cout << "Actual TX Gain: " <<  usrp->get_tx_gain(channel) << " dB" << std::endl << std::endl;

        // set the antenna
        std::string ant = usrp->get_tx_antenna(channel);
        std::cout << "Antenna: " << ant << std::endl;
        usrp->set_tx_antenna(ant, channel);

        std::string cpu_format = "sc16";
        std::string wire_format = "sc16";
        uhd::stream_args_t stream_args(cpu_format, wire_format);
        std::vector<size_t> channel_nums;
        channel_nums.push_back(channel);
        stream_args.channels = channel_nums;
        uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

        size_t samps_per_buff = tx_stream->get_max_num_samps();

        std::cout << std::endl << "Press enter to transmit." << std::endl;
        std::cin.ignore();


        freq = 314000000;

        for (idx = 0; idx < 20; ++idx)
        {
            uhd::tune_request_t tune_request(freq + freq_offset, lo_offset);
            usrp->set_tx_freq(tune_request, channel);

            data_index = 0;
            tx_complete = false;

            send_data(tx_stream, samples, samps_per_buff);
            while (!tx_complete)
            {
                sleep_ms(50);
            }

            freq_offset = rng_offset * idx;
            //freq_offset = (rng_offset * (rand() % range));
            sleep_ms(200);

            std::cout << "loop #: " << idx << std::endl;
        }


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
