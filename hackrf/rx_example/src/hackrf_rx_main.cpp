// ----------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)

#elif defined(__linux__)

#endif

#include <cstdint>
#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>

// bladeRF includes
#include <hackrf.h>

// Custom Includes
#include <hackrf_common.h>

// Project Includes



//-----------------------------------------------------------------------------



//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{

    uint32_t idx;
    
    // hackrf specific structs
    hackrf_device_list_t *hackrf_list;
    hackrf_transfer hackrf_rx;
    hackrf_device* dev = NULL;

    double sample_rate = 10000000;
    uint64_t freq = 315000000;
    
    int32_t rv;

    uint8_t board_id = 0;

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
        rv = hackrf_set_freq(dev, freq);



    }
    catch (std::exception e)
    {
        std::cout << "error: " << e.what() << std::endl;
    }
    // initialize the hackrf - required before opening
    
    rv = hackrf_close(dev); 
    rv = hackrf_exit();
    
}   // end of main
