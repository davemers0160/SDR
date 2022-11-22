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
    
    
    // initialize the hackrf - required before opening
    auto rv = hackrf_init();
    
    
    hackrf_list = hackrf_device_list();
    
    
    rv = hackrf_device_list_open(hackrf_list, 0, &dev);
    
    // free the list 
    hackrf_device_list_free(hackrf_list);
    
    
    
    rv = hackrf_close(dev);
    
    rv = hackrf_exit();
    
    
}   // end of main
