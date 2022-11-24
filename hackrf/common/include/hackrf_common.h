#ifndef _HACKRF_COMMON_H_
#define _HACKRF_COMMON_H_

#include <hackrf.h>

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)

#else

#endif

// ----------------------------------------------------------------------------
//inline std::ostream& operator<< (
//    std::ostream& out,
//    const struct bladerf_devinfo &item
//    )
//{
//    out << "BladeRF Device Information: " << std::endl;
//    out << "  backend:       " << std::string(bladerf_backend_str(item.backend)) << std::endl;
//    out << "  serial number: " << std::string(item.serial) << std::endl;
//    out << "  usb_bus:       " << (uint32_t)item.usb_bus << std::endl;
//    out << "  usb_addr:      " << (uint32_t)item.usb_addr << std::endl;
//    out << "  instance:      " << item.instance << std::endl;
//    out << "  manufacturer:  " << std::string(item.manufacturer) << std::endl;
//    out << "  product:       " << std::string(item.product) << std::endl;
//    out << std::endl;
//    return out;
//}

//-----------------------------------------------------------------------------
int32_t select_hackf(hackrf_device** dev)
{
    uint32_t idx;
    std::string console_input;

    int32_t rv;
    int32_t index = 0;

    // get the list of currently attached devices
    hackrf_device_list_t* hackrf_list = hackrf_device_list();

    int32_t num_devices = hackrf_list->devicecount;

    for (idx = 0; idx < num_devices; ++idx)
    {
        std::cout << "HackRF Device [" << idx << "]: " << std::string(hackrf_list->serial_numbers[idx]) << std::endl;
    }

    if (num_devices == 1)
    {
        std::cout << std::endl << "Selecting HackRF[0]" << std::endl;
        rv = hackrf_device_list_open(hackrf_list, 0, dev);
    }
    else if (num_devices > 1)
    {
        std::cout << "Select HackRF device number: ";
        std::getline(std::cin, console_input);

        index = std::stoi(console_input);
        rv = hackrf_device_list_open(hackrf_list, index, dev);
    }
    else
    {
        std::cout << "Could not detect any bladeRF devices.  Check connections and try again..." << std::endl;
        rv = HACKRF_ERROR_OTHER;
    }

    // free the list 
    hackrf_device_list_free(hackrf_list);

    return rv;

}   // end of get_device_list

#endif  // _HACKRF_COMMON_H_
