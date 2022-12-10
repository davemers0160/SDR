#ifndef _HACKRF_COMMON_H_
#define _HACKRF_COMMON_H_

#include <cstdint>
#include <string>
#include <iostream>
#include <iomanip>

#include <hackrf.h>

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)

#else

#endif

//----------------------------------------------------------------------------
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

//----------------------------------------------------------------------------
template<typename T>
inline std::string num2str(T val, std::string fmt)
{
    char in_string[64];
    sprintf(in_string, fmt.c_str(), val);
    return std::string(in_string);
}

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
        if(hackrf_list->serial_numbers[idx] != NULL)
            std::cout << "HackRF Device [" << idx << "]: " << std::string(hackrf_list->serial_numbers[idx]) << std::endl;
    }

    if ((num_devices == 1) & (hackrf_list->serial_numbers[0] != NULL))
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

    std::cout << std::endl;

    return rv;

}   // end of get_device_list

//-----------------------------------------------------------------------------
void get_hack_info(hackrf_device* dev)
{
    char version[256];
    uint16_t usb_version;
    read_partid_serialno_t read_partid_serialno;
    uint8_t board_id = 0;

    int32_t rv;

    std::cout << "libhackrf version: " << std::string(hackrf_library_release()) << " " << std::string(hackrf_library_version()) << std::endl << std::endl;

    rv = hackrf_version_string_read(dev, version, 255);
    rv |= hackrf_usb_api_version_read(dev, &usb_version);
    rv |= hackrf_board_id_read(dev, &board_id);
    rv |= hackrf_board_partid_serialno_read(dev, &read_partid_serialno);

    if (rv != HACKRF_SUCCESS)
    {
        std::cout << "error getting HackRF info: " << std::string(hackrf_error_name((enum hackrf_error)rv)) << std::endl;
        std::cout << "Press enter to close" << std::endl;
        std::cin.ignore();
    }
    else
    {
        std::cout << "Firmware Version: " << std::string(version) << " (API: " << num2str((usb_version >> 8) & 0xFF, "%x") << "." << num2str(usb_version & 0xFF, "%02x") << ")" << std::endl;
        std::cout << "HackRF board ID: " << std::string(hackrf_board_id_name((enum hackrf_board_id)board_id)) << std::endl;
        std::cout << "Part ID Number: " << num2str(read_partid_serialno.part_id[0], "0x%08x") << " " << num2str(read_partid_serialno.part_id[1], "0x%08x") << std::endl;
    }
    std::cout << std::endl;

}   // end of get_hack_info

#endif  // _HACKRF_COMMON_H_
