#ifndef _BLADERF_COMMON_H_
#define _BLADERF_COMMON_H_

#include <cstdint>
#include <string>

#include <file_parser.h>

// bladeRF includes
#include <libbladeRF.h>
#include <bladeRF2.h>


// ----------------------------------------------------------------------------
inline std::ostream& operator<< (
    std::ostream& out,
    const struct bladerf_devinfo &item
    )
{
    out << "BladeRF Device Information: " << std::endl;
    out << "  backend:       " << std::string(bladerf_backend_str(item.backend)) << std::endl;
    out << "  serial number: " << std::string(item.serial) << std::endl;
    out << "  usb_bus:       " << (uint32_t)item.usb_bus << std::endl;
    out << "  usb_addr:      " << (uint32_t)item.usb_addr << std::endl;
    out << "  instance:      " << item.instance << std::endl;
    out << "  manufacturer:  " << std::string(item.manufacturer) << std::endl;
    out << "  product:       " << std::string(item.product) << std::endl;
    out << std::endl;
    return out;
}


//-----------------------------------------------------------------------------
int select_bladerf(int num_devices, struct bladerf_devinfo* device_list)
{
    uint32_t idx;
    std::string console_input;

    if (num_devices == 1)
    {
        return 0;
    }
    else if (num_devices > 1)
    {

        for (idx = 0; idx < num_devices; ++idx)
        {
            std::cout << "BladeRF Device [" << idx << "]: " << std::string(device_list[idx].serial) << std::endl;
        }
        
        std::cout << "Select BladeRF device number: ";
        std::getline(std::cin, console_input);
        return std::stoi(console_input);
    }
    else
    {
        std::cout << "Could not detect any bladeRF devices.  Check connections and try again..." << std::endl;
    }

    return -1;

}   // end of get_device_list

// ----------------------------------------------------------------------------
void bladerf_status(int status)
{
    if (status != 0)
    {
        std::cout << "Unable to open device: " << std::string(bladerf_strerror(status)) << std::endl;
        exit(status);
    }
    //return status;

}

// ----------------------------------------------------------------------------
void read_bladerf_params(std::string param_filename,
    bladerf_frequency& rx_freq,
    bladerf_sample_rate& fs,
    bladerf_bandwidth& rx_bw,
    bladerf_gain& rx1_gain,
    double* t = nullptr,                /* optional */
    std::string* filename = nullptr     /* optional */
)
{

    uint32_t idx = 0;

    std::vector<std::vector<std::string>> params;
    parse_csv_file(param_filename, params);

    for (idx = 0; idx < params.size(); ++idx)
    {
        switch (idx)
        {
        case 0:
            try
            {
                rx_freq = (bladerf_frequency)std::stoull(params[idx][0]);
            }
            catch (...)
            {
                rx_freq = 137000000;
            }
            break;

        case 1:
            try
            {
                fs = (bladerf_sample_rate)std::stoi(params[idx][0]);
            }
            catch (...)
            {
                fs = 1000000;
            }
            break;

        case 2:
            try
            {
                rx_bw = (bladerf_bandwidth)std::stoi(params[idx][0]);
            }
            catch (...)
            {
                rx_bw = 1000000;
            }
            break;

        case 3:
            try
            {
                rx1_gain = (bladerf_gain)std::stoi(params[idx][0]);
            }
            catch (...)
            {
                rx1_gain = 20;
            }
            break;

        case 4:
            if (t != nullptr)
            {
                try
                {
                    *t = std::stod(params[idx][0]);
                }
                catch (...)
                {
                    *t = 10;
                }
            }
            break;


        case 5:
            if (filename != nullptr)
            {
                *filename = params[idx][0];
            }
            break;
        }

    }

}   // end of read_bladerf_params

#endif  // _BLADERF_COMMON_H_
