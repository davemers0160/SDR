#ifndef _BLADERF_PARSE_INPUT_H_
#define _BLADERF_PARSE_INPUT_H_

#include <cstdint>
#include <string>
#include <vector>


// bladeRF includes
#include <libbladeRF.h>
#include <bladeRF2.h>


//-----------------------------------------------------------------------------
inline void read_hop_params(std::string param_filename,
    bladerf_frequency& start_freq,
    bladerf_frequency& stop_freq,
    int64_t& hop_step,
    bladerf_sample_rate& sample_rate,
    uint16_t& hop_type,
    double& on_time,
    double& off_time,
    bladerf_gain& tx1_gain,
    std::string& iq_filename
)
{

    try {

        std::ifstream tmp_stream(param_filename);
        std::stringstream buffer;
        buffer << tmp_stream.rdbuf();
        std::string contents = buffer.str();
        tmp_stream.close();

        ryml::Tree config = ryml::parse_in_arena(ryml::to_csubstr(contents));

        // background: depthmap value, probablility, blur radius values
        ryml::NodeRef frequency = config["frequency"];
        frequency["start"] >> start_freq;
        frequency["stop"] >> stop_freq;
        frequency["step"] >> hop_step;

        // hop_type: 0 - linear, 1 - random
        config["hop_type"] >> hop_type;

        // sample_rate
        config["sample_rate"] >> sample_rate;

        // foreground: depthmap value, probablility, blur radius values
        ryml::NodeRef timing = config["timing"];
        timing["on_time"] >> on_time;
        timing["off_time"] >> off_time;

        // tx_gain
        config["tx_gain"] >> tx1_gain;

        // IQ file
        config["IQ_file"] >> iq_filename;

    }
    catch (std::exception& e)
    {
        std::string error_string = "Error parsing input file: " + param_filename + " - " + std::string(e.what()) + "\n";
        error_string += "File: " + std::string(__FILE__) + ", Line #: " + std::to_string(__LINE__);
        throw std::runtime_error(error_string);
    }

}   // end of read_hop_params
#endif	// _BLADERF_PARSE_INPUT_H_
