#ifndef _BLADERF_PARSE_INPUT_H_
#define _BLADERF_PARSE_INPUT_H_

#include <cstdint>
#include <string>
#include <vector>


// bladeRF includes
#include <libbladeRF.h>
#include <bladeRF2.h>

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
template <typename T>
inline void generate_range(T start, T stop, T step, std::vector<T>& range)
{
    range.clear();

    T s = start;
    if (step > 0)
    {
        while (s <= stop)
        {
            range.push_back(s);
            s += step;
        }
    }
    else if (step < 0)
    {
        while (s >= stop)
        {
            range.push_back(s);
            s += step;
        }
    }
    else
    {
        range.push_back(start);
    }

}   // end of generate_range


//-----------------------------------------------------------------------------
void parse_input(std::string param_filename,
    std::vector<uint64_t>& freq_range,
    uint32_t &sample_rate,
    double &duration,
    std::string &save_location
)
{

    uint32_t idx;

    uint64_t start, stop, step;

    try {
        std::ifstream tmp_stream(param_filename);
        std::stringstream buffer;
        buffer << tmp_stream.rdbuf();
        std::string contents = buffer.str();
        tmp_stream.close();

        //std::cout << contents << std::endl;

        ryml::Tree config = ryml::parse_in_arena(ryml::to_csubstr(contents));

        // frequency step plan
        ryml::NodeRef freq_node = config["frequency"];
        freq_node["start"] >> start;
        freq_node["stop"] >> stop;
        freq_node["step"] >> step;

        generate_range(start, stop, step, freq_range);

        // sample rate
        config["sample_rate"] >> sample_rate;

        // step duration
        config["duration"] >> duration;

        config["save_location"] >> save_location;

    }
    catch (std::exception& e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }
}
#endif	// _BLADERF_PARSE_INPUT_H_
