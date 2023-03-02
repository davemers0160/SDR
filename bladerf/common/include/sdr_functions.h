#ifndef _DSP_FUNCTIONS_H_
#define _DSP_FUNCTIONS_H_

#include <cstdint>
#include <string>
#include <vector>
#include <complex>
#include <deque>


// ----------------------------------------------------------------------------
template<typename T>
inline std::vector<T> maximal_length_sequence(uint16_t N, std::vector<uint16_t> taps = { 0, (uint16_t)(N - 1) })
{
    uint64_t idx, jdx;
    uint16_t tmp;
    std::vector<T> sr;

    //std::vector<uint16_t> taps = { 0, (uint16_t)(N - 1) };

    // initialize the register
    std::deque<uint8_t> r(N, 0);
    r[0] = 1;

    // shift register 
    uint64_t sr_size = (1 << N) - 1;

    for (idx = 0; idx < sr_size; ++idx)
    {
//        sr.insert(sr.end(), rep, amplitude * (2 * r[N - 1] - 1));
        sr.insert(sr.end(), r[N - 1]);

        tmp = 0;
        for (jdx = 0; jdx < taps.size(); ++jdx)
        {
            tmp += r[taps[jdx]];
        }
        tmp = tmp % 2;

        r.push_front((uint8_t)tmp);
        r.pop_back();
    }

    return sr;
}   // end of maximal_length_sequence

// ----------------------------------------------------------------------------
template<typename U, typename T>
inline std::vector<std::complex<U>> generate_bpsk_iq(std::vector<T> s, double amplitude = 1.0)
{
    uint64_t idx;

    std::vector<std::complex<U>> data(s.size(), std::complex<U>(0, 0));

    for (idx = 0; idx < s.size(); ++idx)
    {
        data[idx] = std::complex<U>(amplitude * (2 * s[idx] - 1), 0);
    }

    return data;
}
//-----------------------------------------------------------------------------
std::vector<std::complex<double>> create_freq_rotation(uint64_t N, double fr)
{
    uint64_t idx;
    std::vector<std::complex<double>> res(N, 0.0);

    for (idx = 0; idx < N; ++idx)
    {
        res[idx] = std::exp(1i * 2.0 * M_PI * fr * (double)idx);
    }

    return res;
}


#endif  //_DSP_FUNCTIONS_H_
