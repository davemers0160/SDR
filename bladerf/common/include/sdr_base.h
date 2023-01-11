#ifndef _SDR_BASE_CLASS_H_
#define _SDR_BASE_CLASS_H_

#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <condition_variable>


class SDR_BASE
{
public:

    bool collect_samples;
    uint64_t num_samples;

    std::mutex sdr_mtx;
    std::condition_variable sdr_cv;
    bool processed;

    //   static std::unique_ptr<Source> build(const std::string& type, Config& config);
    static std::unique_ptr<SDR_BASE> build();
    
    //virtual void init();

    virtual uint64_t get_rx_samplerate() const = 0;
    virtual uint64_t get_tx_samplerate() const = 0;

    virtual uint64_t get_rx_frequency() const = 0;
    virtual uint64_t get_tx_frequency() const = 0;

    
    // start the SDR collecting samples
    virtual void start(std::vector<std::complex<float>> &cf_samples) = 0;
    
    // collect a single group of samples
    virtual void start_single(std::vector<std::complex<float>>& cf_samples, uint32_t num_samples) = 0;

    // stop the SDR
    virtual void stop() = 0;
    
    virtual void wait_for_samples()
    {
        std::unique_lock<std::mutex> lck(sdr_mtx);
        sdr_cv.wait(lck, [this] { return processed; });
        processed = false;
    }

protected:
    //inline bool get_processed() {return processed;}
};

#endif  // _SDR_BASE_CLASS_H_
