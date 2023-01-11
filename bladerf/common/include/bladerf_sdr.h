#ifndef _BLADERF_SDR_H_
#define _BLADERF_SDR_H_

#include <cstdint>
#include <thread>
#include <mutex>
#include <condition_variable>

// bladeRF includes
#include <libbladeRF.h>
#include <bladeRF2.h>

#include "sdr_base.h"

class BLADERF_SDR : virtual public SDR_BASE
{

public:
    
    //-----------------------------------------------------------------------------
    static std::unique_ptr<BLADERF_SDR> open()
    {
        struct bladerf* dev_ = nullptr;
        struct bladerf_devinfo *device_list = NULL;
        struct bladerf_devinfo dev_info;

        int num_devices = bladerf_get_device_list(&device_list);

        auto bladerf_num = select_bladerf(num_devices, device_list);

        if (bladerf_num < 0)
        {
            std::cerr << "Could not detect any bladeRF devices..." << std::endl;
            exit(1);
        }
      
        auto blade_status = bladerf_open(&dev_, ("*:serial=" +  std::string(device_list[bladerf_num].serial)).c_str());
        if (blade_status != 0)
        {
            std::cout << "Unable to open device: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            exit(1);
        }

        blade_status = bladerf_get_devinfo(dev_, &dev_info);
        if (blade_status != 0)
        {
            std::cout << "Unable to get the device info: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            exit(1);
        }
        //std::cout << std::endl << dev_info << std::endl;

        return std::make_unique<BLADERF_SDR>(dev_);
    }   // end of open
    
    
    //-----------------------------------------------------------------------------
    BLADERF_SDR(struct bladerf* dev_) : dev(dev_) 
    {
        // Load list of supported sample rates
        //sampleRates_ = loadSampleRates();

        // We produce floats so let the airspy library take care of it
        //auto rv = airspy_set_sample_type(dev_, AIRSPY_SAMPLE_FLOAT32_IQ);
        //if (blade_status != 0)
        //{
        //    std::cout << "Unable to get the device info: " << std::string(bladerf_strerror(blade_status)) << std::endl;
        //    exit(1);
        //}
        
        // set the rx channel as RX1
        rx = BLADERF_CHANNEL_RX(0);

        num_buffers = 16;
        buffer_size = 1024 * 4 * 8;        // must be a multiple of 1024
        num_transfers = 8;
        timeout_ms = 10000;

        num_samples = 4096 * 32;            // optimally a multiple of 4096 for USB3
               
    }   // end of BLADERF_SDR

    //-----------------------------------------------------------------------------
    ~BLADERF_SDR() 
    {
        if (dev != nullptr)
        {
            auto blade_status = bladerf_enable_module(dev, BLADERF_RX, false);
            blade_status = bladerf_enable_module(dev, BLADERF_TX, false);
            bladerf_close(dev);
        }
    }   // end of ~
    
    //-----------------------------------------------------------------------------
    void set_rx_frequency(uint64_t rx_freq)
    {
        auto blade_status = bladerf_set_frequency(dev, rx, rx_freq);
        blade_status |= bladerf_get_frequency(dev, rx, &rx_freq);
        
        if (blade_status != 0)
        {
            std::cout << "Unable to set rx frequency: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            exit(1);
        }
    }   // end of set_rx_frequency
    
    //-----------------------------------------------------------------------------
    uint64_t get_rx_frequency() const
    {
        uint64_t rx_freq = 0;
        auto blade_status = bladerf_get_frequency(dev, rx, &rx_freq);

        return rx_freq;
    }   // end of get_rx_frequency

    //-----------------------------------------------------------------------------
    void set_tx_frequency(uint64_t tx_freq)
    {
        auto blade_status = bladerf_set_frequency(dev, tx, tx_freq);
        blade_status |= bladerf_get_frequency(dev, tx, &tx_freq);
        
        if (blade_status != 0)
        {
            std::cout << "Unable to set tx frequency: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            exit(1);
        }        
    }   // end of set_tx_frequency
    
    //-----------------------------------------------------------------------------
    uint64_t get_tx_frequency() const
    {
        uint64_t tx_freq = 0;
        auto blade_status = bladerf_get_frequency(dev, tx, &tx_freq);

        return tx_freq;
    }   // end of get_tx_frequency

    //-----------------------------------------------------------------------------
    void set_rx_samplerate(uint64_t fs)
    {
        auto blade_status = bladerf_set_sample_rate(dev, rx, fs, &rx_fs);

        if (blade_status != 0)
        {
            std::cout << "Unable to set rx samplerate: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            exit(1);
        } 
    }   // end of set_rx_samplerate
    
    //-----------------------------------------------------------------------------
    uint64_t get_rx_samplerate() const
    {
        return rx_fs;
    }   // end of get_rx_samplerate
    
    
    //-----------------------------------------------------------------------------
    void set_tx_samplerate(uint64_t fs)
    {
        auto blade_status = bladerf_set_sample_rate(dev, tx, fs, &tx_fs);

        if (blade_status != 0)
        {
            std::cout << "Unable to set tx samplerate: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            exit(1);
        } 
    }   // end of set_tx_samplerate
    
    //-----------------------------------------------------------------------------
    uint64_t get_tx_samplerate() const
    {
        return tx_fs;
    }   // end of get_tx_samplerate
    
    //-----------------------------------------------------------------------------
    void init_rx()
    {
        // configure the sync to receive/transmit data
        auto blade_status = bladerf_sync_config(dev, BLADERF_RX_X1, BLADERF_FORMAT_SC16_Q11, num_buffers, buffer_size, num_transfers, timeout_ms);
        if (blade_status != 0)
        {
            std::cout << "Unable to set sync bladerf: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            exit(1);
        } 
        
        // enable the rx channel RF frontend
        blade_status = bladerf_enable_module(dev, BLADERF_RX, true);
        if (blade_status != 0)
        {
            std::cout << "Unable to enable bladerf: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            exit(1);
        } 
    }   // end of init_rx
    
    //-----------------------------------------------------------------------------
    void init_tx()
    {
        // configure the sync to receive/transmit data
        auto blade_status = bladerf_sync_config(dev, BLADERF_TX_X1, BLADERF_FORMAT_SC16_Q11, num_buffers, buffer_size, num_transfers, timeout_ms);
        if (blade_status != 0)
        {
            std::cout << "Unable to set sync bladerf: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            exit(1);
        } 
        
        // enable the rx channel RF frontend
        blade_status = bladerf_enable_module(dev, BLADERF_TX, true);
        if (blade_status != 0)
        {
            std::cout << "Unable to enable bladerf: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            exit(1);
        } 
    }   // end of init_tx
    
    //-----------------------------------------------------------------------------
    void set_rx_gain(bladerf_gain rx_gain, bladerf_gain_mode gain_mode)
    {
        // the gain must be set after the module has been enabled
        auto blade_status = bladerf_set_gain_mode(dev, rx, gain_mode);
        blade_status |= bladerf_set_gain(dev, rx, rx_gain);
        blade_status |= bladerf_get_gain(dev, rx, &rx_gain);
        
        if (blade_status != 0)
        {
            std::cout << "Unable to set rx gain: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            exit(1);
        }         
    }   // end of set_rx_gain
    
    //-----------------------------------------------------------------------------
    void set_tx_gain(bladerf_gain tx_gain, bladerf_gain_mode gain_mode)
    {
        // the gain must be set after the module has been enabled
        auto blade_status = bladerf_set_gain_mode(dev, tx, gain_mode);
        blade_status |= bladerf_set_gain(dev, tx, rx_gain);
        blade_status |= bladerf_get_gain(dev, tx, &rx_gain);
        
        if (blade_status != 0)
        {
            std::cout << "Unable to set rx gain: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            exit(1);
        }
    }   // end of set_tx_gain

    //-----------------------------------------------------------------------------
    void set_rx_bandwidth(bladerf_bandwidth bw_)
    {
        auto blade_status = bladerf_set_bandwidth(dev, rx, bw_, &rx_bw);

        if (blade_status != 0)
        {
            std::cout << "Unable to set rx bandwidth: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            exit(1);
        }
    }

    //-----------------------------------------------------------------------------
    void set_tx_bandwidth(bladerf_bandwidth bw_)
    {
        auto blade_status = bladerf_set_bandwidth(dev, tx, bw_, &tx_bw);

        if (blade_status != 0)
        {
            std::cout << "Unable to set rx bandwidth: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            exit(1);
        }
    }

    //-----------------------------------------------------------------------------
    void get_continuous_samples(std::vector<std::complex<float>> &cf_samples)
    {
        uint32_t idx;
        std::vector<std::complex<int16_t>> samples(num_samples, 0);
        std::complex<float> cf_scale(1.0 / 2048.0, 0.0f);
        std::vector<std::complex<float>> cs(num_samples, 0);
        int blade_status;
        try
        {
            while (collect_samples)
            {
                blade_status = bladerf_sync_rx(dev, (void*)samples.data(), num_samples, NULL, timeout_ms);

                //std::lock_guard<std::mutex> lg(sdr_mtx);
                //sdr_mtx.lock();
                std::unique_lock<std::mutex> lck(sdr_mtx);

                // convert from complex int16 to complex floats, scale and frequency shift
                for (idx = 0; idx < num_samples; ++idx)
                {                   
                    cf_samples[idx] = std::complex<float>(std::real(samples[idx]), std::imag(samples[idx])) * cf_scale;
                }
                
                processed = true;
                lck.unlock();
                sdr_cv.notify_all();
            }
        }
        catch (...)
        {
            std::cout << "error in collection thread" << std::endl;
        }
    }   // end of data_collection

    //-----------------------------------------------------------------------------
    void start(std::vector<std::complex<float>>& cf_samples)
    {
        collect_samples = true;
        processed = false;
        if(dev == nullptr)
        {
            exit(1);
        }
        
        cf_samples.clear();
        cf_samples.resize(num_samples);
        
        sdr_thread = std::thread(&BLADERF_SDR::get_continuous_samples, this, std::ref(cf_samples));

    }   // end of start

    //-----------------------------------------------------------------------------
    void start_single(std::vector<std::complex<float>>& cf_samples, uint32_t num_samples)
    {
        uint32_t idx;
        std::vector<std::complex<int16_t>> samples(num_samples, 0);
        std::complex<float> cf_scale(1.0 / 2048.0, 0.0f);

        cf_samples.clear();
        cf_samples.resize(num_samples);

        auto blade_status = bladerf_sync_rx(dev, (void*)samples.data(), num_samples, NULL, timeout_ms);

        // convert from complex int16 to complex floats, scale and frequency shift
        for (idx = 0; idx < num_samples; ++idx)
        {
            cf_samples[idx] = std::complex<float>(std::real(samples[idx]), std::imag(samples[idx])) * cf_scale;
        }
    }   // end of start_single
        
    //-----------------------------------------------------------------------------
    void stop()
    {
        collect_samples = false;
        sdr_thread.join();

    }   // end of stop

    //void wait_for_samples()
    //{
    //    std::unique_lock<std::mutex> lck(sdr_mtx);
    //    sdr_cv.wait(lck);
    //}

protected:

    struct bladerf* dev;
    bladerf_channel rx = BLADERF_CHANNEL_RX(0);
    bladerf_channel tx = BLADERF_CHANNEL_TX(0);
    bladerf_frequency rx_freq = 137000000; //162425000;
    bladerf_sample_rate rx_fs = 624000;
    bladerf_sample_rate tx_fs = 624000;
    bladerf_bandwidth rx_bw = 624000;
    bladerf_bandwidth tx_bw = 624000;
    bladerf_gain rx_gain = 64;
    bladerf_gain tx_gain = 10;
    
    uint32_t timeout_ms;
    uint32_t num_buffers;
    uint32_t buffer_size;        // must be a multiple of 1024
    uint32_t num_transfers;
    
    std::thread sdr_thread;

    //-----------------------------------------------------------------------------
    static inline int select_bladerf(int num_devices, struct bladerf_devinfo* device_list)
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

    }   // end of select_bladerf


};

#endif  // _BLADERF_SDR_H_
