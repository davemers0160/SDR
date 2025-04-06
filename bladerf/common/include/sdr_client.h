#ifndef SDR_CLIENT_BASE_H_
#define SDR_CLIENT_BASE_H_

#include <cstdint>
#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <complex>
#include <csignal>
#include <chrono>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <filesystem>

// ZMQ includes
#include <zmq.hpp>
#include <zmq_addon.hpp>

#include "bladerf_server_commands.h"


//-----------------------------------------------------------------------------
class sdr_client
{
public:

    sdr_client(){};
    

    //-----------------------------------------------------------------------------
    inline void connect(std::string ip_address, std::string port)
    {
        sdr_context = zmq::context_t{ 1 };
        sdr_socket = create_server_connection(ip_address, port, sdr_context, static_cast<int32_t>(zmq::socket_type::req));
        
        get_version();
    }
    
    //-----------------------------------------------------------------------------
    inline void get_version()
    {
        std::vector<uint32_t> msg(1, static_cast<uint32_t>(BLADE_MSG_ID::GET_VERSION));
        zmq::message_t sdr_server_message;
        
        try
        {       
            // send the request to the sdr server
            sdr_socket.send(zmq::buffer(msg), zmq::send_flags::none);
            
            // wait for the response
            auto result = sdr_socket.recv(sdr_server_message, zmq::recv_flags::none);
            std::vector<uint32_t> sdr_msg(sdr_server_message.size() / sizeof(uint32_t));
            memcpy(sdr_msg.data(), sdr_server_message.data(), sdr_server_message.size());
                    
            std::cout << "Server Version: " << sdr_msg[0] << "." << sdr_msg[1] << "." << sdr_msg[2] << "." << std::endl;        
            
        }
        catch(...)
        {
        
        }
       
    }   // end of get_version

    //-----------------------------------------------------------------------------
    inline void config_tx(uint64_t start_frequency, uint64_t stop_frequency, int32_t frequency_step, uint32_t sample_rate, uint32_t bw, int32_t gain)
    {
        std::vector<uint32_t> msg(1, static_cast<uint32_t>(BLADE_MSG_ID::CONFIG_TX));
        
        msg.push_back(start_frequency >> 32);
        msg.push_back(start_frequency & 0x00FFFFFFFF);
        msg.push_back(stop_frequency >> 32);
        msg.push_back(stop_frequency & 0x00FFFFFFFF);
        msg.push_back(frequency_step);
        msg.push_back(sample_rate);
        msg.push_back(bw);
        msg.push_back(gain);       
        
        try
        {       
            // send the request to the sdr server
            sdr_socket.send(zmq::buffer(msg), zmq::send_flags::none);
            
            // wait for the response
            auto result = sdr_socket.recv(sdr_server_message, zmq::recv_flags::none);
            std::vector<uint32_t> sdr_msg(sdr_server_message.size() / sizeof(uint32_t));
            memcpy(sdr_msg.data(), sdr_server_message.data(), sdr_server_message.size());
                        
            
        }
        catch(...)
        {
        
        }     
        
    }   // end of config_tx
    

//-----------------------------------------------------------------------------
private:

std::string ip_address;
std::string port;

zmq::socket_t sdr_socket;
zmq::context_t sdr_context;


};  // end of sdr_client class


#endif  // SDR_CLIENT_BASE_H_
