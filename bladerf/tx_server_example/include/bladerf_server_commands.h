#ifndef _BLADERF_SERVER_COMMANDS_H_
#define _BLADERF_SERVER_COMMANDS_H_

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)

#else

#endif

#include <cstdint>
#include <string>

// bladeRF includes
//#include <libbladeRF.h>
//#include <bladeRF2.h>

// Server Control Messages
#define BLADERF_SERVER_ID 0xB0000000

const std::string BLADERF_IP_ADDRESS = "*";
const std::string BLADERF_PORT = "25252";

enum class BLADE_MSG_ID : uint32_t
{
    SET_RX_FREQ             = (BLADERF_SERVER_ID | 0x00000101),
    SET_RX_GAIN             = (BLADERF_SERVER_ID | 0x00000102),
    SET_RX_SAMPLERATE       = (BLADERF_SERVER_ID | 0x00000103),
    SET_RX_BANDWIDTH        = (BLADERF_SERVER_ID | 0x00000104),
    CONFIG_RX               = (BLADERF_SERVER_ID | 0x00000105),
    RECEIVE_SAMPLES         = (BLADERF_SERVER_ID | 0x00000106),
    
    SET_TX_FREQ             = (BLADERF_SERVER_ID | 0x00000201),
    SET_TX_GAIN             = (BLADERF_SERVER_ID | 0x00000202),
    SET_TX_SAMPLERATE       = (BLADERF_SERVER_ID | 0x00000203),
    SET_TX_BANDWIDTH        = (BLADERF_SERVER_ID | 0x00000204),
    CONFIG_TX               = (BLADERF_SERVER_ID | 0x00000205),
    TRANSMIT_SAMPLES        = (BLADERF_SERVER_ID | 0x00000206),
    
    UNKNOWN                 = 0xFFFFFFFF
};

//-----------------------------------------------------------------------------
inline zmq::socket_t create_server_connection(std::string ip_address, std::string port, zmq::context_t& context, int32_t type = static_cast<int32_t>(zmq::socket_type::rep))
{
    std::string binding_str = "tcp://" + ip_address + ":" + port;

    // construct a REP (reply) socket and bind to interface
    zmq::socket_t socket{ context, type };
    socket.bind(binding_str);

    int32_t linger = 0; // Set to 0 for immediate close, or a positive value for a timeout
    int32_t result = zmq_setsockopt(socket, ZMQ_LINGER, &linger, sizeof(linger));

    return socket;
}  // end of create_server_connection

#endif  // _BLADERF_SERVER_COMMANDS_H_
