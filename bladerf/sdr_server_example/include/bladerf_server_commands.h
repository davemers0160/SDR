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

const uint32_t MAJOR_REVISION = 1;
const uint32_t MINOR_REVISION = 0;
const uint32_t FIX_REVISION = 0;

// RX/TX config parameter indecies
const uint16_t CONFIG_START_FREQ_MSB_INDEX = 1;
const uint16_t CONFIG_START_FREQ_LSB_INDEX = 2;
const uint16_t CONFIG_STOP_FREQ_MSB_INDEX = 3;
const uint16_t CONFIG_STOP_FREQ_LSB_INDEX = 4;
const uint16_t CONFIG_FREQ_STEP_INDEX = 5;
const uint16_t CONFIG_SAMPLERATE_INDEX = 6;
const uint16_t CONFIG_BANDWIDTH_INDEX = 7;
const uint16_t CONFIG_GAIN_INDEX = 8;

enum class BLADE_MSG_ID : uint32_t
{
    // General Commands
    GET_VERSION             = (BLADERF_SERVER_ID | 0x00000000),
    SELECT_MODE             = (BLADERF_SERVER_ID | 0x00000001),     /* state (uint32) 0 - RX, 1 - TX */

    // RX Commands
    CONFIG_RX               = (BLADERF_SERVER_ID | 0x00000100),     /* Start Freq (2-uint32 MSB), Stop Freq (2-uint32 MSB), Freq Step (uint32), samplerate (uint32), bw (uint32), gain (int32) */
    ENABLE_RX               = (BLADERF_SERVER_ID | 0x00000101),     /* state (uint32) 0 - off, 1 - on */
    SET_RX_FREQ             = (BLADERF_SERVER_ID | 0x00000102),     /*  */
    SET_RX_GAIN             = (BLADERF_SERVER_ID | 0x00000103),
    //SET_RX_SAMPLERATE       = (BLADERF_SERVER_ID | 0x00000104),
    SET_RX_BANDWIDTH        = (BLADERF_SERVER_ID | 0x00000105),
    
    // TX Commands
    CONFIG_TX               = (BLADERF_SERVER_ID | 0x00000200),     /* Start Freq (2-uint32 MSB), Stop Freq (2-uint32 MSB), samplerate (uint32), bw (uint32), gain (int32) */
    ENABLE_TX               = (BLADERF_SERVER_ID | 0x00000201),     /* state (uint32) 0 - off, 1 - on */
    SET_TX_FREQ             = (BLADERF_SERVER_ID | 0x00000202),
    SET_TX_GAIN             = (BLADERF_SERVER_ID | 0x00000203),
    //SET_TX_SAMPLERATE       = (BLADERF_SERVER_ID | 0x00000204),
    SET_TX_BANDWIDTH        = (BLADERF_SERVER_ID | 0x00000205),
    ENABLE_AMP				= (BLADERF_SERVER_ID | 0x00000206),

    GET_IQ_FILES            = (BLADERF_SERVER_ID | 0x00000300),
    LOAD_IQ_FILE            = (BLADERF_SERVER_ID | 0x00000301),
    
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
