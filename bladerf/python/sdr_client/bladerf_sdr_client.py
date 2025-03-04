import tomllib
import yaml
import struct

from enum import IntEnum

import numpy as np
import numpy.typing as npt

import zmq

# from modulation_params import MODULATION_TYPES, am_params, fm_params, iq_params, modulation_params
# from signal_builder import signal_builder

#------------------------------------------------------------------------------
class bladerf_sdr_client:

    # ID of the server
    BLADERF_SERVER_ID = 0xB0000000
    
    #------------------------------------------------------------------------------
    # class SB_MESSAGE_ID(IntEnum):
    # standard request for the server version
    GET_VERSION             = (BLADERF_SERVER_ID | 0x00000000)
    SELECT_MODE             = (BLADERF_SERVER_ID | 0x00000001)
    
    CONFIG_RX               = (BLADERF_SERVER_ID | 0x00000100)
    ENABLE_RX               = (BLADERF_SERVER_ID | 0x00000101)
    SET_RX_FREQ             = (BLADERF_SERVER_ID | 0x00000102)
    SET_RX_GAIN             = (BLADERF_SERVER_ID | 0x00000103)
    #SET_RX_SAMPLERATE       = (BLADERF_SERVER_ID | 0x00000104)
    SET_RX_BANDWIDTH        = (BLADERF_SERVER_ID | 0x00000105)

    CONFIG_TX               = (BLADERF_SERVER_ID | 0x00000200)
    ENABLE_TX               = (BLADERF_SERVER_ID | 0x00000201)
    SET_TX_FREQ             = (BLADERF_SERVER_ID | 0x00000202)
    SET_TX_GAIN             = (BLADERF_SERVER_ID | 0x00000203)
    #SET_TX_SAMPLERATE       = (BLADERF_SERVER_ID | 0x00000204)
    SET_TX_BANDWIDTH        = (BLADERF_SERVER_ID | 0x00000205)

    UNKNOWN                 = 0xFFFFFFFF

    #------------------------------------------------------------------------------
    def __init__(self, ip_address: str, port: str):
        # create the connection string for zmq
        connect_string = "tcp://" + ip_address + ":" + port
        
        # create the context and socket
        # TODO: potentially move the socket type into the initializer
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        
        # connect
        self.socket.connect(connect_string)

    #------------------------------------------------------------------------------
    # def import_from_toml(self, filename):
    #
    #     with open(filename, "rb") as f:
    #         data = tomllib.load(f)
    #
    #     # TODO add file check to make sure there is data
    #
    #     return self.parse_input_data(data)
    #
    # #------------------------------------------------------------------------------
    # def import_from_yaml(self, filename):
    #
    #     with open(filename, "rb") as f:
    #         data = yaml.safe_load(f)
    #
    #     # TODO add file check to make sure there is data
    #
    #     return self.parse_input_data(data)

    #------------------------------------------------------------------------------
    def get_version(self):
        # create the command message and convert to bytearray
        command = np.uint32(self.GET_VERSION)
        command_msg = struct.pack("<I", command)

        server_version = np.array([0])
        try:
            # send the command message
            self.socket.send(command_msg)

            response = self.socket.recv()
            response = np.array(struct.unpack("<4I", response)).astype(np.uint32)

            if (response[0] == np.uint32(self.GET_VERSION)):
                server_version = response[1:]

        except Exception as e:
            print(f"An error occurred sending/receiving the request: {e}")

        finally:
            return server_version
          
    #------------------------------------------------------------------------------
    def select_mode(self, mode: np.uint32):
        command = np.array([self.SELECT_MODE, mode]).astype(np.uint32)
        cmd_fs = "<" + str(command.size) + "I"       
        command_msg = struct.pack(cmd_fs, *command)

        result = -1
        try:
            # send the command message
            self.socket.send(command_msg)

            response = self.socket.recv()
            res_fs = "<" + str(len(response)//4) + "I"
            response = np.array(struct.unpack(res_fs, response)).astype(np.uint32)

            if (response[0] == np.uint32(self.SELECT_MODE)):
                result = response[1];

        except Exception as e:
            print(f"An error occurred sending/receiving the request: {e}")

        finally:
            return result       
    
    #------------------------------------------------------------------------------
    def config_tx(self, start_frequency: np.uint64, stop_frequency: np.uint64, frequency_step: np.int32, sample_rate: np.uint32, bw: np.uint32, gain: np.int32):
        
        stf_32M = (start_frequency >> 32).astype(np.uint32)
        stf_32L = (start_frequency & 0x00FFFFFFFF).astype(np.uint32)
        spf_32M = (stop_frequency >> 32).astype(np.uint32)
        spf_32L = (stop_frequency & 0x00FFFFFFFF).astype(np.uint32)        
        
        # create the command message and convert to bytearray
        command = np.array([self.CONFIG_TX, stf_32M, stf_32L, spf_32M, spf_32L, frequency_step, sample_rate, bw, gain]).astype(np.uint32)
        cmd_fs = "<" + str(command.size) + "I"       
        command_msg = struct.pack(cmd_fs, *command)
        
        result = -1
        try:
            # send the command message
            self.socket.send(command_msg)

            response = self.socket.recv()
            res_fs = "<" + str(len(response)//4) + "I"
            response = np.array(struct.unpack(res_fs, response)).astype(np.uint32)

            if (response[0] == np.uint32(self.CONFIG_TX)):
                result = response[1];

        except Exception as e:
            print(f"An error occurred sending/receiving the request: {e}")

        finally:
            return result       

    #------------------------------------------------------------------------------
    def enable_tx(self, status: bool):
        command = np.array([self.ENABLE_TX, int(status)]).astype(np.uint32)
        cmd_fs = "<" + str(command.size) + "I"       
        command_msg = struct.pack(cmd_fs, *command)

        result = -1
        try:
            # send the command message
            self.socket.send(command_msg)

            response = self.socket.recv()
            res_fs = "<" + str(len(response)//4) + "I"
            response = np.array(struct.unpack(res_fs, response)).astype(np.uint32)

            if (response[0] == np.uint32(self.ENABLE_TX)):
                result = response[1];

        except Exception as e:
            print(f"An error occurred sending/receiving the request: {e}")

        finally:
            return result   


    #------------------------------------------------------------------------------
    def set_iq_savepath(self, file_path: str):
        # create the command message and convert to bytearray
        command = np.uint32(self.SET_SAVE_LOCATION)
        command_msg = struct.pack("<I", command)

        # fp_array = bytearray()
        fp_array = file_path.encode("utf-8")

        result = -1
        try:
            # send the command message and the data as a multipart message
            self.socket.send(command_msg, zmq.SNDMORE)
            self.socket.send(fp_array)

            response = self.socket.recv()
            response = np.array(struct.unpack("<2I", response)).astype(np.uint32)

            if (response[0] == command):
                result = response[1].astype(np.int32)

        except Exception as e:
            print(f"An error occurred sending/receiving the request: {e}")

        finally:
            return result


    #------------------------------------------------------------------------------
    def create_signal_builder(self, sb: signal_builder):
        # create the command message and convert to bytearray
        command = np.uint32(self.CREATE_SIGNAL)
        command_msg = struct.pack("<I", command)

        sb_data = bytearray()
        sb_data = sb.serialize(sb_data)

        result = -1
        try:
            # send the command message and the data as a multipart message
            self.socket.send(command_msg, zmq.SNDMORE)
            self.socket.send(sb_data)

            response = self.socket.recv()
            response = np.array(struct.unpack("<2I", response)).astype(np.uint32)

            if (response[0] == command):
                result = response[1].astype(np.int32)

        except Exception as e:
            print(f"An error occurred sending/receiving the request: {e}")

        finally:
            return result

    #------------------------------------------------------------------------------
    def generate_iq_with_data(self, data: npt.NDArray[np.int16], index):
        # create the command message and convert to bytearray
        command = np.array([self.GENERATE_IQ_WITH_DATA, index]).astype(np.uint32)
        command_msg = struct.pack("<2I", *command)

        # start by serializing the number of data messages
        num_messages = data.shape[0]
        data_msg = bytearray()
        tmp = struct.pack(">I", num_messages)
        data_msg.extend(tmp)

        result = -1
        iq_filename = ""

        for idx in range(num_messages):
            # get the number of elements
            data_size = data[idx].size
            tmp = struct.pack(">Q", data_size)
            data_msg.extend(tmp)

            # pack the data into the byte array
            nd_fs = ">" + str(data_size) + "h"
            tmp = struct.pack(nd_fs, *data[idx])
            data_msg.extend(tmp)

        try:
            # send the command message and the data as a multipart message
            self.socket.send(command_msg, zmq.SNDMORE)
            self.socket.send(data_msg)

            response = self.socket.recv()
            res_fs = "<" + str(len(response)//4) + "I"
            response = np.array(struct.unpack(res_fs, response)).astype(np.uint32)

            if (response[0] == np.uint32(self.GENERATE_IQ_WITH_DATA)):
                result = response[1]
                if result == 1:
                    iq_filename = ''.join([chr(x) for x in response[2:]])

        except Exception as e:
            print(f"An error occurred sending/receiving the request: {e}")

        finally:
            return result, iq_filename

    #------------------------------------------------------------------------------
    def close(self):
        self.socket.close()
        self.context.term()

    #------------------------------------------------------------------------------
