import sys
import numpy as np

from signal_builder_client import signal_builder_client

# config file for a signal_builder object
filename = "../common/test_config.toml"

#------------------------------------------------------------------------------
ip_address = "localhost"              # local server
# ip_address = "172.24.36.134"            # WSL server
port = "25252"                          # server port


# create the client server
sdr_client = bladerf_sdr_client(ip_address, port)

# get the server version
server_version = sdr_client.get_version()
print("server version: {}.{}.{}\n".format(server_version[0], server_version[1], server_version[2]))


start_frequency = 2300000000
stop_frequency = 2900000000
frequency_step = 20000000
sample_rate = 40000000
bw = 5000000
gain = 60


result = sdr_client.config_tx(start_frequency, stop_frequency, frequency_step, sample_rate, bw, gain)
print("result: {}\n".format(result))

result = sdr_client.enable_amp(True)
print("result: {}\n".format(result))

result = sdr_client.enable_amp(False)
print("result: {}\n".format(result))

result = sdr_client.enable_amp(True)
print("result: {}\n".format(result))

result = sdr_client.enable_tx(True)
print("result: {}\n".format(result))

result = sdr_client.enable_tx(False)
print("result: {}\n".format(result))


# close the client - closes the socket and the context
sdr_client.close()

# break point :-)
bp = 2
