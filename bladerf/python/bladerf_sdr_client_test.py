import sys
import numpy as np

from signal_builder_client import signal_builder_client

# config file for a signal_builder object
filename = "../common/test_config.toml"

#------------------------------------------------------------------------------
ip_address = "localhost"              # local server
# ip_address = "172.24.36.134"            # WSL server
port = "54321"                          # server port
sb = []                                 # container to hold the signal_builder objects

# create the client server
sb_client = signal_builder_client(ip_address, port)

# get the server version
server_version = sb_client.get_server_version()
print("server version: {}.{}.{}\n".format(server_version[0], server_version[1], server_version[2]))

# create a signal_builder object from a toml file and put it into a list
sb_tmp = sb_client.import_from_toml(filename)
sb.append(sb_tmp)

# send the signal builder object to the server
index = sb_client.create_signal_builder(sb[0])
print("index: {}\n".format(index))

result = sb_client.set_iq_savepath("d:/Projects/data/RF/")
print("result: {}\n".format(result))

# generate IQ data
result, iq_filename = sb_client.generate_iq(index)
print("result: {}\n".format(result))
print("iq_filename: {}\n".format(iq_filename))

# create a set of random data to generate IQ data - 20 bits per
msg_data = np.random.randint(2, size=[sb[0].num_params, 20])

# generate IQ data using the externally supplied data
result, iq_filename = sb_client.generate_iq_with_data(msg_data, index)
print("result: {}\n".format(result))
print("iq_filename: {}\n".format(iq_filename))

# generate N IQ data bursts
result, iq_filename = sb_client.generate_n_iq(index, 10)
print("result: {}\n".format(result))
print("iq_filename: {}\n".format(iq_filename))

# close the client - closes the socket and the context
sb_client.close()

# break point :-)
bp = 2
