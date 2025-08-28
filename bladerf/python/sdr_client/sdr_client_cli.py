import sys
import numpy as np
import cmd
import shlex

from bladerf_sdr_client import bladerf_sdr_client

#------------------------------------------------------------------------------
class sdr_client_cli(cmd.Cmd):
    intro = ("SDR Client CLI. Type help or ? to list commands.\n  "
             "version\n  "
             "select_mode 0-RX, 1-TX\n  "
             "config_tx start_frequency, stop_frequency, frequency_step, sample_rate, bw, gain, scan_time\n  "
             "config_rx start_frequency, stop_frequency, frequency_step, sample_rate, bw, gain\n  "
             "enable_amp 0-OFF, 1-ON\n  "
             "enable_tx 0-OFF, 1-ON\n  "
             "enable_scan 0-OFF, 1-ON\n  "
             "get_iq_files\n  "
             "load_iq_file iq_filename\n  "
             "rx_capture_samples duration_in_seconds\n  "
             "rx_set_freq start_frequency\n  "
             "rx_set_gain gain\n  "
             # "set_savepath '/location/to/save/iq/files/'\n  "
             )

    prompt = "SDR> "

    #------------------------------------------------------------------------------
    ip_address = "localhost"              # local server
    # ip_address = "172.24.36.134"            # WSL server
    port = "25252"                          # server port

    # create the client server
    sdr_client = bladerf_sdr_client(ip_address, port)

    # --- basic commands ---
    def do_version(self, stupid_cmd):
        # get the server version
        server_version = self.sdr_client.get_version()
        print("server version: {}.{}.{}\n".format(server_version[0], server_version[1], server_version[2]))

    def do_select_mode(self, stupid_cmd):
        # switch between TX and RX
        args = shlex.split(stupid_cmd)
        mode = np.uint32(args[0])

        result = self.sdr_client.select_mode(mode)
        print("result: {}\n".format(result))

    def do_config_tx(self, stupid_cmd):
        # config_tx on the server
        # start_frequency: np.uint64, stop_frequency: np.uint64, frequency_step: np.int32,
        # sample_rate: np.uint32, bw: np.uint32, gain: np.int32, scan_time: np.float32
        args = shlex.split(stupid_cmd)

        start_frequency = np.uint64(args[0])
        stop_frequency = np.uint64(args[1])
        frequency_step = np.int32(args[2])
        sample_rate = np.uint32(args[3])
        bw = np.uint32(args[4])
        gain = np.int32(args[5])
        scan_time = np.float32(args[6])

        result = self.sdr_client.config_tx(start_frequency, stop_frequency, frequency_step, sample_rate, bw, gain, scan_time)
        print("Config TX: {}\n".format(result))

    def do_config_rx(self, stupid_cmd):
        # config_rx on the server
        # start_frequency: np.uint64, stop_frequency: np.uint64, frequency_step: np.int32,
        # sample_rate: np.uint32, bw: np.uint32, gain: np.int32
        args = shlex.split(stupid_cmd)

        start_frequency = np.uint64(args[0])
        stop_frequency = np.uint64(args[1])
        frequency_step = np.int32(args[2])
        sample_rate = np.uint32(args[3])
        bw = np.uint32(args[4])
        gain = np.int32(args[5])

        result = self.sdr_client.config_tx(start_frequency, stop_frequency, frequency_step, sample_rate, bw, gain)
        print("Config TX: {}\n".format(result))

    def do_enable_amp(self,stupid_cmd):
        # enable amp
        args = shlex.split(stupid_cmd)
        val = np.uint32(args[0])

        result = self.sdr_client.enable_amp(val)
        print("result: {}\n".format(result))

    def do_enable_tx(self, stupid_cmd):
        # enbale tx
        args = shlex.split(stupid_cmd)
        val = np.uint32(args[0])

        result = self.sdr_client.enable_tx(val)
        print("result: {}\n".format(result))

    def do_enable_scan(self,stupid_cmd):
        # delete all signal builder objects on the server
        args = shlex.split(stupid_cmd)
        val = np.uint32(args[0])

        result = self.sdr_client.enable_scan(val)
        print("result: {}\n".format(result))

    def do_get_iq_files(self, stupid_cmd):
        # get the list of iq files
        result, file_list = self.sdr_client.get_iq_files()
        if (result == 1):
            for item in file_list:
                print("file: {}".format(item))
        else:
            print("No files found! result: {}\n".format(result))

    def do_load_iq_file(self, stupid_cmd):
        # generate IQ data
        args = shlex.split(stupid_cmd)
        iq_filename = args[0]

        result = self.sdr_client.load_iq_file(iq_filename)
        print("result: {}\n".format(result))

    def do_rx_capture_samples(self, stupid_cmd):
        # generate IQ data
        args = shlex.split(stupid_cmd)
        capture_time = np.float32(args[0])

        result, iq_filename = self.sdr_client.rx_capture_samples(capture_time)
        if result == 1:
            print("iq_filename: {}\n".format(iq_filename))
        else:
            print("error capturing data\n")

    def do_rx_set_freq(self, stupid_cmd):
        # generate IQ data
        args = shlex.split(stupid_cmd)
        start_freq = np.uint64(args[0])

        result = self.sdr_client.rx_set_freq(start_freq)
        print("result: {}\n".format(result))



    def do_rx_set_gain(self, stupid_cmd):
        # generate IQ data
        args = shlex.split(stupid_cmd)
        gain = np.int32(args[0])

        result, iq_filename = self.sdr_client.rx_set_gain(gain)
        print("result: {}\n".format(result))

    def do_set_savepath(self, stupid_cmd):
        args = shlex.split(stupid_cmd)
        savepath = args[0]

        # result = self.sdr_client.set_iq_savepath(savepath)
        # print("result: {}\n".format(result))

    def do_eng(self, stupid_cmd):
        bp = 1
        args = shlex.split(stupid_cmd)
        index = 0
        data = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]).astype(np.int16)
        frames = np.array([0])

        result = 1
        print("result: {}\n".format(result))

    def do_sb_close(self, stupid_cmd):
        # close the client - closes the socket and the context
        self.sdr_client.close()

#------------------------------------------------------------------------------
if __name__ == '__main__':
    sdr_client_cli().cmdloop()

    # break point :-)
    bp = 2


# result = sb_client.set_iq_savepath("d:/Projects/data/RF/")
# print("result: {}\n".format(result))



# # create a set of random data to generate IQ data - 20 bits per
# msg_data = np.random.randint(2, size=[sb[0].num_params, 20])
#
# # generate IQ data using the externally supplied data
# result, iq_filename = sb_client.generate_iq_with_data(msg_data, index)
# print("result: {}\n".format(result))
# print("iq_filename: {}\n".format(iq_filename))

# generate N IQ data bursts
# result, iq_filename = sb_client.generate_n_iq(index, 10)
# print("result: {}\n".format(result))
# print("iq_filename: {}\n".format(iq_filename))




