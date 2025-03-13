
import os
import sys

# Importing flask module in the project is mandatory
# An object of Flask class is our WSGI application.
from flask import Flask
from flask import render_template, url_for, request, redirect

import numpy as np
import numpy.typing as npt

import zmq

script_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(script_path + "/sdr_client/")
from bladerf_sdr_client import bladerf_sdr_client

# Flask constructor takes the name of
# current module (__name__) as argument.
app = Flask(__name__)

sdr_client = []

#------------------------------------------------------------------------------
# scan directory for file
def scan_directory_for_filetype(directory_path, file_extension):
    matching_files = []

    with os.scandir(directory_path) as entries:
        for entry in entries:
            if entry.is_file() and entry.name.lower().endswith(file_extension.lower()):
                matching_files.append(entry.path)

    return matching_files


# The route() function of the Flask class is a decorator,
# which tells the application which URL should call
# the associated function.
@app.route('/test')
# ‘/’ URL is bound with hello_world() function.
def hello_world():
    return 'Hello World'

@app.route('/test2', methods=['GET', 'POST'])
def index():
    iq_filename = None
    tx_enable_state = False

    file_list = scan_directory_for_filetype("D:/data/RF", "sc16")

    if request.method == 'POST':

        if request.form["submit_button"] == "on":
            # print("yes")
            tx_enable_state = False
        elif request.form["submit_button"] == "off":
            # print("no")
            tx_enable_state = True

        # Process the text data here
        # if request.form["submit_button"] == "b1":
        #     text_data = request.form['text_input']
        # elif request.form["submit_button"] == "b2":
        #     # text_data = request.form['text_input2']
        elif request.form["submit_button"] == "Connect":
            sdr_server_ip = request.form["sdr_server_ip"]
            sdr_server_port = request.form["sdr_server_port"]

            # create the client server
            sdr_client = bladerf_sdr_client(sdr_server_ip, sdr_server_port)

        elif request.form["submit_button"] == "Config Tx":
            sr_str = request.form["sample_rate"]
            start_freq_str = request.form["start_freq"]
            stop_freq_str = request.form["stop_freq"]
            freq_step_str = request.form["freq_step"]
            gain_str = request.form["gain"]
            bw_str = request.form["bandwidth"]
            iq_filename = request.form['file_dropdown']

            try:
                sample_rate = np.uint32(int(sr_str))
                start_freq = np.uint64(int(start_freq_str))
                stop_freq = np.uint64(int(stop_freq_str))
                freq_step = np.uint32(int(freq_step_str))
                gain = np.uint32(int(gain_str))
                bandwidth = np.uint32(int(bw_str))

                result = sdr_client.config_tx(start_freq, stop_freq, freq_step, sample_rate, bandwidth, gain)
                print("result: {}\n".format(result))

            except:
                sample_rate = np.uint32(20000000)
                start_freq = np.uint64(1600000000)
                stop_freq = np.uint64(160000000)
                freq_step = np.uint32(2000000)
                gain = np.uint32(65)
                bandwidth = np.uint32(5000000)


        # return f"You entered: {text_data}"

    # return render_template('index.html')  # Ensure you have an index.html file
    return render_template('index.html', options=file_list, tx_enable_state=tx_enable_state) #, selected_value=text_data)


# main driver function
if __name__ == '__main__':

    # run() method of Flask class runs the application
    # on the local development server.
    app.run(debug=True)

