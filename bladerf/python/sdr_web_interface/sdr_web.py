
import os
import sys
import threading
import random
import time

# Importing flask module in the project is mandatory
# An object of Flask class is our WSGI application.
from flask import Flask
from flask import render_template, url_for, request, redirect
from turbo_flask import Turbo

import numpy as np
import numpy.typing as npt

import zmq

script_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(script_path + "/sdr_client/")
from bladerf_sdr_client import bladerf_sdr_client

# Flask constructor takes the name of
# current module (__name__) as argument.
app = Flask(__name__)
turbo = Turbo(app)


global sdr_client, sdr_publisher, message_label, file_list, sdr_server_ip, sdr_server_port, tx_enable_state, amp_enable_state

#------------------------------------------------------------------------------
# scan directory for file
def scan_directory_for_filetype(directory_path, file_extension):
    matching_files = []

    with os.scandir(directory_path) as entries:
        for entry in entries:
            if entry.is_file() and entry.name.lower().endswith(file_extension.lower()):
                matching_files.append(entry.path)

    return matching_files

#------------------------------------------------------------------------------
# The route() function of the Flask class is a decorator,
# which tells the application which URL should call
# the associated function.
@app.route('/test')
# ‘/’ URL is bound with hello_world() function.
def hello_world():
    return 'Hello World'

#------------------------------------------------------------------------------
@app.route('/test2', methods=['GET', 'POST'])
def index():
    global sdr_client, message_label, file_list, sdr_server_ip, sdr_server_port, tx_enable_state, amp_enable_state, \
        sr_str, start_freq_str, stop_freq_str, freq_step_str, gain_str, bw_str, sdr_publisher, scan_enable_state, \
        scan_str

    iq_filename = None
    # tx_enable_state = False
    # message_label = "Init"
    # amp_enable_state = False

    # file_list = scan_directory_for_filetype("D:/Projects/data/RF", "sc16")

    if request.method == 'POST':

        # ------------------------------------------------------------------------------
        # TX enable button
        item_list = request.form.getlist('tx_button')
        if len(item_list) > 0:

            if request.form["tx_button"] == "on":
                # print("yes")
                try:
                    tx_enable_state = False

                    result = sdr_client.enable_tx(tx_enable_state)
                    message_label = message_label + "Transmit Status: {}\n".format(result)
                    print("result: {}\n".format(result))
                except NameError:
                    tx_enable_state = False
                    message_label = message_label + "Failed to set transmit state\n"

            elif request.form["tx_button"] == "off":
                # print("no")
                try:
                    tx_enable_state = True

                    result = sdr_client.enable_tx(tx_enable_state)
                    message_label = message_label + "Transmit Status: {}\n".format(result)
                    print("result: {}\n".format(result))
                except NameError:
                    tx_enable_state = False
                    message_label = message_label + "Failed to set transmit state\n"


        # ------------------------------------------------------------------------------
        # RF enable button
        item_list = request.form.getlist("rf_button")
        if len(item_list) > 0:

            if request.form["rf_button"] == "amp_on":
                try:
                    amp_enable_state = False

                    result = sdr_client.enable_amp(amp_enable_state)
                    message_label = message_label + "RF Section: {}\n".format(result)
                    print("result: {}\n".format(result))

                except NameError:
                    amp_enable_state = False
                    message_label = message_label + "Failed to set RF state\n"

            elif request.form["rf_button"] == "amp_off":
                try:
                    amp_enable_state = True

                    result = sdr_client.enable_amp(amp_enable_state)
                    message_label = message_label + "RF Section: {}\n".format(result)
                    print("result: {}\n".format(result))

                except NameError:
                    amp_enable_state = False
                    message_label = message_label + "Failed to set RF state\n"

        # ------------------------------------------------------------------------------
        # Scan enable button
        item_list = request.form.getlist("scan_button")
        if len(item_list) > 0:

            if request.form["scan_button"] == "scan_on":
                try:
                    scan_enable_state = False

                    result = sdr_client.enable_scan(scan_enable_state)
                    message_label = message_label + "Scan: {}\n".format(result)
                    print("result: {}\n".format(result))

                except NameError:
                    scan_enable_state = False
                    message_label = message_label + "Failed to set Scan state\n"

            elif request.form["scan_button"] == "scan_off":
                try:
                    scan_enable_state = True

                    result = sdr_client.enable_scan(scan_enable_state)
                    message_label = message_label + "Scan: {}\n".format(result)
                    print("result: {}\n".format(result))

                except NameError:
                    scan_enable_state = False
                    message_label = message_label + "Failed to set Scan state\n"

        #------------------------------------------------------------------------------
        # Submit button
        item_list = request.form.getlist("submit_button")
        if len(item_list) > 0:
            # Connect to the SDR server button
            if request.form["submit_button"] == "Connect":
                tmp_msg = ""
                file_list = []
                try:
                    sdr_server_ip = request.form["sdr_server_ip"]
                    sdr_server_port = request.form["sdr_server_port"]
                    sdr_publisher_port = "25254"
                    tmp_msg = "Connect\n"

                    # create the client server
                    sdr_client = bladerf_sdr_client(sdr_server_ip, sdr_server_port)
                    sdr_publisher = sdr_client.context.socket(zmq.SUB)
                    sdr_publisher.connect("tcp://" + sdr_server_ip + ":" + sdr_publisher_port)
                    sdr_publisher.setsockopt_string(zmq.SUBSCRIBE, "")

                    # get the version
                    server_version = sdr_client.get_version()
                    tmp_msg += "server version: {}.{}.{}\n".format(server_version[0], server_version[1], server_version[2])

                    # get the listing for the available IQ files
                    result, file_list = sdr_client.get_iq_files()

                except NameError:
                    tmp_msg = "SDR client not initialized, connect to server first\n"

                message_label = message_label + tmp_msg

            #------------------------------------------------------------------------------
            # configure TX button
            elif request.form["submit_button"] == "Config Tx":
                sr_str = request.form["sample_rate"]
                start_freq_str = request.form["start_freq"]
                stop_freq_str = request.form["stop_freq"]
                freq_step_str = request.form["freq_step"]
                gain_str = request.form["gain"]
                bw_str = request.form["bandwidth"]
                scan_str = request.form["scan_time"]

                try:
                    sample_rate = np.uint32(float(sr_str)*1e6)
                    start_freq = np.uint64(float(start_freq_str)*1e6)
                    stop_freq = np.uint64(float(stop_freq_str)*1e6)
                    freq_step = np.uint32(float(freq_step_str)*1e6)
                    bandwidth = np.uint32(float(bw_str)*1e6)
                    gain = np.uint32(int(gain_str))
                    scan_time = np.float32(float(scan_str))

                    result = sdr_client.config_tx(start_freq, stop_freq, freq_step, sample_rate, bandwidth, gain, scan_time)
                    print("result: {}\n".format(result))
                    message_label = message_label + "Config Tx: {}\n".format(result)

                except:
                    sample_rate = np.uint32(40000000)
                    start_freq = np.uint64(2000000000)
                    stop_freq = np.uint64(200000000)
                    freq_step = np.uint32(2000000)
                    gain = np.uint32(66)
                    bandwidth = np.uint32(2000000)
                    message_label = message_label + "Check the entires, something isn't right\n"

            #------------------------------------------------------------------------------
            # update the IQ file
            elif request.form["submit_button"] == "IQ File":
                tmp_msg = ""

                try:
                    iq_filename = request.form['file_dropdown']
                    result = sdr_client.load_iq_file(iq_filename)
                    tmp_msg = "Loading IQ File: {}\n".format(iq_filename)
                    tmp_msg += "Result: {}\n".format(result)
                    print("result: {}\n".format(result))

                except NameError:
                    tmp_msg = "SDR client not initialized, connect to server first\n"

                except Exception as e:
                    # print(f"An error occurred: {e}")
                    tmp_msg = "{}\n".format(e)

                message_label = message_label + tmp_msg

    # elif request.method == 'GET':
    #     bp = 3

    # return render_template('index.html')  # Ensure you have an index.html file
    return render_template('index.html', sdr_server_ip=sdr_server_ip, sdr_server_port=sdr_server_port,
                           options=file_list, tx_enable_state=tx_enable_state, sr_str=sr_str, start_freq_str=start_freq_str,
                           stop_freq_str=stop_freq_str, freq_step_str=freq_step_str, gain_str=gain_str, bw_str=bw_str,
                           scan_str=scan_str,
                           amp_enable_state=amp_enable_state, message_label=message_label, scan_enable_state=scan_enable_state) #, selected_value=text_data)


# @app.context_processor
# def inject_load():
#     global sdr_publisher, server_status
#     time.sleep(1)
#     try:
#         server_status = sdr_publisher.recv_string()
#     except NameError:
#         server_status = ""
#
#     return {'server_status': server_status }

@app.before_request
def before_first_request():
    app.before_request_funcs[None].remove(before_first_request)
    threading.Thread(target=update_status).start()

def update_status():
    global sdr_publisher, server_status, message_label
    with app.app_context():
        while True:
            tmp_status = ""
            try:
                time.sleep(1)
                server_status = sdr_publisher.recv_string()
                res = turbo.push(turbo.replace(render_template('server_status.html', server_status=server_status, message_label=message_label),"status"))

            except NameError:
                tmp_status = ""

            # finally:
            #     turbo.push(turbo.replace(render_template('server_status.html', server_status=tmp_status), 'status'))

#------------------------------------------------------------------------------
# main driver function
if __name__ == '__main__':
    # global message_label, file_list, sdr_server_ip, sdr_server_port

    message_label = "Init\n"
    file_list = []
    sdr_server_ip = "10.223.0.100"
    sdr_server_port = "25252"
    tx_enable_state = False
    amp_enable_state = False
    scan_enable_state = False
    sr_str = "20"
    start_freq_str = "2000"
    stop_freq_str = "2000"
    freq_step_str = "2"
    bw_str = "20"
    gain_str = "66"
    scan_str = "4.00"
    server_status = ""
    # th = threading.Thread(target=update_status)
    # th.daemon = True
    # th.start()

    # run() method of Flask class runs the application
    # on the local development server.
    app.run(debug=True, host='0.0.0.0', port=5000)

