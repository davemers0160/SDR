
import os

# Importing flask module in the project is mandatory
# An object of Flask class is our WSGI application.
from flask import Flask
from flask import render_template, url_for, request, redirect

# Flask constructor takes the name of
# current module (__name__) as argument.
app = Flask(__name__)


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

        # Process the text data here
        if request.form["submit_button"] == "b1":
            text_data = request.form['text_input']
        elif request.form["submit_button"] == "b2":
            # text_data = request.form['text_input2']

            if request.form['value'] == "Enable TX":
                # print("yes")
                tx_enable_state = True
            elif request.form['value'] == "Disable TX":
                # print("no")
                tx_enable_state = False

        elif request.form["submit_button"] == "Config Tx":
            sr_str = request.form["sample_rate"]
            start_freq_str = request.form["start_freq"]
            stop_freq_str = request.form["stop_freq"]
            freq_step_str = request.form["freq_step"]
            gain_str = request.form["gain"]
            bw_str = request.form["bandwidth"]
            iq_filename = request.form['file_dropdown']

            try:
                sample_rate = int(sr_str)
                start_freq = int(start_freq_str)
                stop_freq = int(stop_freq_str)
                freq_step = int(freq_step_str)
                gain = int(gain_str)
                bandwidth = int(bw_str)

            except:
                sample_rate = 20000000
                start_freq = 1600000000
                stop_freq = 160000000
                freq_step = 2000000
                gain = 65
                bandwidth = 5000000


        # return f"You entered: {text_data}"

    # return render_template('index.html')  # Ensure you have an index.html file
    return render_template('index.html', options=file_list, tx_enable_state=tx_enable_state) #, selected_value=text_data)


# main driver function
if __name__ == '__main__':

    # run() method of Flask class runs the application
    # on the local development server.
    app.run(debug=True)

