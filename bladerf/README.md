# BladeRF example repository
This repository is intended to show how to configure the Nuand BladeRF to recieve samples and to transmit sample IQ data.  The code is a simple example of how to interface to the BladeRF API in both the Windows and Linux OS's without the use of GNU Radio.

## Dependencies

The code in this repository has the following dependecies

1. [CMake 3.10+](https://cmake.org/download/ )
2. [davemers0160 common code repository](https://github.com/davemers0160/Common )
3. [rapidyaml](https://github.com/davemers0160/rapidyaml )
4. [BladeRF Driver & API](https://www.nuand.com )
5. [ArrayFire](https://www.arrayfire.com/ )

## BladeRF Install

### Windows
The windows install of the BladeRF is very simple.  Follow the instructions [here](https://www.nuand.com/win_installers/) to install the pre-compiled binaries.  Select the version that you want an install.  This will install the bladeRF drivers, the bladeRF-cli (command line interface) and the development headers.

### Linux
The Linux install is done in one of two ways: 1. Using the standard repository method, 2. Build from scratch (recommended for Linux).  The instructions for both can be found here: [https://github.com/Nuand/bladeRF/wiki/Getting-Started:-Linux](https://github.com/Nuand/bladeRF/wiki/Getting-Started:-Linux ).  The general instructions are summarized below:  

```
sudo add-apt-repository ppa:nuandllc/bladerf
sudo apt-get update
sudo apt-get install bladerf libbladerf-dev
```

The repository method works but in general the repos are not as up-to-date as the development releases are published.  Insome cases I've found that the repository version is over 5 years old.  For this reason it is recommended to build from scratch.  Make sure the the user who installs and uses the bladeRF is in the plugdev group so that they have access to the bladeRF.

```
git clone https://github.com/Nuand/bladeRF.git ./bladeRF
cd ./bladeRF/host
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DINSTALL_UDEV_RULES=ON ../
make -j 4 && sudo make install && sudo ldconfig
```

This will build the bladeRF library and install it in the standard Linux locations.

## Repository Projects

### rx_example
This folder contains the project code that illustrates the example code to configure the BladeRF to recieve samples and display them using the ArrayFire library. 

### rx_fm_demod
This folder contains the project code that illustrates the example code to configure the BladeRF, recieve samples and then perform the necessary digital signal processing (DSP) techniques needed to frequency shift, filter, downsample, and demodulate an FM radio station mono audio.  

### rx_record
This folder contains the project code that illustrates the example code to configure the BladeRF to recieve samples and then same them as a binary file for later use in C++, python, MATLAB, etc...

### rx_sweep
This folder contains the code to record RF in a large sweep.  Set the start and stop frequencies and the step size (within the max samplerate of the BladeRF).  The code will calculate the correct tuning frequencies and record the specified number of seconds for each tuned frequency.

### tx_example
This folder contains the project code that illustrates the example code to configure the BladeRF to transmit an FSK waveform, LFM waveform, BPSK waveform or an IQ file in the SC16 (signed complex 16-bit format).

### apt_demod
This folder contains the code to demod the NOAA 15, 18 & 19 weather satellite APT transmissions and display as an image.  

### tx_filter_example
This folderr contains an example of how to filter a signal using a FIR filter and then transmit the filtered version of the signal.

### tx_hop_example
This folder contains the code for using the quick tune finctionality of the BladeRF2.0 to take a simple waveform and hop it across a wide frequency range (wider than the max allowed samplerate).

### tx_rx_example
This folder contains the code to use threading to receive an RF signal while transmitting a burst of RF.

### common
This folder contains the project code that is shared between all of the projects.