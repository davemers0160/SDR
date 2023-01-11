# BladeRF example repository
This repository is intended to show how to configure the Nuand BladeRF to recieve samples and to transmit sample IQ data.  The code is a simple example of how to interface to the BladeRF API in both the Windows and Linux OS's without the use of GNU Radio.

## Dependencies

The code in this repository has the following dependecies

1. [CMake 2.8.12+](https://cmake.org/download/ )
2. [davemers0160 common code repository](https://github.com/davemers0160/Common )
3. [BladeRF Driver & API](https://www.nuand.com )
4. [ArrayFire](https://www.arrayfire.com/ )

## Repository Breakdown

### rx_example
This folder contains the project code that illustrates the example code to configure the BladeRF to recieve samples and display them using the ArrayFire library. 

### rx_fm_demod
This folder contains the project code that illustrates the example code to configure the BladeRF, recieve samples and then perform the necessary digital signal processing (DSP) techniques needed to frequency shift, filter, downsample, and demodulate an FM radio station mono audio.  

### rx_record
This folder contains the project code that illustrates the example code to configure the BladeRF to recieve samples and then same them as a binary file for later use in C++, python, MATLAB, etc...

### tx_example
This folder contains the project code that illustrates the example code to configure the BladeRF to transmit FSK samples.

### common
This folder contains the project code that is shared between all of the projects.
