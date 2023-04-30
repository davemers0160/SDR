# Analog Devices ADALM Pluto Example Projects
This part of the repository is designed to show examples of the ADI Pluto and the IIO Library on both Windows and Linux.  Details of the [Pluto](https://wiki.analog.com/university/tools/pluto) can be found at the link.

## Driver Install
The primary instructions for the installation of the IIO Library are located here: [What is libiio?](https://wiki.analog.com/resources/tools-software/linux-software/libiio)

There are two options for the installation.
1. Binary Install
2. Building and Install LIBIIO from source
-- Windows: [Building libiio in Visual Studio](https://wiki.analog.com/resources/tools-software/linux-software/building_libiio_for_windows)
-- Linux: [Building on the Linux Host Target](https://wiki.analog.com/resources/tools-software/linux-software/libiio)

The easiest path is to install from binaries.  This should work for most efforts.  We will discuss any issues or problems with the binary install for each OS.

### Windows
LIBIIO Notes:
- Navigate to the [What is libiio?](https://wiki.analog.com/resources/tools-software/linux-software/libiio) page and look for the Windows link to the binary: https://github.com/analogdevicesinc/libiio/releases
- Select the latest release version (you may need to expand the list).  As of v0.24 the Windows install file is "libiio-0.24.gc4498c2-Windows-setup.exe"
- Run the installer and select your language.  The installer may ask to stop certain USB interfaces.  Go ahead and stop them to install the file.  Once done click "Finish" to complete the process.

PLUTO Driver Notes:
- The USB drivers are located here: [PlutoSDR-M2k-USB-Drivers](https://github.com/analogdevicesinc/plutosdr-m2k-drivers-win/releases).  Select the latest version.
- Run the installer and follow the instructions.

### Linux - Ubuntu 20.04
LIBIIO Notes:
- Navigate to the [What is libiio?](https://wiki.analog.com/resources/tools-software/linux-software/libiio) page and look for your Linux distribution verion: https://github.com/analogdevicesinc/libiio/releases
- Select the latest release version (you may need to expand the list).

PLUTO Driver Notes:
- There are no Linux specific drivers for the USB to Ethernet converter to install.  It looks like ADI takes advantage of drivers that are already built into Linux.

```
sudo apt-get update
sudo apt-get install avahi-daemon avahi-utils
```


## PLUTO checkout

Once the LIBIIO Library and PLUTO Drivers are installed restart the computer.  After the restart plug the PLUTO into your computer and ensure that the SDR shows up as a mass storage device and check to make sure the USB to Ethernet is functioning.  To do this open a terminal window and runn the following command:

```
iio_info -u ip:192.168.2.1
```

You should get an output with no errors.  You may need to update the firmware on the PLUTO.  Follow the instruction that are located on the info.html file located on the PLUTO SDR mass storage device.

## LIBIIO API
The full API is located here: [LIBIIO API](http://analogdevicesinc.github.io/libiio/).  Select the documentation version that is commensurate with the library version.

## Projects

### rx_example
This is a simple example that shows how to do very basic operations using the LIBIIO library.  The code will tune the SDR, set the samplerate and receive gain.  It will record approximately 1 second worth of IQ data.

### tx_example
This is a simple example that shows how to do very basic operations using the LIBIIO library.  The code will tune the SDR, set the samplerate and transmit gain.  It will then send 20 FSK bursts and stop.

