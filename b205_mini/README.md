# B205mini Example Projects
This part of the repository is designed to show examples of the USRP B205mini and the UHD driver on both Windows and Linux.  Details of the [B205mini](https://www.ettus.com/all-products/usrp-b205mini-i/) can be found at the link.

## Driver Install
The primary instructions for the installation of the UHD drivers are located here: [USRP Hardware Driver and USRP Manual](https://files.ettus.com/manual/index.html)

There are two options for the installation.
1. Binary Install
2. Building and Installing UHD from source

The easiest path is to install from binaries.  This should work for most efforts.  We will discuss any issues or problems with the binary install for each OS.

### Windows
Notes:
- Select the install version that corresponds with the version of Visual Studio that you have installed.  If the version of Visual Studio installed on your system is greater than the available driver version select the closest UHD version possible.
- During the install you will be asked to add UHD to the system path.  Add it for all users.
- Select the default install location and do a full install
- The standard install will only include the release versions of the UHD library.  This is important because debugging code in certains instances will cause the program to crash because of the debug/release mis-match.  To get the debug version of the UHD library you will need to build the library.

Boost Install
- None of the docs indicate that Boost is required to compile/run anything with the UHD driver.  Download and install from here: https://www.boost.org/users/download/  Select the binaries option and then pick the appropriate boost version for your PC.  Be sure to select the boost version according to your Visual Studio version.

https://www.boost.org/doc/libs/1_81_0/more/getting_started/unix-variants.html

Once the software is installed restart the computer.

After the restart plug in the B205mini and check that the drivers installed correctly using the Device Manager.  If you see "Westbridge" and a yellow exclamation next to it then you need to install libusb.  Download the latest verson of [Zadig](https://github.com/pbatard/libwdi/releases/) and run it.  Select the WestBridge device from the dropdown menu and then select WinUSB from the list of drivers.  Click the install button and wait for it to finish the install.

### Linux - Ubuntu 20.04
Notes:
There are two different ways to install the UHD support on Ubuntu.  the first is through the default package manager configuration.  This will work, but the libraries and drivers may not be the most up-to-date from Ettus.  The second way is to add the Ettus PPA.  This will allow you to get updates as Ettus makes them.  Run the following commands:

```
sudo add-apt-repository ppa:ettusresearch/uhd
sudo apt-get update
sudo apt-get install -y libuhd-dev uhd-host
```

Boost Install
For Ubuntu 20.04 the package manager will install Boost version 1.71.0.  Use the following command to install using the package manager:

```
sudo apt-get install -y libboost-all-dev
```

## UHD API
The full API is located here: [UHD Development Manual](https://files.ettus.com/manual/page_uhd.html)

A knowledge base for the B205mini can be found here: https://kb.ettus.com/B200/B210/B200mini/B205mini#B205mini-i

## Projects

### rx_example
This is a simple example that shows how to do very basic operations using the UHD library.  The code will tune the SDR, set the samplerate and receive gain.  It will record approximately 1 seconds worth of data.

### tx_example
This is a simple example that shows how to do very basic operations using the UHD library.  The code will tune the SDR, set the samplerate and transmit gain.  It will then send 20 FSK bursts and stop.

