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
- Select the install version that corresponds with the version of Visual Studio that you have installed.  If the version of Visual Studio installed on your system is greater than the available driver version select the closest version possible.
- During the install you will be asked to add UHD to the system path.  Add it for all users.
- Select the default install location and do a full install

Boost Install
- None of the docs indicate that Boost is required to compile/run anything with the UHD driver.  Download and install from here: https://www.boost.org/users/download/  Select the binaries option and then pick the appropriate boost version for your PC.

Once the software is installed restart the computer.

After the restart plug in the B205mini and check that the drivers installed correctly using the Device Manager.  If you "Westbridge" and a yellow exclamation next to it then you need to install libusb.  Download the latest verson of [Zadig](https://github.com/pbatard/libwdi/releases/) and run it.  Select the WestBridge device from the dropdown menu and then select WinUSB from the list of drivers.  Click the install button and wait for it to finish the install.

### Linux
Notes:


## UHD API
The full API is located here: [UHD Development Manual](https://files.ettus.com/manual/page_uhd.html)

A knowledge base for the B205mini can be found here: https://kb.ettus.com/B200/B210/B200mini/B205mini#B205mini-i

## Projects

### rx_example

### tx_example



