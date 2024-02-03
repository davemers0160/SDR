# BladeRF custom project generation

These instructions are for the bladeRF-micro (bladeRF 2.0).

Clone the bladeRF repo.

## Building libbladerf 
You only need to build a portion of the libbladeRF library. But in order to get CMake to generate a Visual Studio projec file you will need several libraries that are common in Linux but not so much in Windows.

You will need the following package manager (VCPKG)[https://github.com/microsoft/vcpkg.git ].  You can clone this repo or just download a zip file and follow the Quick Start: Windows instructions.

You will need the following packages:

```
vcpkg install pthreads --triplet=x64-windows
vcpkg install libusb --triplet=x64-windows
```

Run the CMake GUI and point CMake to the correct library and header locations.

## install Quartus Prime
As of Janury 2024 the recommended version is 20.1, but I've run into substantial issues with the WSL requirement and the correct paths being setup and pointing to the Quartus install.  Quartus 17.1 does not rely on WSL and works without issue.


## Adding your own platform revision (take from the bladeRF wiki and modified)
Working on bladerf-hosted.vhd is not a good idea if you plan to implement your own signal processing for the bladeRF, as you might want to go back to the original functionality without hassle. For this reason, the bladeRF codebase uses Quartus revisions to allow different FPGA design implementations to coexist.

To add your own revision to the bladeRF project, perform the following steps:

-- In hdl/fpga/platforms/bladerf-micro/vhdl/, create a new vhdl file for your top level architecture. It is possible to copy bladerf-hosted.vhd to bladerf-revision_name.vhd as a start.
-- In hdl/fpga/platforms/bladerf-micro/, create a new QIP file containing the necessary IP files, including your top level design file. Again, you can copy bladerf-hosted.qip to bladerf-revision_name.qip and just change the top level VHDL design file name to fit.
-- In hdl/fpga/platforms/bladerf-micro/build/bladerf.tcl, where all the revisions are created, add your revision by adding a line such as "make_revision revision_name" to make your revision known to the Quartus project.
-- In hdl/fpga/platforms/bladerf-micro/build/platform.conf, you need to add your revision to the list of allowed revisions. This can be accomplished by finding the "PLATFORM_REVISIONS" variable and adding your revision as an option there. Don't forget to change the help message in usage() to list your revision as well.

There is one last change that needs to be made:

```
hdl/fpga/ip/analogdevicesinc/no_OS/CMakeLists.txt
```

Change:
```
COMMAND ${Patcher_EXECUTABLE} -p3
to:
COMMAND ${Patcher_EXECUTABLE} -p3 --binary -l
```


At this point, you should be able to build your revision using the build_bladerf.sh script, passing it your revision name as a parameter:

```
./build_bladerf.sh -b bladeRF-micro -r headless -s A9 -a ../../../fpga/platforms/bladerf-micro/signaltap/tx.stp -f
```
/cygdrive/d/Projects/bladeRF/hdl/
