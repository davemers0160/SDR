# BladeRF FM Demodulator Example Project
This project is designed to show how to configure the BladeRF2, continously recieve samples tuned to an FM radio station and then demodulate and send the audio to the default sound device on the PC (audio portion only works in Windows for right now).

## Dependencies

The code in this project has the following dependecies:

1. [CMake 2.8.12+](https://cmake.org/download/ )
2. [davemers0160 common code repository](https://github.com/davemers0160/Common )
3. [BladeRF Driver & API](https://www.nuand.com )
4. [ArrayFire](https://www.arrayfire.com/ )

## Build

The project uses CMake as the primary mechanism to build the executables.  There are some modifications that may have to be made to the CMakeLists.txt file in order to get the project to build successfully.

The first thing that must be done is to create an environment variable called "PLATFORM".  The CMakeLists.txt file uses this variable to determine where to look for the other required repositories and/or libraries.  These will be machine specific.

To create an environment variable in Windows type the following (drop the -m if you do not have elevated privileges):
```
setx -m PLATFORM MY_PC
```

In Linux (usually placed in .profile or .bashrc):
```
export PLATFORM=MY_PC
```

In the CMakeLists.txt file make sure to add a check for the platform you've added and point to the right locations for the repositories/libraries.

Before building the project you must install the BladeRF driver and API.  You must also install the ArrayFire library that is used to perform the FFTs and display the spectrum in a window.  

There is a CMake mechanisim to find the ArrayFire library once it is install, however, depending on your system CMake may not find it.  If this happens, use the CMake GUI to point to the correct install location for ArrayFire.

### Windows

From the directory that contains this file, execute the following commands in a Windows command window:

```
mkdir build
cd build
cmake -G "Visual Studio 15 2017 Win64" -T host=x64 ..
cmake --build . --config Release
```

Or you can use the CMake GUI and set the "source code" location to the location of the CmakeLists.txt file and the set the "build" location to the build folder. Then you can open the project in Visual Studio and compile from there.

### Linux

From the directory that contains this file, execute the following commands in a terminal window:

```
mkdir build
cd build
cmake ..
cmake --build . --config Release -- -j4
```

Or you can use the CMake GUI and set the "source code" location to the location of the CmakeLists.txt file and the set the "build" location to the build folder. Then open a terminal window and navigate to the build folder and execute the following command:

```
cmake --build . --config Release -- -j4
```

The -- -j4 tells the make to use 4 cores to build the code.  This number can be set to as many cores as you have on your PC.

## Running

The code is run by entering the following on the command line:

```
Windows: blade_fm_demod
Linux: ./blade_fm_demod
```

The code will scan the USB 3.0 bus to any available BladeRFs and they will be listed.  Select the index of the desired BladeRF and press Enter to begin capturing data.

To stop the program close the window that is displaying the spectrum.
