# ----------------------------------------------------------------------------
#  Root CMake file for the USRP B205mini recieve example
# ----------------------------------------------------------------------------
#
# To perform an out of source build do the following on the command line:
# 1. navigate to the root project directory
# 2. mkdir build
# 3. cd build
# 4. depending on your system (windows/linux) type the following
# Example CMake build line for windows & VS2015
# windows: cmake -G "Visual Studio 14 2015 Win64" -DUSE_AVX_INSTRUCTIONS=ON ..
# linux: cmake -DUSE_AVX_INSTRUCTIONS=ON ..
#
# 5. To compile:
# windows: cmake --build . --config Release
# linux: cmake --build . --config Release -- -j4
#


cmake_minimum_required(VERSION 2.8.12)

PROJECT(b205mini_rx_example)
set(exe_name "b205m_rx")

set(PLATFORM "$ENV{PLATFORM}" CACHE INTERNAL "Copied from environment variable")
if("${PLATFORM}" STREQUAL "")
    set(PLATFORM "NOT_SET")
    message("The PLATFORM Environment variable is not set on this system.  Setting it to:")
endif()
message("Platform: " ${PLATFORM})

# find the root of the main project
string(FIND ${PROJECT_SOURCE_DIR} "/" INDEX REVERSE)
string(SUBSTRING ${PROJECT_SOURCE_DIR} 0 ${INDEX} PR_ROOT)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(WIN32)

	if(${PLATFORM} STREQUAL "Laptop")
		set(COMMON_ROOT "D:/Common")
	elseif(${PLATFORM} STREQUAL "LaptopN")
		set(COMMON_ROOT "C:/Projects/Common")
    elseif(${PLATFORM} STREQUAL "BEAST")
        set(COMMON_ROOT "D:/Common")
    else()
        set(COMMON_ROOT "C:/Projects/Common")
    endif()    
    
else()

    if(${PLATFORM} STREQUAL "MainGear")
        set(_ECLIPSE_VERSION "3.8")
        set(COMMON_ROOT "~/Projects/Common")
    elseif(${PLATFORM} STREQUAL "HPC")
        set(COMMON_ROOT "~/Projects/Common")
    else()
        set(COMMON_ROOT "~/Projects/Common")
    endif()
    
endif()

# look for any *.c and *.cpp files
file(GLOB SOURCES "src/*.c*"
    "${COMMON_ROOT}/src/*.cpp"
    )

set(project_includes 
    "include" 
    "${PR_ROOT}/common/include"
    "${COMMON_ROOT}/include"
    )

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PR_ROOT}/common/")

# find the UHD API & driver, minimum is 4.0.0
find_package(UHD 4.0.0 REQUIRED)
if(UHD_FOUND)
	set(project_includes ${project_includes} ${UHD_INCLUDE_DIRS})
	set(project_libs ${project_libs} ${UHD_LIBRARIES})
else()
    message(" ")
    message("Couldn't find UHD Drivers...")
endif()

# The USRP requires Boost.
# Set components here, then include UHDBoost to do the actual finding
set(UHD_BOOST_REQUIRED_COMPONENTS
    program_options
    system
    thread
)
set(BOOST_MIN_VERSION 1.65)
find_package(Boost ${BOOST_MIN_VERSION} COMPONENTS ${UHD_BOOST_REQUIRED_COMPONENTS} REQUIRED
    HINTS "C:/local/boost_1_81_0/lib64-msvc-14.2/cmake/Boost-1.81.0")

#include(UHDBoost "c:/local/boost_1_81_0/")

set(project_includes ${project_includes} ${Boost_INCLUDE_DIRS})
set(project_libs ${project_libs} ${Boost_LIBRARIES})

link_directories(${Boost_LIBRARY_DIRS})

message(" ")
message(STATUS "--------------------------------------------------------")
message(STATUS "Source Files: ")
foreach(S ${SOURCES})
    message(STATUS "  " ${S})
endforeach()
message(STATUS "--------------------------------------------------------")

message(" ")

message(STATUS "--------------------------------------------------------")
message(STATUS "Project Includes: ")
foreach(S ${project_includes})
    message(STATUS "  " ${S})
endforeach()
message(STATUS "--------------------------------------------------------")

message(" ")

message(STATUS "--------------------------------------------------------")
message(STATUS "Project Libs: ")
foreach(S ${project_libs})
    message(STATUS "  " ${S})
endforeach()
message(STATUS "--------------------------------------------------------")
message(" ")


include_directories(${project_includes})
add_executable(${exe_name} ${SOURCES})
target_link_libraries(${exe_name} ${project_libs})

if(WIN32)
    message(STATUS "MSVC Version: " ${MSVC_VERSION})
    # uncomment for MSVC and cmake greater than 3.6.0
    set(default_build_type "Release")
    if(MSVC AND (${CMAKE_VERSION} VERSION_GREATER "3.6.0"))
        target_compile_options(${exe_name} PRIVATE "/MP")
        set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${exe_name})
    endif()

endif()

message(" ")
message(STATUS "Build Complete!")
message(" ")
