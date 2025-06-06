# ----------------------------------------------------------------------------
#  Root CMake file for the bladeRF transmit example
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


cmake_minimum_required(VERSION 3.9)

PROJECT(bladerf_tx_example)
set(exe_name "blade_tx")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PLATFORM "$ENV{PLATFORM}" CACHE INTERNAL "Copied from environment variable")
if("${PLATFORM}" STREQUAL "")
    set(PLATFORM "NOT_SET")
    message("The PLATFORM Environment variable is not set on this system.  Setting it to:")
endif()
message("Platform: " ${PLATFORM})

# find the root of the main project
string(FIND ${PROJECT_SOURCE_DIR} "/" INDEX REVERSE)
string(SUBSTRING ${PROJECT_SOURCE_DIR} 0 ${INDEX} PR_ROOT)

if(WIN32)

	if(${PLATFORM} STREQUAL "Laptop")
		set(COMMON_ROOT "D:/Common")
        set(RYML_ROOT "D:/Projects/rapidyaml")
	elseif(${PLATFORM} STREQUAL "LaptopN")
		set(COMMON_ROOT "C:/Projects/Common")
        set(RYML_ROOT "C:/Projects/rapidyaml")
    elseif(${PLATFORM} STREQUAL "BEAST")
        set(COMMON_ROOT "D:/Common")
        set(RYML_ROOT "D:/Projects/rapidyaml")
    elseif(${PLATFORM} STREQUAL "Laptop_Beast")
        set(COMMON_ROOT "D:/Projects/Common")
        set(RYML_ROOT "D:/Projects/rapidyaml")
    else()
        set(COMMON_ROOT "C:/Projects/Common")
        set(RYML_ROOT "C:/Projects/rapidyaml")
    endif()    
        
    set(RYML_LIB "${RYML_ROOT}/build/Release/ryml.lib")

else()

    if(${PLATFORM} STREQUAL "MainGear")
        set(_ECLIPSE_VERSION "3.8")
        set(COMMON_ROOT "~/Projects/Common")
        set(RYML_ROOT "~/Projects/rapidyaml")
    elseif(${PLATFORM} STREQUAL "HPC")
        set(COMMON_ROOT "~/Projects/Common")
        set(RYML_ROOT "~/Projects/rapidyaml")
    else()
        set(COMMON_ROOT "~/Projects/Common")
        set(RYML_ROOT "~/Projects/rapidyaml")
    endif()
    
    set(RYML_LIB "/usr/local/lib/libryml.so")

    set(CMAKE_CXX_FLAGS "-O3")
    
endif()

#include(${COMMON_ROOT}/cmake_utils/use_cpp_11.cmake)
include(${COMMON_ROOT}/cmake_utils/use_sse_avx.cmake)

# look for any *.c and *.cpp files
file(GLOB SOURCES "src/*.c*"
    "${COMMON_ROOT}/src/*.cpp"
    )

set(project_includes 
    "include" 
    "${PR_ROOT}/common/include"
    "${COMMON_ROOT}/include"
    "${RYML_ROOT}/include"
    )

set(project_libs ${project_libs} 
    ${RYML_LIB}
    )
    
#------------------------------------------------------------------------------
message(" ")
include(${COMMON_ROOT}/cmake_utils/find_bladerf.cmake)
if(BLADERF_FOUND)
    set(project_includes ${project_includes} ${BLADERF_INCLUDE_DIRS})
    set(project_libs ${project_libs} ${BLADERF_LIBS})
endif()

#------------------------------------------------------------------------------
message(" ")

message(STATUS "--------------------------------------------------------------------------------")
message(STATUS "Source Files: ")
foreach(S ${SOURCES})
    message(STATUS "  " ${S})
endforeach()
message(STATUS "--------------------------------------------------------------------------------")

message(" ")

message(STATUS "--------------------------------------------------------------------------------")
message(STATUS "Project Includes: ")
foreach(S ${project_includes})
    message(STATUS "  " ${S})
endforeach()
message(STATUS "--------------------------------------------------------------------------------")

message(" ")

message(STATUS "--------------------------------------------------------------------------------")
message(STATUS "Project Libraries: ")
foreach(S ${project_libs})
    message(STATUS "  " ${S})
endforeach()
message(STATUS "--------------------------------------------------------------------------------")

message(" ")

#------------------------------------------------------------------------------
include_directories(${project_includes})
add_executable(${exe_name} ${SOURCES})
target_link_libraries(${exe_name} ${project_libs})

add_definitions(-DUSE_RAPIDYAML)

if(WIN32)
    message("MSVC Version: " ${MSVC_VERSION})
    # uncomment for MSVC and cmake greater than 3.6.0
    set(default_build_type "Release")
    if(MSVC AND (${CMAKE_VERSION} VERSION_GREATER "3.6.0"))
        target_compile_options(${exe_name} PRIVATE "/MP")
        set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${exe_name})
    endif()

endif()

#------------------------------------------------------------------------------
message(" ")
message(STATUS "Build Complete!")
message(STATUS "cmake --build . --config Release -- -j4")
message(" ")
