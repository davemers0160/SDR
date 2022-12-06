message(STATUS "--------------------------------------------------------------------------------")
message(STATUS "Looking for HACKRF Library...")

find_path(HACKRF_INCLUDE_DIRS hackrf.h
    PATHS /usr/local /usr "C:/install/PothosSDR-2021.07.25-vc16-x64/include"  "C:/Program Files/PothosSDR/include" ENV CPATH
    PATH_SUFFIXES include libhackrf
    )

find_library(HACKRF_LIBRARIES hackrf
    HINTS ${HACKRF_INCLUDE_DIRS}
    PATHS /usr /usr/lib "C:/install/PothosSDR-2021.07.25-vc16-x64" "C:/Program Files/PothosSDR"
    PATH_SUFFIXES lib amd64 lib64 x64 "x86_64-linux-gnu"
    )

mark_as_advanced(HACKRF_INCLUDE_DIRS HACKRF_LIBRARIES)

if (HACKRF_LIBRARIES AND HACKRF_INCLUDE_DIRS)
    set(HACKRF_FOUND TRUE)
    message(STATUS "Found HACKRF Includes: " ${HACKRF_INCLUDE_DIRS})
    message(STATUS "Found HACKRF Library: " ${HACKRF_LIBRARIES})

else()
    message("--- HACKRF library was not found!")
    message("--- HACKRF Includes: " ${HACKRF_INCLUDE_DIRS})
    message("--- HACKRF Library: " ${HACKRF_LIBRARIES})
    if(UNIX)
    	message("--- If this is a new install of the hackrf a complete symbolic link wasn't generated")
    	message("--- find the location of libhackrf. (hint: /usr/lib/x86_64-linux-gnu/) and run the following:")
    	message("--- sudo ln -s /usr/lib/x86_64-linux-gnu/libhackrf.so.0.5.0 /usr/lib/x86_64-linux-gnu/libhackrf.so")
    	message("--- make sure the developer package is installed: sudo apt install libhackrf-dev")
    endif()
    set(HACKRF_FOUND FALSE)
endif()

message(STATUS "--------------------------------------------------------------------------------")
message(STATUS " ")
