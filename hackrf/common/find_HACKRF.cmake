message(STATUS "--------------------------------------------------------------------------------")
message(STATUS "Looking for HACKRF Library...")

find_path(HACKRF_INCLUDE_DIRS hackrf.h
    PATHS /usr/local /usr "C:/install/PothosSDR-2021.07.25-vc16-x64/include"  "C:/Program Files/PothosSDR/include" ENV CPATH
    PATH_SUFFIXES include libhackrf
    )

find_library(HACKRF_LIBRARIES hackrf
    HINTS ${HACKRF_INCLUDE_DIRS}
    PATHS /usr/local /usr "C:/install/PothosSDR-2021.07.25-vc16-x64" "C:/Program Files/PothosSDR"
    PATH_SUFFIXES lib amd64 lib64 x64 
    )

mark_as_advanced(HACKRF_INCLUDE_DIRS HACKRF_LIBRARIES)

if (HACKRF_LIBRARIES AND HACKRF_INCLUDE_DIRS)
    set(HACKRF_FOUND TRUE)
    message(STATUS "Found HACKRF Includes: " ${HACKRF_INCLUDE_DIRS})
    message(STATUS "Found HACKRF Library: " ${HACKRF_LIBRARIES})

else()
    message("--- HACKRF library was not found! ---")
    set(HACKRF_FOUND FALSE)
endif()

message(STATUS "--------------------------------------------------------------------------------")
message(STATUS " ")
