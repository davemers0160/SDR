# common CMake file for the paths for all of the projects

if(WIN32)

	if(${PLATFORM} STREQUAL "Laptop")
		set(COMMON_ROOT "D:/Common")
	elseif(${PLATFORM} STREQUAL "LaptopN")
		set(COMMON_ROOT "C:/Projects/Common")
    elseif(${PLATFORM} STREQUAL "BEAST")
        set(COMMON_ROOT "D:/Common")
    elseif(${PLATFORM} STREQUAL "Laptop_Beast")
        set(COMMON_ROOT "D:/Projects/Common")
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