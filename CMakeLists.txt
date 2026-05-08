cmake_minimum_required(VERSION 2.6)
project(ft_libbarrett)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Optional build type
# if(NOT CMAKE_BUILD_TYPE)
#     set(CMAKE_BUILD_TYPE RelWithDebInfo)
# endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -------------------------
# Project headers
# -------------------------
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# -------------------------
# tinyxml2
# -------------------------
find_package(tinyxml2 REQUIRED)

# -------------------------
# NI-DAQmx
# -------------------------
find_path(NIDAQMX_INCLUDE_DIR
    NAMES NIDAQmx.h
    PATHS
        /usr/include
        /usr/local/include
        /usr/local/natinst/nidaqmx/include
)

find_library(NIDAQMX_LIBRARY
    NAMES nidaqmx
    PATHS
        /usr/lib
        /usr/local/lib
        /usr/local/natinst/nidaqmx/lib
)

if (NOT NIDAQMX_INCLUDE_DIR OR NOT NIDAQMX_LIBRARY)
    message(FATAL_ERROR "NI-DAQmx not found.")
endif()

include_directories(${NIDAQMX_INCLUDE_DIR})

# -------------------------
# libbarrett
# -------------------------
find_package(Barrett REQUIRED)

include_directories(${BARRETT_INCLUDE_DIRS})
link_directories(${BARRETT_LIBRARY_DIRS})

add_definitions(${BARRETT_DEFINITIONS})

# -------------------------
# curses
# Often needed by libbarrett programs
# -------------------------
find_package(Curses REQUIRED)

include_directories(${CURSES_INCLUDE_DIR})

# -------------------------
# Function: add_ati_program
# -------------------------
function(add_ati_program prog)
    add_executable(${prog}
        src/${prog}.cpp
    )

    target_link_libraries(${prog}
        tinyxml2
        ${NIDAQMX_LIBRARY}
        ${BARRETT_LIBRARIES}
        ${CURSES_LIBRARIES}
        pthread
        rt
    )
endfunction()

# -------------------------
# Executables
# -------------------------
add_ati_program(test)
add_ati_program(record)
add_ati_program(test_ft_system)
add_ati_program(ft_compensation)