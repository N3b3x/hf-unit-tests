# ESP32-C6 HardFOC Interface Test Project
cmake_minimum_required(VERSION 3.16)

# Set the components directory
set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/components
)

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Set project name and version
project(esp32c6_hf_test VERSION 1.0.0)

# Project-wide C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Suppress common warnings from ESP-IDF headers
add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-pedantic>
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-variadic-macros>
)
