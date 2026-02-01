cmake_minimum_required(VERSION 3.0..3.24)
project(peak_micropulse)

option(BUILD_PeakMicroPulse_EXAMPLES "Build the example use cases" OFF)
if(BUILD_PeakMicroPulse_EXAMPLES)
    add_subdirectory(examples)
endif()

option(BUILD_PeakMicroPulse_TESTS "Build the unit tests" OFF)
if(BUILD_PeakMicroPulse_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

add_subdirectory(peak_micropulse)
