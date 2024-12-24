cmake_minimum_required(VERSION 3.0..3.24)
project(examples)

add_executable(example standalone.cpp)

target_link_libraries(example PUBLIC PeakMicroPulseHandler)
