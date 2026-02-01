cmake_minimum_required(VERSION 3.0..3.24)
project(peak_micropulse)

# Sanitizer options
option(ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(ENABLE_TSAN "Enable ThreadSanitizer" OFF)

if(ENABLE_ASAN AND ENABLE_TSAN)
    message(FATAL_ERROR "ASan and TSan cannot be enabled simultaneously")
endif()

if(ENABLE_ASAN)
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer -g)
    add_link_options(-fsanitize=address)
endif()

if(ENABLE_TSAN)
    add_compile_options(-fsanitize=thread -g)
    add_link_options(-fsanitize=thread)
endif()

option(BUILD_PeakMicroPulse_EXAMPLES "Build the example use cases" OFF)
if(BUILD_PeakMicroPulse_EXAMPLES)
    add_subdirectory(examples)
endif()

option(BUILD_PeakMicroPulse_TESTS "Build the unit tests" OFF)
if(BUILD_PeakMicroPulse_TESTS)
    enable_testing()
    add_subdirectory(mock_hardware)
    add_subdirectory(tests)
endif()

add_subdirectory(peak_micropulse)
