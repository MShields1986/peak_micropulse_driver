cmake_minimum_required(VERSION 3.0..3.24)
project(PeakMicroPulseHandler
    VERSION 0.0.1
    DESCRIPTION "Class to handle external control of Peak MicroPulse hardware over TCP."
    LANGUAGES CXX)

add_compile_options(-std=c++17)

include(FetchContent)
FetchContent_Declare(
  simple_boost_asio_sockets
  GIT_REPOSITORY https://github.com/MShields1986/boost_asio_socket_wrappers
  GIT_TAG        feature/nodelet_compatibility
  # GIT_TAG        feature/async_io
)
FetchContent_MakeAvailable(simple_boost_asio_sockets)

set(LIBRARY_NAME ${PROJECT_NAME})
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(${LIBRARY_NAME} STATIC src/peak_handler.cpp)
target_include_directories(${LIBRARY_NAME} PUBLIC ${INCLUDE_DIR})
target_link_libraries(${LIBRARY_NAME} BoostSocketWrappers)

install(TARGETS ${LIBRARY_NAME})
install(DIRECTORY ${INCLUDE_DIR}/ DESTINATION include/${LIBRARY_NAME} FILES_MATCHING PATTERN "*.h*")
