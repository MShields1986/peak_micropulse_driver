project(MockPeakHardware LANGUAGES CXX)

add_compile_options(-std=c++17)

find_package(Boost 1.71.0 REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)

set(LIBRARY_NAME "MockPeakHardware")
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(${LIBRARY_NAME} STATIC src/mock_peak_hardware.cpp)
target_include_directories(${LIBRARY_NAME} PUBLIC ${INCLUDE_DIR})
target_link_libraries(${LIBRARY_NAME} PUBLIC Boost::system Threads::Threads)
