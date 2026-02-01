find_package(GTest REQUIRED)

add_executable(test_peak_handler test_peak_handler.cpp)

target_link_libraries(test_peak_handler
    PeakMicroPulseHandler
    GTest::GTest
    GTest::Main
    pthread
)

target_compile_definitions(test_peak_handler PRIVATE
    MPS_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../examples/mps"
)

add_test(NAME PeakHandlerTests COMMAND test_peak_handler)
