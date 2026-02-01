find_package(GTest REQUIRED)

# Existing unit tests (no network, fast)
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

# Stress / integration tests (uses mock hardware, network)
add_executable(test_stress test_stress.cpp)
target_link_libraries(test_stress
    PeakMicroPulseHandler
    MockPeakHardware
    GTest::GTest
    GTest::Main
    pthread
)
target_compile_definitions(test_stress PRIVATE
    MPS_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../examples/mps"
)
add_test(NAME StressTests COMMAND test_stress)

# Valgrind / Helgrind wrappers
find_program(VALGRIND_EXECUTABLE valgrind)
if(VALGRIND_EXECUTABLE)
    add_test(NAME StressTests_Memcheck
        COMMAND ${VALGRIND_EXECUTABLE}
            --tool=memcheck
            --leak-check=full
            --error-exitcode=1
            $<TARGET_FILE:test_stress>
    )
    add_test(NAME StressTests_Helgrind
        COMMAND ${VALGRIND_EXECUTABLE}
            --tool=helgrind
            --error-exitcode=1
            $<TARGET_FILE:test_stress>
    )
    set_tests_properties(StressTests_Memcheck StressTests_Helgrind PROPERTIES
        LABELS "valgrind"
        TIMEOUT 300
    )
endif()
