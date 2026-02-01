#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <cmath>

#include "PeakMicroPulseHandler/peak_handler.h"
#include "MockPeakHardware/mock_peak_hardware.h"


// ---------------------------------------------------------------------------
// Test fixture — manages mock server lifecycle
// ---------------------------------------------------------------------------
class MockIntegrationTest : public ::testing::Test {
protected:
    MockPeakHardware::Config mock_config_;
    std::unique_ptr<MockPeakHardware> mock_;

    void startMock() {
        mock_ = std::make_unique<MockPeakHardware>(mock_config_);
        mock_->start();
    }

    // Create a PeakHandler, point it at the mock, read the MPS file,
    // connect, reset (with zero sleep), and send MPS config.
    std::unique_ptr<PeakHandler> connectHandler(const std::string& mps_file) {
        auto handler = std::make_unique<PeakHandler>();
        handler->setup(50, "127.0.0.1", mock_->port(), mps_file);
        handler->readMpsFile();
        handler->connect();
        handler->sendReset(mock_config_.actual_dig_rate, 0);
        handler->sendMpsConfiguration();
        return handler;
    }

    void TearDown() override {
        if (mock_) {
            mock_->stop();
        }
    }
};


// ===========================================================================
// 1. Synchronous round-trip — DOF 1 (8-bit)
// ===========================================================================
TEST_F(MockIntegrationTest, SyncRoundTrip_Dof1) {
    mock_config_.dof = 1;
    mock_config_.ascan_length = 775;
    mock_config_.num_a_scans = 49;
    startMock();

    auto handler = connectHandler(
        std::string(MPS_TEST_DATA_DIR) + "/Immersion_5MHz_128EL_Long_8_bit_mod.mps");

    ASSERT_TRUE(handler->sendDataRequest());

    const auto* data = handler->ltpa_data_ptr();
    ASSERT_NE(data, nullptr);
    EXPECT_EQ(static_cast<int>(data->ascans.size()), 49);

    // Verify first A-scan amplitudes match the mock's deterministic pattern
    const auto& first = data->ascans[0];
    EXPECT_EQ(first.header.dof, 1);
    EXPECT_EQ(first.header.testNo, 1);
    ASSERT_EQ(static_cast<int>(first.amps.size()), 775);
    // Mock pattern: raw byte = 128 + ((0*7 + i) % 100), parsed = raw - 128 = i % 100
    for (int i = 0; i < 10; ++i) {
        EXPECT_EQ(first.amps[i], i % 100) << "at sample " << i;
    }

    EXPECT_EQ(mock_->resetCount(), 1);
    EXPECT_EQ(mock_->dataRequestCount(), 1);
}


// ===========================================================================
// 2. Synchronous round-trip — DOF 4 (16-bit)
// ===========================================================================
TEST_F(MockIntegrationTest, SyncRoundTrip_Dof4) {
    mock_config_.dof = 4;
    mock_config_.ascan_length = 2000;
    mock_config_.num_a_scans = 61;
    startMock();

    auto handler = connectHandler(
        std::string(MPS_TEST_DATA_DIR) + "/roller_probe.mps");

    ASSERT_TRUE(handler->sendDataRequest());

    const auto* data = handler->ltpa_data_ptr();
    ASSERT_NE(data, nullptr);
    EXPECT_EQ(static_cast<int>(data->ascans.size()), 61);

    const auto& first = data->ascans[0];
    EXPECT_EQ(first.header.dof, 4);
    ASSERT_EQ(static_cast<int>(first.amps.size()), 2000);
    // Mock pattern: raw word = 32768 + ((0*7 + i) % 1000), parsed = raw - 32768 = i % 1000
    for (int i = 0; i < 10; ++i) {
        EXPECT_EQ(first.amps[i], i % 1000) << "at sample " << i;
    }
}


// ===========================================================================
// 3. Repeated synchronous requests — memory leak detection
// ===========================================================================
TEST_F(MockIntegrationTest, RepeatedSyncRequests) {
    mock_config_.dof = 1;
    mock_config_.ascan_length = 775;
    mock_config_.num_a_scans = 49;
    startMock();

    auto handler = connectHandler(
        std::string(MPS_TEST_DATA_DIR) + "/Immersion_5MHz_128EL_Long_8_bit_mod.mps");

    for (int i = 0; i < 100; ++i) {
        ASSERT_TRUE(handler->sendDataRequest()) << "Failed on iteration " << i;
    }

    EXPECT_EQ(mock_->dataRequestCount(), 100);
}


// ===========================================================================
// 4. Async acquisition — basic correctness
// ===========================================================================
TEST_F(MockIntegrationTest, AsyncBasic) {
    mock_config_.dof = 1;
    mock_config_.ascan_length = 100;
    mock_config_.num_a_scans = 5;
    startMock();

    auto handler = connectHandler(
        std::string(MPS_TEST_DATA_DIR) + "/Immersion_5MHz_128EL_Long_8_bit_mod.mps");

    // Override the handler's internal config to match our small mock
    handler->dof_ = 1;
    handler->ascan_length_ = 100;
    handler->num_a_scans_ = 5;
    handler->calcPacketLength();

    std::atomic<int> callback_count{0};
    std::atomic<int> valid_count{0};

    handler->startAsyncAcquisition([&](bool valid) {
        ++callback_count;
        if (valid) ++valid_count;
    });

    // Wait for at least 10 callbacks (with timeout)
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (callback_count < 10 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    handler->stopAsyncAcquisition();

    EXPECT_GE(callback_count.load(), 10);
    EXPECT_GE(valid_count.load(), 10);
}


// ===========================================================================
// 5. Async tight getLatestData loop — race condition detection
// ===========================================================================
TEST_F(MockIntegrationTest, AsyncTightGetLatestData) {
    mock_config_.dof = 1;
    mock_config_.ascan_length = 100;
    mock_config_.num_a_scans = 5;
    startMock();

    auto handler = connectHandler(
        std::string(MPS_TEST_DATA_DIR) + "/Immersion_5MHz_128EL_Long_8_bit_mod.mps");

    handler->dof_ = 1;
    handler->ascan_length_ = 100;
    handler->num_a_scans_ = 5;
    handler->calcPacketLength();

    handler->startAsyncAcquisition();

    // Wait until at least one frame arrives (tolerates Valgrind/TSan overhead)
    {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
        PeakHandler::OutputFormat tmp{};
        while (!handler->getLatestData(tmp)) {
            ASSERT_LT(std::chrono::steady_clock::now(), deadline)
                << "Timed out waiting for first async frame";
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // Two reader threads hammering getLatestData
    std::atomic<int> successes_a{0};
    std::atomic<int> successes_b{0};
    std::atomic<bool> corruption_detected{false};

    auto reader = [&](std::atomic<int>& successes) {
        for (int i = 0; i < 1000; ++i) {
            PeakHandler::OutputFormat out{};
            if (handler->getLatestData(out)) {
                ++successes;
                // Validate the parsed ascans vector is coherent
                if (static_cast<int>(out.ascans.size()) != 5) {
                    corruption_detected = true;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    };

    std::thread t1(reader, std::ref(successes_a));
    std::thread t2(reader, std::ref(successes_b));
    t1.join();
    t2.join();

    handler->stopAsyncAcquisition();

    EXPECT_FALSE(corruption_detected.load());
    // At least one reader should have gotten some data
    EXPECT_GT(successes_a.load() + successes_b.load(), 0);
}


// ===========================================================================
// 6. Rapid start/stop async acquisition — lifecycle stress
//
//    Exercises repeated start/stop cycles on the SAME connection.
//    stopAsyncAcquisition gracefully cancels pending I/O and drains the
//    socket buffer, so restarting reads clean data.
// ===========================================================================
TEST_F(MockIntegrationTest, AsyncRapidStartStop) {
    mock_config_.dof = 1;
    mock_config_.ascan_length = 100;
    mock_config_.num_a_scans = 5;
    startMock();

    auto handler = connectHandler(
        std::string(MPS_TEST_DATA_DIR) + "/Immersion_5MHz_128EL_Long_8_bit_mod.mps");

    handler->dof_ = 1;
    handler->ascan_length_ = 100;
    handler->num_a_scans_ = 5;
    handler->calcPacketLength();

    for (int i = 0; i < 50; ++i) {
        handler->startAsyncAcquisition();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        handler->stopAsyncAcquisition();
    }

    // After all the start/stop cycles, do one final round-trip to prove
    // the connection is still healthy and framing is intact.
    handler->startAsyncAcquisition();

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    PeakHandler::OutputFormat out{};
    bool got_data = false;
    while (std::chrono::steady_clock::now() < deadline) {
        if (handler->getLatestData(out)) {
            got_data = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    handler->stopAsyncAcquisition();

    ASSERT_TRUE(got_data);
    EXPECT_EQ(static_cast<int>(out.ascans.size()), 5);
}


// ===========================================================================
// 7. Large packet stress — DOF 4, big ascans
// ===========================================================================
TEST_F(MockIntegrationTest, LargePacketStress) {
    mock_config_.dof = 4;
    mock_config_.ascan_length = 4000;
    mock_config_.num_a_scans = 128;
    startMock();

    // Create handler with matching config (no MPS file needed for this)
    auto handler = std::make_unique<PeakHandler>();
    handler->setup(50, "127.0.0.1", mock_->port(), "");
    handler->dof_ = 4;
    handler->ascan_length_ = 4000;
    handler->num_a_scans_ = 128;
    handler->calcPacketLength();
    handler->connect();
    handler->sendReset(50, 0);

    for (int i = 0; i < 50; ++i) {
        ASSERT_TRUE(handler->sendDataRequest()) << "Failed on iteration " << i;
        const auto* data = handler->ltpa_data_ptr();
        EXPECT_EQ(static_cast<int>(data->ascans.size()), 128);
    }
}


// ===========================================================================
// 8. Async callback verification
// ===========================================================================
TEST_F(MockIntegrationTest, AsyncCallbackVerification) {
    mock_config_.dof = 1;
    mock_config_.ascan_length = 100;
    mock_config_.num_a_scans = 5;
    startMock();

    auto handler = connectHandler(
        std::string(MPS_TEST_DATA_DIR) + "/Immersion_5MHz_128EL_Long_8_bit_mod.mps");

    handler->dof_ = 1;
    handler->ascan_length_ = 100;
    handler->num_a_scans_ = 5;
    handler->calcPacketLength();

    std::atomic<int> true_callbacks{0};
    std::atomic<int> false_callbacks{0};

    handler->startAsyncAcquisition([&](bool valid) {
        if (valid) ++true_callbacks;
        else ++false_callbacks;
    });

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (true_callbacks < 20 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    handler->stopAsyncAcquisition();

    EXPECT_GE(true_callbacks.load(), 20);
    EXPECT_EQ(false_callbacks.load(), 0);
}


// ===========================================================================
// 9. Mock server restart — clean resource cleanup
// ===========================================================================
TEST_F(MockIntegrationTest, MockServerRestart) {
    mock_config_.dof = 1;
    mock_config_.ascan_length = 775;
    mock_config_.num_a_scans = 49;

    // First session
    {
        startMock();
        auto handler = connectHandler(
            std::string(MPS_TEST_DATA_DIR) + "/Immersion_5MHz_128EL_Long_8_bit_mod.mps");
        ASSERT_TRUE(handler->sendDataRequest());
        // handler and its socket destroyed here
    }
    mock_->stop();
    mock_.reset();

    // Second session on a new ephemeral port
    {
        startMock();
        auto handler = connectHandler(
            std::string(MPS_TEST_DATA_DIR) + "/Immersion_5MHz_128EL_Long_8_bit_mod.mps");
        ASSERT_TRUE(handler->sendDataRequest());
        EXPECT_EQ(mock_->dataRequestCount(), 1);  // fresh mock counter
    }
}
