#pragma once

#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <cstdint>
#include <chrono>
#include <queue>
#include <boost/asio.hpp>

class MockPeakHardware {
public:
    struct Config {
        int port             = 0;       // 0 = OS-assigned ephemeral port
        int dof              = 1;       // 1 (8-bit) or 4 (16-bit)
        int ascan_length     = 100;     // samples per A-scan
        int num_a_scans      = 5;       // number of focal laws / channels
        int system_type      = 0x30;    // byte 4 of RST response (LTPA)
        int default_dig_rate = 50;      // byte 8 of RST response (MHz)
        int actual_dig_rate  = 50;      // byte 9 of RST response (MHz)
        int gate_start       = 0;       // gate start in machine units
        int gate_end         = 1000;    // gate end in machine units
        int fixed_delay_us   = 100;     // fixed processing/network delay in microseconds
    };

    explicit MockPeakHardware(const Config& config);
    ~MockPeakHardware();

    // Start the server thread. Blocks until the server is listening.
    void start();

    // Stop the server, close socket, join thread.
    void stop();

    // Returns the actual port the server is listening on.
    int port() const;

    // Statistics for test assertions
    int resetCount() const        { return reset_count_.load(); }
    int configLinesCount() const  { return config_lines_count_.load(); }
    int dataRequestCount() const  { return data_request_count_.load(); }

private:
    void serverLoop();
    void doAccept();
    void doReadByte();
    void handleCommand(const std::string& command);
    void parseGatsCommand(const std::string& command);
    void scheduleDataResponse();
    void sendQueuedResponse();

    std::vector<unsigned char> buildResetResponse() const;
    std::vector<unsigned char> buildDataPacket() const;
    std::vector<unsigned char> buildAscanMessage(int ascan_index) const;

    void sendBytes(const std::vector<unsigned char>& data);

    // Calculate response delay based on gate settings and digitization rate
    std::chrono::microseconds calculateResponseDelay() const;

    Config                          config_;

    boost::asio::io_context         io_context_;
    boost::asio::ip::tcp::acceptor  acceptor_;
    boost::asio::ip::tcp::socket    socket_;
    std::unique_ptr<boost::asio::steady_timer> response_timer_;
    int                             actual_port_ = 0;

    std::thread                     server_thread_;
    std::atomic<bool>               listening_{false};
    std::atomic<bool>               running_{false};
    std::mutex                      start_mutex_;
    std::condition_variable         start_cv_;

    std::string                     read_buffer_;
    char                            read_byte_;

    // Queue for buffered CALS commands (hardware buffers commands)
    int                             pending_cals_count_{0};

    std::atomic<int>                reset_count_{0};
    std::atomic<int>                config_lines_count_{0};
    std::atomic<int>                data_request_count_{0};
};
