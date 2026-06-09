#include "MockPeakHardware/mock_peak_hardware.h"

#include <iostream>
#include <cstring>
#include <sstream>


MockPeakHardware::MockPeakHardware(const Config& config)
    : config_(config),
      io_context_(),
      acceptor_(io_context_),
      socket_(io_context_),
      response_timer_(nullptr)
{
}


MockPeakHardware::~MockPeakHardware() {
    stop();
}


void MockPeakHardware::start() {
    running_ = true;
    listening_ = false;

    server_thread_ = std::thread([this]() { serverLoop(); });

    // Block until the server is listening
    std::unique_lock<std::mutex> lock(start_mutex_);
    start_cv_.wait(lock, [this]() { return listening_.load(); });
}


void MockPeakHardware::stop() {
    // Set flag first (atomic write is visible to I/O thread)
    running_.store(false, std::memory_order_release);
    io_context_.stop();  // thread-safe, unblocks io_context::run()

    if (server_thread_.joinable()) {
        server_thread_.join();
    }

    // After join, single-threaded cleanup
    boost::system::error_code ec;
    socket_.close(ec);
    acceptor_.close(ec);
    response_timer_.reset();
}


int MockPeakHardware::port() const {
    return actual_port_;
}


void MockPeakHardware::serverLoop() {
    try {
        acceptor_.open(boost::asio::ip::tcp::v4());

        int one = 1;
        setsockopt(acceptor_.native_handle(), SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &one, sizeof(one));

        acceptor_.bind(boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), config_.port));
        acceptor_.listen();
        actual_port_ = acceptor_.local_endpoint().port();

        // Signal that we are listening
        {
            std::lock_guard<std::mutex> lock(start_mutex_);
            listening_ = true;
        }
        start_cv_.notify_one();

        doAccept();
        io_context_.run();
    } catch (const boost::system::system_error& e) {
        // Expected when stop() closes the socket/acceptor
        if (running_.load(std::memory_order_acquire)) {
            std::cerr << "MockPeakHardware: " << e.what() << std::endl;
        }
    }
}


void MockPeakHardware::doAccept() {
    acceptor_.async_accept(socket_, [this](boost::system::error_code ec) {
        if (!ec && running_.load(std::memory_order_acquire)) {
            read_buffer_.clear();
            doReadByte();
        }
    });
}


void MockPeakHardware::doReadByte() {
    boost::asio::async_read(socket_, boost::asio::buffer(&read_byte_, 1),
        [this](boost::system::error_code ec, std::size_t) {
            if (ec || !running_.load(std::memory_order_acquire)) return;
            read_buffer_.push_back(read_byte_);
            if (read_buffer_.size() >= 2 &&
                read_buffer_[read_buffer_.size()-2] == '\r' &&
                read_buffer_[read_buffer_.size()-1] == '\n') {
                std::string cmd = read_buffer_.substr(0, read_buffer_.size()-2);
                read_buffer_.clear();
                handleCommand(cmd);
            }
            doReadByte();
        });
}


void MockPeakHardware::handleCommand(const std::string& command) {
    if (command.rfind("RST", 0) == 0) {
        auto response = buildResetResponse();
        sendBytes(response);
        ++reset_count_;
    } else if (command.rfind("CALS", 0) == 0) {
        // Buffer the CALS command - hardware can buffer multiple commands
        ++pending_cals_count_;
        ++data_request_count_;

        // If no timer is currently running, schedule a response
        if (pending_cals_count_ == 1) {
            scheduleDataResponse();
        }
    } else if (command.rfind("GATS", 0) == 0 || command.rfind("GAT ", 0) == 0) {
        // Parse gate settings to determine response timing
        parseGatsCommand(command);
        ++config_lines_count_;
    } else {
        // MPS configuration line — absorb
        ++config_lines_count_;
    }
}


std::vector<unsigned char> MockPeakHardware::buildResetResponse() const {
    std::vector<unsigned char> response(32, 0);
    response[0]  = 0x23;  // '#' success marker
    response[4]  = static_cast<unsigned char>(config_.system_type);
    response[7]  = static_cast<unsigned char>(config_.dof);
    response[8]  = static_cast<unsigned char>(config_.default_dig_rate);
    response[9]  = static_cast<unsigned char>(config_.actual_dig_rate);
    response[10] = static_cast<unsigned char>(config_.dof);  // default DOF
    return response;
}


std::vector<unsigned char> MockPeakHardware::buildDataPacket() const {
    std::vector<unsigned char> packet;
    for (int i = 0; i < config_.num_a_scans; ++i) {
        auto ascan = buildAscanMessage(i);
        packet.insert(packet.end(), ascan.begin(), ascan.end());
    }
    return packet;
}


std::vector<unsigned char> MockPeakHardware::buildAscanMessage(int ascan_index) const {
    int data_bytes = (config_.dof == 4)
        ? 2 * config_.ascan_length
        : config_.ascan_length;
    int count = 8 + data_bytes;

    std::vector<unsigned char> msg(count, 0);

    // Sub-header
    msg[0] = 0x1A;
    msg[1] = static_cast<unsigned char>(count & 0xFF);
    msg[2] = static_cast<unsigned char>((count >> 8) & 0xFF);
    msg[3] = static_cast<unsigned char>((count >> 16) & 0xFF);
    int testNo = ascan_index + 1;
    msg[4] = static_cast<unsigned char>(testNo & 0xFF);
    msg[5] = static_cast<unsigned char>((testNo >> 8) & 0xFF);
    msg[6] = static_cast<unsigned char>(config_.dof);
    msg[7] = static_cast<unsigned char>(ascan_index & 0xFF);

    // Amplitude data — deterministic ramp pattern
    if (config_.dof == 1) {
        // 8-bit mode: zero point is 128
        for (int i = 0; i < config_.ascan_length; ++i) {
            int amp = 128 + ((ascan_index * 7 + i) % 100);
            msg[8 + i] = static_cast<unsigned char>(amp & 0xFF);
        }
    } else if (config_.dof == 4) {
        // 16-bit mode: zero point is 32768, little-endian
        for (int i = 0; i < config_.ascan_length; ++i) {
            uint16_t amp = static_cast<uint16_t>(32768 + ((ascan_index * 7 + i) % 1000));
            int offset = 8 + 2 * i;
            msg[offset]     = static_cast<unsigned char>(amp & 0xFF);
            msg[offset + 1] = static_cast<unsigned char>((amp >> 8) & 0xFF);
        }
    }

    return msg;
}


void MockPeakHardware::sendBytes(const std::vector<unsigned char>& data) {
    boost::system::error_code ec;
    boost::asio::write(socket_, boost::asio::buffer(data), ec);
    if (ec && running_.load(std::memory_order_acquire)) {
        std::cerr << "MockPeakHardware: send error: " << ec.message() << std::endl;
        running_.store(false, std::memory_order_release);
    }
}


void MockPeakHardware::parseGatsCommand(const std::string& command) {
    // Format: GAT(S) <test number> <gate start> <gate end>
    // Example: GATS 1 16 791 or GAT 1 1000 2000
    std::istringstream iss(command);
    std::string cmd;
    int test_num, gate_start, gate_end;

    iss >> cmd >> test_num >> gate_start >> gate_end;

    if (!iss.fail()) {
        config_.gate_start = gate_start;
        config_.gate_end = gate_end;
    }
}


std::chrono::microseconds MockPeakHardware::calculateResponseDelay() const {
    // Machine unit = 1 / digitization_rate in microseconds
    // For 100MHz: 1 machine unit = 0.01us (10ns)
    // For 50MHz:  1 machine unit = 0.02us (20ns)
    // For 25MHz:  1 machine unit = 0.04us (40ns)
    // For 10MHz:  1 machine unit = 0.1us  (100ns)

    // Gate end defines when the ultrasound measurement window closes
    // The response can only be sent after this time plus processing delay

    double machine_unit_us = 1.0 / static_cast<double>(config_.actual_dig_rate);
    double gate_time_us = static_cast<double>(config_.gate_end) * machine_unit_us;

    // Total delay = ultrasound measurement window + fixed processing/network delay
    int total_delay_us = static_cast<int>(gate_time_us) + config_.fixed_delay_us;

    return std::chrono::microseconds(total_delay_us);
}


void MockPeakHardware::scheduleDataResponse() {
    if (!response_timer_) {
        response_timer_ = std::make_unique<boost::asio::steady_timer>(io_context_);
    }

    auto delay = calculateResponseDelay();
    response_timer_->expires_after(delay);
    response_timer_->async_wait([this](boost::system::error_code ec) {
        if (ec || !running_.load(std::memory_order_acquire)) return;
        sendQueuedResponse();
    });
}


void MockPeakHardware::sendQueuedResponse() {
    if (pending_cals_count_ <= 0 || !running_.load(std::memory_order_acquire)) {
        return;
    }

    // Send ONE response
    auto packet = buildDataPacket();
    sendBytes(packet);
    --pending_cals_count_;

    // If more CALS are pending, schedule another response after the measurement delay
    if (pending_cals_count_ > 0) {
        scheduleDataResponse();
    }
}
