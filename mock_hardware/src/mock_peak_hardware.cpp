#include "MockPeakHardware/mock_peak_hardware.h"

#include <iostream>
#include <cstring>


MockPeakHardware::MockPeakHardware(const Config& config)
    : config_(config),
      io_context_(),
      acceptor_(io_context_),
      socket_(io_context_)
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
    running_ = false;
    io_context_.stop();  // thread-safe, unblocks io_context::run()

    if (server_thread_.joinable()) {
        server_thread_.join();
    }

    // After join, single-threaded cleanup
    boost::system::error_code ec;
    socket_.close(ec);
    acceptor_.close(ec);
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
        if (running_) {
            std::cerr << "MockPeakHardware: " << e.what() << std::endl;
        }
    }
}


void MockPeakHardware::doAccept() {
    acceptor_.async_accept(socket_, [this](boost::system::error_code ec) {
        if (!ec && running_) {
            read_buffer_.clear();
            doReadByte();
        }
    });
}


void MockPeakHardware::doReadByte() {
    boost::asio::async_read(socket_, boost::asio::buffer(&read_byte_, 1),
        [this](boost::system::error_code ec, std::size_t) {
            if (ec || !running_) return;
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
        auto packet = buildDataPacket();
        sendBytes(packet);
        ++data_request_count_;
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
    if (ec && running_) {
        std::cerr << "MockPeakHardware: send error: " << ec.message() << std::endl;
        running_ = false;
    }
}
