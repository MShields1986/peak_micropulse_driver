#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <mutex>
#include <atomic>
#include <functional>

#include <BoostSocketWrappers/tcp_client_boost.h>



class PeakHandler {
public:
// Output data structures: made to stay close to the LTPA DOF message
    enum DofHeaderByte {
        ascan,                         // 1A Hex
        normal_indications,            // 1C Hex
        gain_reduced_indications,      // 1D Hex
        lwl_coupling_failure,          // 1E Hex
        error                          // 0x6 and everything else
    };

    struct DofMessageHeader {
        DofHeaderByte                  header;
        int                            count;
        int                            testNo;
        int                            dof;
        int                            channel;
    };

    struct DofMessage {
        DofMessageHeader               header;
        std::vector<int32_t>           amps;
    };

    struct OutputFormat {
        int                            digitisation_rate;     // MHz
        int                            ascan_length;
        int                            num_a_scans;
        int                            n_elements;
        double                         element_pitch;         // mm
        double                         inter_element_spacing; // mm
        double                         element_width;         // mm
        double                         vel_wedge;             // m/s
        double                         vel_couplant;          // m/s
        double                         vel_material;          // m/s
        double                         wedge_angle;           // degrees
        double                         wedge_depth;           // mm
        double                         couplant_depth;        // mm
        double                         specimen_depth;        // mm
        std::vector<DofMessage>        ascans;
        int32_t                        max_amplitude;
    };

    explicit PeakHandler();
    ~PeakHandler();

    void                              setup(const int& frequency,
                                            const std::string& ip_address,
                                            const int& port,
                                            const std::string& mps_file);

    void                               logToConsole(const std::string& message);
    void                               errorToConsole(const std::string& message);
    void                               setReconstructionConfiguration(
                                                        const int& n_elements,
                                                        const double& element_pitch,         // mm
                                                        const double& inter_element_spacing, // mm
                                                        const double& element_width,         // mm
                                                        const double& vel_wedge,             // m/s
                                                        const double& vel_couplant,          // m/s
                                                        const double& vel_material,          // m/s
                                                        const double& wedge_angle,           // degrees
                                                        const double& wedge_depth,           // mm
                                                        const double& specimen_depth,        // mm
                                                        const double& couplant_depth);       // mm
    void                               readMpsFile();
    std::vector<std::string>           processMpsLine(const std::string& command);
    void                               setDof(const std::string& command);
    void                               setGates(const std::string& command);
    void                               setNumAScans(const std::string& command);
    void                               calcPacketLength();

    void                               connect();
    void                               sendCommand(const std::string& command);
    void                               sendReset(int digitisation_rate, int sleep_seconds = 10);
    void                               sendMpsConfiguration();
    DofMessage                         dataOutpoutFormatReader(const unsigned char* data, size_t length);
    DofMessage                         dataOutpoutFormatReader(const std::vector<unsigned char>& packet);
    bool                               sendDataRequest();

    // Async acquisition API
    using DataReadyCallback = std::function<void(bool)>;
    void                               startAsyncAcquisition(DataReadyCallback on_data_ready = nullptr,
                                                              int acquisition_rate_hz = 0);
    void                               stopAsyncAcquisition();
    bool                               getLatestData(OutputFormat& out);

private:
    friend class PeakHandlerTest;

    // Parse raw response bytes into an OutputFormat struct
    bool                               parseResponse(const std::vector<unsigned char>& response, OutputFormat& output);

    // Async loop internals
    void                               initiateAsyncReceive(uint64_t gen);
    void                               scheduleCalsTimer(uint64_t gen);
    void                               onReceiveComplete(const std::vector<unsigned char>& data, boost::system::error_code ec, uint64_t gen);

    OutputFormat                       ltpa_data_;
    OutputFormat*                      ltpa_data_ptr_;
public:
    const OutputFormat*                ltpa_data_ptr() const { return ltpa_data_ptr_; };

private:
    const int                                  sub_header_size_;
    int                                        frequency_;
    std::string                                ip_address_;
    int                                        port_;
    BoostSocketWrappers::TcpClientBoost        ltpa_client_;
    std::string                                mps_file_;
    std::vector<std::string>                   commands_;

public:
    int                                dof_;
    int                                gate_start_;
    int                                gate_end_;
    int                                ascan_length_;
    int                                num_a_scans_;

private:
    int                                individual_ascan_obs_length_;
    int                                packet_length_;

    // Async double-buffer members
    mutable std::mutex                 data_mutex_;
    OutputFormat                       ready_buffer_;
    std::atomic<bool>                  data_ready_{false};
    std::atomic<bool>                  acquiring_{false};
    std::atomic<uint64_t>              async_generation_{0};
    DataReadyCallback                  data_ready_cb_;

    // Decoupled timer for sending CALS commands at fixed rate
    std::unique_ptr<boost::asio::steady_timer> cals_timer_;
    std::chrono::milliseconds          cals_period_{50};  // Default 20 Hz
    std::atomic<bool>                  send_loop_active_{false};
};
