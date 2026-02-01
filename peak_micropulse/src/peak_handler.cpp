#include "PeakMicroPulseHandler/peak_handler.h"



PeakHandler::PeakHandler()
    :  ltpa_data_(), ltpa_data_ptr_(&ltpa_data_),
       sub_header_size_(8),
       ltpa_client_()
{
}


PeakHandler::~PeakHandler() {
    stopAsyncAcquisition();
}


void PeakHandler::setup(const int& frequency,
                        const std::string& ip_address,
                        const int& port,
                        const std::string& mps_file)

{
    frequency_ = frequency;

    // Micropulse Hardware TCP Client
    ip_address_ = ip_address;
    port_ = port;
    ltpa_client_.setup(ip_address_, port_);

    // Micropulse Configuration
    mps_file_ = mps_file;
}


void PeakHandler::logToConsole(const std::string& message) {
    std::cout << "PeakHandler :: " << message << std::endl;
}


void PeakHandler::errorToConsole(const std::string& message) {
    std::cout << "\033[31m";
    std::cout << "PeakHandler :: " << message << std::endl;
    std::cout << "\033[0m";
}


void PeakHandler::setReconstructionConfiguration(
        const int& n_elements,
        const double& element_pitch,         // mm
        const double& inter_element_spacing, // mm
        const double& element_width,         // mm
        const double& vel_wedge,             // m/s
        const double& vel_couplant,          // m/s
        const double& vel_material,          // m/s
        const double& wedge_angle,           // degrees
        const double& wedge_depth,           // mm
        const double& couplant_depth,        // mm
        const double& specimen_depth) {      // mm

    ltpa_data_.n_elements = n_elements;
    ltpa_data_.element_pitch = element_pitch;
    ltpa_data_.inter_element_spacing = inter_element_spacing;
    ltpa_data_.element_width = element_width;
    ltpa_data_.vel_wedge = vel_wedge;
    ltpa_data_.vel_couplant = vel_couplant;
    ltpa_data_.vel_material = vel_material;
    ltpa_data_.wedge_angle = wedge_angle;
    ltpa_data_.wedge_depth = wedge_depth;
    ltpa_data_.couplant_depth = couplant_depth;
    ltpa_data_.specimen_depth = specimen_depth;
}

void PeakHandler::readMpsFile() {
    logToConsole("Attempting to open " + mps_file_);
    std::ifstream file;
    file.open(mps_file_);

    if (!file.is_open()) {
        errorToConsole("Error: Unable to open " + mps_file_);
        return;
    }

    std::string line;
    commands_.clear();

    while(std::getline(file, line))
    {
        // TODO: May need to trim leading and trailing whitespace here and ignore comments?
        // UTF-8 and strip white space
        commands_.push_back(line);

        // TODO: Consider parsing the NUM directive
        // TODO: Consider parsing TXF and RXF directives
        if (line.rfind("DOF", 0) == 0) {
            setDof(line);
        // TODO: Cover mps files that use the GAT command multiple times instead of GATS
        } else if (line.rfind("GATS", 0) == 0) {
            setGates(line);
        // TODO: Try and find some consensus about how best to determine the channel count
        //} else if (line.rfind("PAV", 0) == 0) {
        //    setNumAScans(line);
        } else if (line.rfind("SWP", 0) == 0) {
            setNumAScans(line);
        }
    }

    calcPacketLength();
    file.close();
    logToConsole("MPS file read successfully");
}


std::vector<std::string> PeakHandler::processMpsLine(const std::string& command) {
    std::string tmp;
    std::stringstream command_stream(command);
    std::vector<std::string> args;

    while(std::getline(command_stream, tmp, ' ')) {
        args.push_back(tmp);
    }
    return args;
}


void PeakHandler::setDof(const std::string& command) {
    logToConsole("Found data output format definition in MPS file: " + command);
    std::vector<std::string> args = processMpsLine(command);
    // Definition - DOF < Mode > [ Ascan mode ]
    dof_ = std::stoi(args[1]);
    logToConsole("Data output format: " + std::to_string(dof_));
}


void PeakHandler::setGates(const std::string& command) {
    logToConsole("Found gate definition in MPS file: " + command);
    std::vector<std::string> args = processMpsLine(command);
    // Definition - GAT(S) <test number><gate start><gate end>
    gate_start_ = std::stoi(args[2]);
    gate_end_ = std::stoi(args[3]);
    ascan_length_ = gate_end_ - gate_start_;

    ltpa_data_.ascan_length = ascan_length_;

    logToConsole("Gate start: " + std::to_string(gate_start_));
    logToConsole("Gate end: " + std::to_string(gate_end_));
}


void PeakHandler::setNumAScans(const std::string& command) {
    if (command.rfind("PAV", 0) == 0) {
        logToConsole("Found A-scan definition in MPS file: " + command);
        std::vector<std::string> args = processMpsLine(command);
        // Definition - PAV <channel start><channel end ><voltage>
        num_a_scans_ = std::stoi(args[2]);
        logToConsole("Number of A-Scans: " + std::to_string(num_a_scans_));

    } else if (command.rfind("SWP", 0) == 0) {
        logToConsole("Found A-scan definition in MPS file: " + command);
        std::vector<std::string> args = processMpsLine(command);
        // Definition - SWP <sweep No.> <start Tn> <-> <end Tn>
        num_a_scans_ = std::stoi(args[4]) - std::stoi(args[2]) + 1;
        logToConsole("Number of A-Scans: " + std::to_string(num_a_scans_));
    }

    ltpa_data_.num_a_scans = num_a_scans_;
}


void PeakHandler::calcPacketLength() {
    // 8 Bit Mode
    if (dof_ == 1) {
        individual_ascan_obs_length_ = ascan_length_ + sub_header_size_;

    // 16 Bit Mode
    } else if (dof_ == 4) {
        individual_ascan_obs_length_ = 2 * ascan_length_ + sub_header_size_;

    // TODO: Implememnt DOF 2, 3, 5 & 6

    } else {
        errorToConsole("ERROR - Unknown DOF in .mps file: " + std::to_string(dof_));
    }

    packet_length_ = num_a_scans_ * individual_ascan_obs_length_;

    logToConsole("Individual A-Scan length: " + std::to_string(individual_ascan_obs_length_));
    logToConsole("Packet length: " + std::to_string(packet_length_));
}


void PeakHandler::connect() {
    logToConsole("Connecting to LTPA at " + ip_address_);
    ltpa_client_.connect();
}


void PeakHandler::sendCommand(const std::string& command) {
    //logToConsole("Sending command: " + command);
    ltpa_client_.send(command + "\r\n\0");
}


void PeakHandler::sendReset(int digitisation_rate/* = 0*/) {
    int attempts(0);
    int max_attempts(2); // TODO: Consider exposing as arg
    bool success(false);

    while (attempts <= max_attempts and not success) {
        logToConsole("Attempting reset...");

        if (digitisation_rate == 0) {
            sendCommand("RST");
        } else if (digitisation_rate == 10) {
            sendCommand("RST 10");
        } else if (digitisation_rate == 25) {
            sendCommand("RST 25");
        } else if (digitisation_rate == 50) {
            sendCommand("RST 50");
        } else if (digitisation_rate == 100) {
            sendCommand("RST 100");
        } else {
            errorToConsole("Digitisation rate ought to be 0, 10, 25, 50 or 100 MHz.");        
            exit(0);
        }
        
        sleep(10);
        // Receive 32 bytes of data for the returned header after reset
        std::vector<unsigned char> response = ltpa_client_.receive(32);
        if ((int)response.front() == 35) {
            logToConsole("Reset successful");
            logToConsole(" -------- LTPA Status Info --------");

            logToConsole("System Type " + std::to_string((int)response[4]));
            // Decode of System type byte is as follows:
                                                    // Bit 4 - 7:
                                                    //     0 = MicroPulse 5
                                                    //     1 = MicroPulse LT1
                                                    //     2 = MicroPulse LT2
                                                    //     3 = LTPA
                                                    //     4 = MPLT
                                                    //     5 = MicroPulse 6


            // TODO: Note the spec document indexes from one and not zero!!
            logToConsole("Default data output format (DOF) " + std::to_string((uint8_t)response[10]));
            logToConsole("Actual data output format (DOF) " + std::to_string((uint8_t)response[7]));

            logToConsole("Default digitisation rate " + std::to_string((uint8_t)response[8]) + " MHz");
            logToConsole("Actual digitisation rate " + std::to_string((uint8_t)response[9]) + " MHz");
            logToConsole(" -------- ---------------- --------");

            ltpa_data_.digitisation_rate = (uint8_t)response[9];

            success = true;
        } else {
            errorToConsole("Reset failed");
            attempts++;
        }
    }

    if (attempts > max_attempts and not success) {
        errorToConsole("Unable to reset LTPA");        
        exit(0);
    }
}


void PeakHandler::sendMpsConfiguration() {
    for (auto command : commands_) {
        sendCommand(command);
    }
    logToConsole("MPS file commands sent to LTPA");
}


PeakHandler::DofMessage PeakHandler::dataOutpoutFormatReader(const std::vector<unsigned char>& packet) {
    DofMessage data;

    if (packet.front() == 26) {
        // TODO: Subheader processing to separate method
        data.header.header =        ascan;
        data.header.count =         (int)((packet[3] << 16) | (packet[2] << 8) | packet[1]);
        data.header.testNo =        (int)(packet[5] << 8 | packet[4]);
        data.header.dof =           (int)packet[6];
        data.header.channel =       (int)packet[7];

        // 8 Bit Mode
        if (data.header.dof == 1) {
            for (int i = sub_header_size_; i < data.header.count; i++) {
                data.amps.push_back( (int32_t)packet[i] - 128 );
            }

        // 16 Bit Mode
        } else if (data.header.dof == 4) {
            int i = sub_header_size_;
            while (i < data.header.count) {
                data.amps.push_back( (int32_t)(packet[i+1] << 8 | packet[i]) - 32768 );
                i = i+2;
            }
        } else {
        // TODO: Implememnt DOF 2, 3, 5 & 6
            errorToConsole("ERROR - Unkown DOF packet sub-header byte: " + std::to_string( data.header.dof ));
        }

    } else if (packet.front() == 28) {
        logToConsole("Normal indications returned");
        data.header.header = normal_indications;
        // TODO: Implement normal indications

    } else if (packet.front() == 29) {
        logToConsole("Gain reduced indications returned");
        data.header.header = gain_reduced_indications;
        // TODO: Implement gain reduced indications

    } else if (packet.front() == 30) {
        logToConsole("LWL coupling failure returned");
        data.header.header = lwl_coupling_failure;
        // TODO: Implement LWL (coupling failure)

    } else if (packet.front() == 6) {
        errorToConsole("ERROR - LTPA error message returned");
        data.header.header = error;
        // TODO: Implement better error message return handling

    } else {
        errorToConsole("ERROR - Unkown DOF packet sub-header byte: " + std::to_string((int)packet.front()));
        // TODO: Subheader processing to separate method
        data.header.header =        error;
        data.header.count =         (int)((packet[3] << 16) | (packet[2] << 8) | packet[1]);
        data.header.testNo =        (int)(packet[5] << 8 | packet[4]);
        data.header.dof =           (int)packet[6];
        data.header.channel =       (int)packet[7];
    }

    return data;
}


bool PeakHandler::parseResponse(const std::vector<unsigned char>& response, OutputFormat& output) {
    int ascan_count = 0;
    int curr_ascan_i = 0;

    int32_t data_max_amp = 0;

    std::vector<DofMessage> data;
    data.reserve(num_a_scans_);

    while (curr_ascan_i < packet_length_) {
        std::vector<unsigned char> ascan_bytes(
            &response[curr_ascan_i],
            &response[curr_ascan_i + individual_ascan_obs_length_]
            );

        DofMessage message = dataOutpoutFormatReader(ascan_bytes);

        if (message.header.header == ascan) {
            // DoF Validation
            if (message.header.dof != dof_) {
                errorToConsole(
                    "ERROR - Returned DOF [" +
                    std::to_string(message.header.dof) +
                    "] does not match MPS file [" +
                    std::to_string(dof_) +
                    "]"
                    );
                break;
            // Packet Length Validation
            } else if (message.header.count != individual_ascan_obs_length_) {
                errorToConsole(
                    "ERROR - Returned A-Scan length [" +
                    std::to_string(message.header.count) +
                    "] does not match MPS file [" +
                    std::to_string(individual_ascan_obs_length_) +
                    "]"
                    );
                break;

            // We have a good packet
            } else {
                int32_t curr_ascan_min_amp = *std::min_element(message.amps.begin(), message.amps.end());
                int32_t curr_ascan_max_amp = *std::max_element(message.amps.begin(), message.amps.end());

                if (abs(curr_ascan_min_amp) > abs(curr_ascan_max_amp)) {
                        curr_ascan_max_amp = abs(curr_ascan_min_amp);
                    } else {
                        curr_ascan_max_amp = abs(curr_ascan_max_amp);
                    }

                if (data_max_amp < curr_ascan_max_amp) {
                    data_max_amp = curr_ascan_max_amp;
                }

                data.emplace_back(std::move(message));
                ++ascan_count;
            }

        } else {
            errorToConsole("ERROR - Returned data message not an A Scan");
            break;
        }

        curr_ascan_i += message.header.count;
    }

    logToConsole(std::to_string(ascan_count) + " A-Scans Received");

    if (ascan_count == num_a_scans_) {
        output.ascans = std::move(data);
        output.max_amplitude = data_max_amp;
        return true;
    } else {
        errorToConsole("Incorrect amount of A-Scans returned");
        return false;
    }
}


bool PeakHandler::sendDataRequest() {
    // TODO: Get a handle on the behavior of these different commands
    //       and which is best for streaming and single measurements
    sendCommand("CALS 1");

    auto begin = std::chrono::high_resolution_clock::now();
    std::vector<unsigned char> response = ltpa_client_.receive(packet_length_);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "\033[32m";
    std::cout << "Profiling [ltpa_client_.receive(packet_length_)] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count() << " us" << std::endl;
    std::cout << "\033[0m";

    return parseResponse(response, ltpa_data_);
}


void PeakHandler::startAsyncAcquisition(DataReadyCallback on_data_ready) {
    if (acquiring_) {
        return;
    }
    // Copy static fields from ltpa_data_ into ready_buffer_ so consumers
    // see configuration values (digitisation_rate, geometry, etc.)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        ready_buffer_ = ltpa_data_;
    }
    data_ready_cb_ = on_data_ready;
    data_ready_ = false;
    acquiring_ = true;
    ltpa_client_.startIoThread();
    initiateAsyncRequest();
}


void PeakHandler::stopAsyncAcquisition() {
    acquiring_ = false;
    ltpa_client_.stopIoThread();
}


bool PeakHandler::getLatestData(OutputFormat& out) {
    if (!data_ready_) {
        return false;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    out = ready_buffer_;
    data_ready_ = false;
    return true;
}


void PeakHandler::initiateAsyncRequest() {
    sendCommand("CALS 1");
    ltpa_client_.asyncReceive(
        packet_length_,
        [this](std::vector<unsigned char> data, boost::system::error_code ec) {
            onReceiveComplete(std::move(data), ec);
        });
}


void PeakHandler::onReceiveComplete(std::vector<unsigned char> data, boost::system::error_code ec) {
    if (ec) {
        errorToConsole("Async receive error: " + ec.message());
        if (acquiring_) {
            initiateAsyncRequest();
        }
        return;
    }

    OutputFormat parsed = ltpa_data_; // copy static config fields
    bool valid = parseResponse(data, parsed);

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        ready_buffer_ = std::move(parsed);
        data_ready_ = true;
    }

    if (data_ready_cb_) {
        data_ready_cb_(valid);
    }

    if (acquiring_) {
        initiateAsyncRequest();
    }
}
