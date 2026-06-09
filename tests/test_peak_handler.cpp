#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <cstdint>
#include <cmath>

#include "PeakMicroPulseHandler/peak_handler.h"

// ---------------------------------------------------------------------------
// Test fixture — friend of PeakHandler, gives access to private members
// ---------------------------------------------------------------------------
class PeakHandlerTest : public ::testing::Test {
protected:
    PeakHandler handler_;

    // Accessors for private members
    int& packetLength()              { return handler_.packet_length_; }
    int& individualAscanObsLength()  { return handler_.individual_ascan_obs_length_; }
    std::atomic<bool>& dataReady()   { return handler_.data_ready_; }
    PeakHandler::OutputFormat& readyBuffer() { return handler_.ready_buffer_; }
    std::mutex& dataMutex()          { return handler_.data_mutex_; }

    bool callParseResponse(const std::vector<unsigned char>& response,
                           PeakHandler::OutputFormat& output) {
        return handler_.parseResponse(response, output);
    }

    // Helper: build an 8-bit (DOF 1) ascan sub-packet
    //   sub_header_size = 8, total count = 8 + ascan_length
    static std::vector<unsigned char> makeDof1Ascan(int ascanLength,
                                                     int testNo,
                                                     int channel,
                                                     unsigned char fillByte = 128) {
        int count = 8 + ascanLength;
        std::vector<unsigned char> pkt(count, fillByte);
        pkt[0] = 0x1A;                                        // marker
        pkt[1] = static_cast<unsigned char>(count & 0xFF);
        pkt[2] = static_cast<unsigned char>((count >> 8) & 0xFF);
        pkt[3] = static_cast<unsigned char>((count >> 16) & 0xFF);
        pkt[4] = static_cast<unsigned char>(testNo & 0xFF);
        pkt[5] = static_cast<unsigned char>((testNo >> 8) & 0xFF);
        pkt[6] = 1;                                           // dof
        pkt[7] = static_cast<unsigned char>(channel);
        return pkt;
    }

    // Helper: build a 16-bit (DOF 4) ascan sub-packet
    //   sub_header_size = 8, total count = 8 + 2*ascan_length
    static std::vector<unsigned char> makeDof4Ascan(int ascanLength,
                                                     int testNo,
                                                     int channel,
                                                     uint16_t fillWord = 32768) {
        int count = 8 + 2 * ascanLength;
        std::vector<unsigned char> pkt(count, 0);
        pkt[0] = 0x1A;
        pkt[1] = static_cast<unsigned char>(count & 0xFF);
        pkt[2] = static_cast<unsigned char>((count >> 8) & 0xFF);
        pkt[3] = static_cast<unsigned char>((count >> 16) & 0xFF);
        pkt[4] = static_cast<unsigned char>(testNo & 0xFF);
        pkt[5] = static_cast<unsigned char>((testNo >> 8) & 0xFF);
        pkt[6] = 4;                                           // dof
        pkt[7] = static_cast<unsigned char>(channel);
        // fill amplitude samples with fillWord (little-endian)
        for (int i = 8; i < count; i += 2) {
            pkt[i]     = static_cast<unsigned char>(fillWord & 0xFF);
            pkt[i + 1] = static_cast<unsigned char>((fillWord >> 8) & 0xFF);
        }
        return pkt;
    }

    // Build a full multi-ascan packet by concatenating individual ascans
    static std::vector<unsigned char> buildPacket(
            const std::vector<std::vector<unsigned char>>& ascans) {
        std::vector<unsigned char> pkt;
        for (const auto& a : ascans) {
            pkt.insert(pkt.end(), a.begin(), a.end());
        }
        return pkt;
    }
};

// ===========================================================================
// 1. ProcessMpsLine
// ===========================================================================
TEST_F(PeakHandlerTest, ProcessMpsLine_DofCommand) {
    auto tokens = handler_.processMpsLine("DOF 4");
    ASSERT_EQ(tokens.size(), 2u);
    EXPECT_EQ(tokens[0], "DOF");
    EXPECT_EQ(tokens[1], "4");
}

TEST_F(PeakHandlerTest, ProcessMpsLine_MultiToken) {
    auto tokens = handler_.processMpsLine("GATS 1 16 791");
    ASSERT_EQ(tokens.size(), 4u);
    EXPECT_EQ(tokens[0], "GATS");
    EXPECT_EQ(tokens[1], "1");
    EXPECT_EQ(tokens[2], "16");
    EXPECT_EQ(tokens[3], "791");
}

TEST_F(PeakHandlerTest, ProcessMpsLine_SwpWithDash) {
    auto tokens = handler_.processMpsLine("SWP 1 256 - 316");
    ASSERT_EQ(tokens.size(), 5u);
    EXPECT_EQ(tokens[0], "SWP");
    EXPECT_EQ(tokens[3], "-");
    EXPECT_EQ(tokens[4], "316");
}

TEST_F(PeakHandlerTest, ProcessMpsLine_SingleToken) {
    auto tokens = handler_.processMpsLine("RST");
    ASSERT_EQ(tokens.size(), 1u);
    EXPECT_EQ(tokens[0], "RST");
}

TEST_F(PeakHandlerTest, ProcessMpsLine_Empty) {
    auto tokens = handler_.processMpsLine("");
    EXPECT_TRUE(tokens.empty());
}

// ===========================================================================
// 2. SetDof
// ===========================================================================
TEST_F(PeakHandlerTest, SetDof_1) {
    handler_.setDof("DOF 1");
    EXPECT_EQ(handler_.dof_, 1);
}

TEST_F(PeakHandlerTest, SetDof_4) {
    handler_.setDof("DOF 4");
    EXPECT_EQ(handler_.dof_, 4);
}

// ===========================================================================
// 3. SetGates
// ===========================================================================
TEST_F(PeakHandlerTest, SetGates_16_791) {
    handler_.setGates("GATS 1 16 791");
    EXPECT_EQ(handler_.gate_start_, 16);
    EXPECT_EQ(handler_.gate_end_, 791);
    EXPECT_EQ(handler_.ascan_length_, 775);
}

TEST_F(PeakHandlerTest, SetGates_0_2000) {
    handler_.setGates("GATS 1 0 2000");
    EXPECT_EQ(handler_.gate_start_, 0);
    EXPECT_EQ(handler_.gate_end_, 2000);
    EXPECT_EQ(handler_.ascan_length_, 2000);
}

TEST_F(PeakHandlerTest, SetGates_500_2100) {
    handler_.setGates("GATS 1 500 2100");
    EXPECT_EQ(handler_.gate_start_, 500);
    EXPECT_EQ(handler_.gate_end_, 2100);
    EXPECT_EQ(handler_.ascan_length_, 1600);
}

// ===========================================================================
// 4. SetNumAScans
// ===========================================================================
TEST_F(PeakHandlerTest, SetNumAScans_61) {
    handler_.setNumAScans("SWP 1 256 - 316");
    EXPECT_EQ(handler_.num_a_scans_, 61);
}

TEST_F(PeakHandlerTest, SetNumAScans_49) {
    handler_.setNumAScans("SWP 1 256 - 304");
    EXPECT_EQ(handler_.num_a_scans_, 49);
}

// ===========================================================================
// 5. CalcPacketLength (uses private packet_length_)
// ===========================================================================
TEST_F(PeakHandlerTest, CalcPacketLength_Dof1) {
    handler_.dof_ = 1;
    handler_.ascan_length_ = 775;
    handler_.num_a_scans_ = 49;
    handler_.calcPacketLength();
    EXPECT_EQ(individualAscanObsLength(), 783);   // 775 + 8
    EXPECT_EQ(packetLength(), 38367);              // 49 * 783
}

TEST_F(PeakHandlerTest, CalcPacketLength_Dof4) {
    handler_.dof_ = 4;
    handler_.ascan_length_ = 2000;
    handler_.num_a_scans_ = 61;
    handler_.calcPacketLength();
    EXPECT_EQ(individualAscanObsLength(), 4008);   // 2*2000 + 8
    EXPECT_EQ(packetLength(), 244488);             // 61 * 4008
}

// ===========================================================================
// 6. DataOutpoutFormatReader
// ===========================================================================
TEST_F(PeakHandlerTest, DataOutputFormatReader_Dof1Header) {
    auto pkt = makeDof1Ascan(100, 42, 3);
    auto msg = handler_.dataOutpoutFormatReader(pkt);
    EXPECT_EQ(msg.header.header, PeakHandler::ascan);
    EXPECT_EQ(msg.header.count, 108);   // 8 + 100
    EXPECT_EQ(msg.header.testNo, 42);
    EXPECT_EQ(msg.header.dof, 1);
    EXPECT_EQ(msg.header.channel, 3);
}

TEST_F(PeakHandlerTest, DataOutputFormatReader_Dof1Amplitude) {
    auto pkt = makeDof1Ascan(4, 1, 0);
    // Set specific amplitude bytes at indices 8..11
    pkt[8]  = 128;  // 128 - 128 = 0
    pkt[9]  = 200;  // 200 - 128 = 72
    pkt[10] = 50;   //  50 - 128 = -78
    pkt[11] = 255;  // 255 - 128 = 127
    auto msg = handler_.dataOutpoutFormatReader(pkt);
    ASSERT_EQ(msg.amps.size(), 4u);
    EXPECT_EQ(msg.amps[0], 0);
    EXPECT_EQ(msg.amps[1], 72);
    EXPECT_EQ(msg.amps[2], -78);
    EXPECT_EQ(msg.amps[3], 127);
}

TEST_F(PeakHandlerTest, DataOutputFormatReader_Dof4Header) {
    auto pkt = makeDof4Ascan(100, 99, 7);
    auto msg = handler_.dataOutpoutFormatReader(pkt);
    EXPECT_EQ(msg.header.header, PeakHandler::ascan);
    EXPECT_EQ(msg.header.count, 208);   // 8 + 2*100
    EXPECT_EQ(msg.header.testNo, 99);
    EXPECT_EQ(msg.header.dof, 4);
    EXPECT_EQ(msg.header.channel, 7);
}

TEST_F(PeakHandlerTest, DataOutputFormatReader_Dof4Amplitude) {
    auto pkt = makeDof4Ascan(3, 1, 0);
    // Set specific 16-bit little-endian samples at indices 8..13
    // Sample 0: 32768 → 32768 - 32768 = 0
    pkt[8]  = 0x00; pkt[9]  = 0x80;
    // Sample 1: 33000 → 33000 - 32768 = 232
    pkt[10] = 0xE8; pkt[11] = 0x80;
    // Sample 2: 100 → 100 - 32768 = -32668
    pkt[12] = 0x64; pkt[13] = 0x00;
    auto msg = handler_.dataOutpoutFormatReader(pkt);
    ASSERT_EQ(msg.amps.size(), 3u);
    EXPECT_EQ(msg.amps[0], 0);
    EXPECT_EQ(msg.amps[1], 232);
    EXPECT_EQ(msg.amps[2], -32668);
}

TEST_F(PeakHandlerTest, DataOutputFormatReader_ErrorHeaders) {
    // 0x1C → normal_indications
    {
        std::vector<unsigned char> pkt(16, 0);
        pkt[0] = 0x1C;
        auto msg = handler_.dataOutpoutFormatReader(pkt);
        EXPECT_EQ(msg.header.header, PeakHandler::normal_indications);
    }
    // 0x1D → gain_reduced_indications
    {
        std::vector<unsigned char> pkt(16, 0);
        pkt[0] = 0x1D;
        auto msg = handler_.dataOutpoutFormatReader(pkt);
        EXPECT_EQ(msg.header.header, PeakHandler::gain_reduced_indications);
    }
    // 0x1E → lwl_coupling_failure
    {
        std::vector<unsigned char> pkt(16, 0);
        pkt[0] = 0x1E;
        auto msg = handler_.dataOutpoutFormatReader(pkt);
        EXPECT_EQ(msg.header.header, PeakHandler::lwl_coupling_failure);
    }
    // 0x06 → error
    {
        std::vector<unsigned char> pkt(16, 0);
        pkt[0] = 0x06;
        auto msg = handler_.dataOutpoutFormatReader(pkt);
        EXPECT_EQ(msg.header.header, PeakHandler::error);
    }
}

// ===========================================================================
// 7. ParseResponse (private, accessed via friend)
// ===========================================================================
TEST_F(PeakHandlerTest, ParseResponse_ValidDof1) {
    const int ascanLen = 100;
    const int numAscans = 5;
    handler_.dof_ = 1;
    handler_.ascan_length_ = ascanLen;
    handler_.num_a_scans_ = numAscans;
    handler_.calcPacketLength();

    std::vector<std::vector<unsigned char>> ascans;
    for (int i = 0; i < numAscans; ++i) {
        ascans.push_back(makeDof1Ascan(ascanLen, i + 1, i));
    }
    auto packet = buildPacket(ascans);

    PeakHandler::OutputFormat output{};
    EXPECT_TRUE(callParseResponse(packet, output));
    EXPECT_EQ(static_cast<int>(output.ascans.size()), numAscans);
}

TEST_F(PeakHandlerTest, ParseResponse_ValidDof4) {
    const int ascanLen = 50;
    const int numAscans = 3;
    handler_.dof_ = 4;
    handler_.ascan_length_ = ascanLen;
    handler_.num_a_scans_ = numAscans;
    handler_.calcPacketLength();

    std::vector<std::vector<unsigned char>> ascans;
    for (int i = 0; i < numAscans; ++i) {
        ascans.push_back(makeDof4Ascan(ascanLen, i + 1, i));
    }
    auto packet = buildPacket(ascans);

    PeakHandler::OutputFormat output{};
    EXPECT_TRUE(callParseResponse(packet, output));
    EXPECT_EQ(static_cast<int>(output.ascans.size()), numAscans);
}

TEST_F(PeakHandlerTest, ParseResponse_DofMismatch) {
    const int ascanLen = 100;
    const int numAscans = 2;
    handler_.dof_ = 1;
    handler_.ascan_length_ = ascanLen;
    handler_.num_a_scans_ = numAscans;
    handler_.calcPacketLength();

    // Build DOF 4 ascans but handler expects DOF 1
    // We need packets that are the right size for DOF 1 iteration,
    // but have DOF 4 in the header byte
    int dof1Count = 8 + ascanLen;
    std::vector<unsigned char> fakeAscan(dof1Count, 128);
    fakeAscan[0] = 0x1A;
    fakeAscan[1] = static_cast<unsigned char>(dof1Count & 0xFF);
    fakeAscan[2] = static_cast<unsigned char>((dof1Count >> 8) & 0xFF);
    fakeAscan[3] = 0;
    fakeAscan[4] = 1; fakeAscan[5] = 0;
    fakeAscan[6] = 4;  // DOF 4 — mismatch!
    fakeAscan[7] = 0;

    std::vector<std::vector<unsigned char>> ascans(numAscans, fakeAscan);
    auto packet = buildPacket(ascans);

    PeakHandler::OutputFormat output{};
    EXPECT_FALSE(callParseResponse(packet, output));
}

TEST_F(PeakHandlerTest, ParseResponse_CountMismatch) {
    const int ascanLen = 100;
    const int numAscans = 2;
    handler_.dof_ = 1;
    handler_.ascan_length_ = ascanLen;
    handler_.num_a_scans_ = numAscans;
    handler_.calcPacketLength();

    auto ascan = makeDof1Ascan(ascanLen, 1, 0);
    // Corrupt the count field to a wrong value
    int wrongCount = 50;
    ascan[1] = static_cast<unsigned char>(wrongCount & 0xFF);
    ascan[2] = static_cast<unsigned char>((wrongCount >> 8) & 0xFF);
    ascan[3] = 0;

    std::vector<std::vector<unsigned char>> ascans(numAscans, ascan);
    auto packet = buildPacket(ascans);

    PeakHandler::OutputFormat output{};
    EXPECT_FALSE(callParseResponse(packet, output));
}

TEST_F(PeakHandlerTest, ParseResponse_WrongAscanCount) {
    const int ascanLen = 100;
    handler_.dof_ = 1;
    handler_.ascan_length_ = ascanLen;
    handler_.num_a_scans_ = 5;  // expect 5
    handler_.calcPacketLength();

    // Only provide 3 ascans worth of data but with full packet_length_ buffer
    std::vector<std::vector<unsigned char>> ascans;
    for (int i = 0; i < 3; ++i) {
        ascans.push_back(makeDof1Ascan(ascanLen, i + 1, i));
    }
    auto partial = buildPacket(ascans);
    // Pad to full packet_length_ with zeros (will hit error header on next parse)
    partial.resize(packetLength(), 0);

    PeakHandler::OutputFormat output{};
    EXPECT_FALSE(callParseResponse(partial, output));
}

TEST_F(PeakHandlerTest, ParseResponse_MaxAmplitudePositive) {
    const int ascanLen = 4;
    const int numAscans = 1;
    handler_.dof_ = 1;
    handler_.ascan_length_ = ascanLen;
    handler_.num_a_scans_ = numAscans;
    handler_.calcPacketLength();

    auto ascan = makeDof1Ascan(ascanLen, 1, 0);
    // Amplitudes: 128→0, 228→100, 138→10, 148→20
    ascan[8]  = 128;
    ascan[9]  = 228;   // peak = 100
    ascan[10] = 138;
    ascan[11] = 148;

    auto packet = buildPacket({ascan});
    PeakHandler::OutputFormat output{};
    EXPECT_TRUE(callParseResponse(packet, output));
    EXPECT_EQ(output.max_amplitude, 100);
}

TEST_F(PeakHandlerTest, ParseResponse_MaxAmplitudeNegative) {
    const int ascanLen = 4;
    const int numAscans = 1;
    handler_.dof_ = 1;
    handler_.ascan_length_ = ascanLen;
    handler_.num_a_scans_ = numAscans;
    handler_.calcPacketLength();

    auto ascan = makeDof1Ascan(ascanLen, 1, 0);
    // Amplitudes: 128→0, 8→-120, 138→10, 148→20
    ascan[8]  = 128;
    ascan[9]  = 8;     // -120, abs = 120 → peak
    ascan[10] = 138;
    ascan[11] = 148;

    auto packet = buildPacket({ascan});
    PeakHandler::OutputFormat output{};
    EXPECT_TRUE(callParseResponse(packet, output));
    EXPECT_EQ(output.max_amplitude, 120);
}

// ===========================================================================
// 8. ReadMpsFile Integration
// ===========================================================================
TEST_F(PeakHandlerTest, ReadMpsFile_RollerProbe) {
    handler_.setup(50, "127.0.0.1", 1234,
                   std::string(MPS_TEST_DATA_DIR) + "/roller_probe.mps");
    handler_.readMpsFile();
    EXPECT_EQ(handler_.dof_, 4);
    EXPECT_EQ(handler_.num_a_scans_, 61);
    EXPECT_EQ(handler_.gate_start_, 0);
    EXPECT_EQ(handler_.gate_end_, 2000);
    EXPECT_EQ(handler_.ascan_length_, 2000);
}

TEST_F(PeakHandlerTest, ReadMpsFile_Immersion8Bit) {
    handler_.setup(50, "127.0.0.1", 1234,
                   std::string(MPS_TEST_DATA_DIR) + "/Immersion_5MHz_128EL_Long_8_bit_mod.mps");
    handler_.readMpsFile();
    EXPECT_EQ(handler_.dof_, 1);
    EXPECT_EQ(handler_.num_a_scans_, 49);
    EXPECT_EQ(handler_.gate_start_, 16);
    EXPECT_EQ(handler_.gate_end_, 791);
    EXPECT_EQ(handler_.ascan_length_, 775);
}

TEST_F(PeakHandlerTest, ReadMpsFile_Immersion16Bit) {
    handler_.setup(50, "127.0.0.1", 1234,
                   std::string(MPS_TEST_DATA_DIR) + "/Immersion_5MHz_128EL_Long_16_bit_mod.mps");
    handler_.readMpsFile();
    EXPECT_EQ(handler_.dof_, 4);
    EXPECT_EQ(handler_.num_a_scans_, 49);
    EXPECT_EQ(handler_.gate_start_, 16);
    EXPECT_EQ(handler_.gate_end_, 791);
    EXPECT_EQ(handler_.ascan_length_, 775);
}

TEST_F(PeakHandlerTest, ReadMpsFile_WingCover100MHz) {
    handler_.setup(50, "127.0.0.1", 1234,
                   std::string(MPS_TEST_DATA_DIR) + "/composite_roller_probe/roller_probe_wing_cover_100_MHz.mps");
    handler_.readMpsFile();
    EXPECT_EQ(handler_.dof_, 4);
    EXPECT_EQ(handler_.num_a_scans_, 61);
    EXPECT_EQ(handler_.gate_start_, 500);
    EXPECT_EQ(handler_.gate_end_, 2100);
    EXPECT_EQ(handler_.ascan_length_, 1600);
}

// ===========================================================================
// 9. GetLatestData
// ===========================================================================
TEST_F(PeakHandlerTest, GetLatestData_FreshHandlerReturnsFalse) {
    PeakHandler::OutputFormat out{};
    EXPECT_FALSE(handler_.getLatestData(out));
}

TEST_F(PeakHandlerTest, GetLatestData_AfterDataReady) {
    // Simulate data being ready
    {
        std::lock_guard<std::mutex> lock(dataMutex());
        readyBuffer().max_amplitude = 42;
        readyBuffer().num_a_scans = 10;
    }
    dataReady() = true;

    PeakHandler::OutputFormat out{};
    EXPECT_TRUE(handler_.getLatestData(out));
    EXPECT_EQ(out.max_amplitude, 42);
    EXPECT_EQ(out.num_a_scans, 10);

    // Second call should return false (data consumed)
    PeakHandler::OutputFormat out2{};
    EXPECT_FALSE(handler_.getLatestData(out2));
}
