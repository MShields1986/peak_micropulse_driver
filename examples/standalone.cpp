#include "PeakMicroPulseHandler/peak_handler.h"


auto main(int argc, char** argv) -> int
{
    
    PeakHandler peak_handler(10, "10.1.1.2", 1067, "examples/mps/roller_probe.mps");

    const PeakHandler::OutputFormat* ltpa_data_ptr(peak_handler.ltpa_data_ptr());

    peak_handler.readMpsFile();
    peak_handler.connect();
    peak_handler.sendMpsConfiguration();

    for (int i=1; i<=10; i++){
        peak_handler.sendDataRequest();

        for (auto ascan : ltpa_data_ptr->ascans) {
            std::cout << ascan.header.testNo << std::endl;
        }
    }

    return 0;
}