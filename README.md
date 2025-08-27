# peak_micropulse_driver
A C++ driver to handle external control of the Peak MicroPulse hardware range over TCP.

# Including in an External Project
Use the inclusions below to have CMake fetch and build the library for you.
```cmake
include(FetchContent)
FetchContent_Declare(
  peak_micropulse
  GIT_REPOSITORY https://github.com/MShields1986/peak_micropulse_driver
  GIT_TAG        main
)
FetchContent_MakeAvailable(peak_micropulse)
```

Then link against your intended compilation target.
```cmake
target_link_libraries(${TARGET_NAME} PeakMicroPulseHandler)
```

# Usage
See the example, [standalone.cpp](https://github.com/MShields1986/peak_micropulse_driver/blob/main/examples/standalone.cpp), which can be compiled using...
```bash
cmake -DBUILD_PeakMicroPulse_EXAMPLES:BOOL=ON -S . -B build/
cmake --build build/
```

Then you can run it using...
```bash
./build/examples/example
```

The implementation within the example is as follows and provides a template for usage in your own code.
```cpp
#include "PeakMicroPulseHandler/peak_handler.h"


auto main(int argc, char** argv) -> int
{
    
    PeakHandler peak_handler(
        10,                                // Frequency -- Currently not used!
        "10.1.1.2",                        // IP address
        1067,                              // Port
        "examples/mps/roller_probe.mps"    // Relative path to .mps configuration file
        );

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
```

Data is output as a OutputFormat struct, which is defined in [peak_handler.h](https://github.com/MShields1986/peak_micropulse_driver/blob/main/peak_micropulse/include/PeakMicroPulseHandler/peak_handler.h). It would be worthwhile familiarising yourself with the Peak data output messages as defined in the [reference documentation](https://github.com/MShields1986/peak_micropulse_driver/blob/main/refs/PNL_1267_Issue_1_02_MicroPulse_Range_MP6_Command_Reference_Manual.pdf) to better understand these fields.

```cpp
// Output data structures: made to stay close to the Peak DOF message
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
        std::vector<short int>         amps;
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
    };
```

## Bugs and Feature Requests
Please report bugs and request features using the [Issue Tracker](https://github.com/MShields1986/peak_micropulse_driver/issues).

## Funding
The authors acknowledge the support of:
- RCSRF1920/10/32; Spirit Aerosystems/Royal Academy of Engineering Research Chair “In-process Non-destructive Testing for Aerospace Structures”
- Research Centre for Non-Destructive Evaluation Technology Transfer Project Scheme
