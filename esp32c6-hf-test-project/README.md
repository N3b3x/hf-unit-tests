# ESP32-C6 HardFOC Interface Test Project

A comprehensive test project for the ESP32-C6 microcontroller that demonstrates and validates all communication channels and hardware peripherals using the `hf-internal-interface-wrap` library.

## 🎯 Project Overview

This project provides:
- **Complete ESP32-C6 hardware testing** - ADC, GPIO, PWM, Timer, Temperature Sensor, NVS
- **Communication interface validation** - SPI, I2C, UART, CAN/TWAI, WiFi
- **Real-world pin mapping** optimized for ESP32-C6 DevKit
- **Comprehensive test suite** with detailed logging and error reporting
- **System health monitoring** with memory usage tracking
- **Production-ready code** with proper error handling and thread safety

## 🏗️ Project Structure

```
esp32c6-hf-test-project/
├── CMakeLists.txt                      # Main project CMake file
├── sdkconfig.defaults                  # ESP-IDF configuration defaults
├── partitions.csv                      # Custom partition table
├── README.md                          # This file
├── build_and_flash.sh                 # Build and flash script
├── main/                              # Main application
│   ├── CMakeLists.txt                 # Main component CMake
│   ├── main.cpp                       # Main application entry point
│   ├── hf_gpio_config.cpp             # GPIO pin configuration
│   ├── test_communication.cpp         # Communication tests
│   ├── test_hardware.cpp              # Hardware peripheral tests
│   └── include/                       # Header files
│       ├── hf_gpio_config.hpp         # GPIO configuration header
│       ├── test_communication.hpp     # Communication test header
│       └── test_hardware.hpp          # Hardware test header
└── components/                        # Custom components
    └── hf_internal_interface_wrap/    # HardFOC wrapper component
        └── CMakeLists.txt             # Component configuration
```

## 🔧 Hardware Requirements

### ESP32-C6 DevKit Board
- **ESP32-C6-DevKitC-1** or compatible
- **USB-C cable** for programming and power
- **Optional:** External components for advanced testing

### Pin Mapping (ESP32-C6 Optimized)

| Peripheral | Pin(s) | Description |
|------------|--------|-------------|
| **ADC** | GPIO0-6 | ADC1 channels 0-6 |
| **SPI** | GPIO10(CS), GPIO11(MOSI), GPIO12(SCLK), GPIO13(MISO) | SPI2 interface |
| **I2C** | GPIO14(SCL), GPIO15(SDA) | I2C0 interface |
| **UART** | GPIO16(TX), GPIO17(RX) | UART1 interface |
| **CAN** | GPIO20(TX), GPIO21(RX) | TWAI0 interface |
| **PWM** | GPIO22, GPIO23, GPIO7, GPIO8 | LEDC channels |
| **GPIO** | GPIO8(LED), GPIO9(Button) | Digital I/O |
| **Special** | GPIO30(Boot) | Boot button |

## 🚀 Quick Start

### Prerequisites

1. **ESP-IDF v5.1+** installed and configured
2. **Python 3.8+** with ESP-IDF tools
3. **ESP32-C6 development board**
4. **Git** for cloning repositories

### Build and Flash

1. **Clone and setup:**
```bash
# Navigate to the project directory (should already exist)
cd esp32c6-hf-test-project

# Set up ESP-IDF environment
. $IDF_PATH/export.sh
```

2. **Configure the project:**
```bash
# Set target to ESP32-C6
idf.py set-target esp32c6

# Configure project (optional - uses sdkconfig.defaults)
idf.py menuconfig
```

3. **Build the project:**
```bash
# Clean build
idf.py fullclean

# Build
idf.py build
```

4. **Flash and monitor:**
```bash
# Flash to device (replace /dev/ttyUSB0 with your port)
idf.py -p /dev/ttyUSB0 flash

# Monitor output
idf.py -p /dev/ttyUSB0 monitor
```

### Quick Build Script

Make the build script executable and run:
```bash
chmod +x build_and_flash.sh
./build_and_flash.sh
```

## 📋 Test Coverage

### Hardware Peripherals
- ✅ **GPIO** - Digital I/O, interrupts, LED control
- ✅ **ADC** - 12-bit ADC readings on channels 0-6
- ✅ **PWM/LEDC** - Variable duty cycle, frequency control
- ✅ **NVS** - Non-volatile storage (strings, integers, blobs)
- ✅ **Timer** - Periodic timers with microsecond precision
- ✅ **Temperature Sensor** - Internal temperature monitoring

### Communication Interfaces
- ✅ **SPI** - Master mode with configurable devices
- ✅ **I2C** - Master mode with device scanning
- ✅ **UART** - Serial communication with flow control
- ✅ **CAN/TWAI** - Controller Area Network communication
- ✅ **WiFi** - Station mode, scanning, MAC address

## 🔍 Test Output Example

```
ESP32-C6 HardFOC Interface Test Project
=========================================

=== ESP32-C6 Chip Information ===
Chip: esp32c6 with 1 CPU core(s), WiFi/BT/BLE, 802.15.4 (Zigbee/Thread)
Silicon revision: v0.0
Flash: 4MB embedded
Free heap: 379544 bytes

=== HardFOC Library Information ===
Target MCU: ESP32-C6
Architecture: RISC-V RV32IMAC
GPIO Pins: 31
ADC Channels: 7
Thread Safety: ENABLED

🔧 Starting Hardware Peripheral Tests...
✅ GPIO functionality test PASSED
✅ ADC functionality test PASSED  
✅ PWM functionality test PASSED
✅ NVS functionality test PASSED
✅ Timer functionality test PASSED
✅ Temperature Sensor test PASSED

📡 Starting Communication Interface Tests...
✅ SPI communication test PASSED
✅ I2C communication test PASSED
✅ UART communication test PASSED
✅ CAN communication test PASSED
✅ WiFi functionality test PASSED

========================================
Test Suite Results
========================================
Hardware Tests: ✅ PASSED
Communication Tests: ✅ PASSED
Overall Result: ✅ ALL TESTS PASSED
Test Duration: 15420 ms
========================================
```

## ⚙️ Configuration Options

### ESP-IDF Configuration (`sdkconfig.defaults`)

Key settings optimized for ESP32-C6:
- **CPU Frequency:** 160 MHz
- **Flash Mode:** QIO, 80 MHz
- **Compiler Optimization:** Size optimized
- **FreeRTOS:** 1000 Hz tick rate
- **WiFi:** WPA3 SAE enabled
- **Memory:** No external PSRAM

### HardFOC Configuration

Automatically configured for ESP32-C6:
- **Target MCU:** `HF_TARGET_MCU_ESP32C6`
- **Thread Safety:** Enabled
- **C++ Standard:** C++17
- **RTTI/Exceptions:** Disabled for performance

## 🛠️ Customization

### Adding New Tests

1. **Hardware tests:** Add to `test_hardware.cpp`
2. **Communication tests:** Add to `test_communication.cpp`
3. **Update headers:** Declare new functions in respective headers

### Modifying Pin Assignments

Edit `hf_gpio_config.hpp` to change pin mappings:
```cpp
// Example: Change SPI pins
#define HF_TEST_SPI_MOSI_PIN          GPIO_NUM_19  // Changed from GPIO_NUM_11
#define HF_TEST_SPI_MISO_PIN          GPIO_NUM_20  // Changed from GPIO_NUM_13
```

### Adding Components

1. Create component directory in `components/`
2. Add `CMakeLists.txt` with `idf_component_register()`
3. Update main `CMakeLists.txt` if needed

## 🐛 Troubleshooting

### Common Issues

1. **Build fails with "component not found":**
   - Check that `hf-internal-interface-wrap` exists at relative path
   - Verify all source files exist

2. **Flash fails:**
   - Check USB cable and port permissions
   - Try different baud rate: `idf.py -p PORT -b 460800 flash`

3. **Tests fail:**
   - Check hardware connections
   - Verify pin assignments match your board
   - Enable debug logging: Set `CONFIG_LOG_DEFAULT_LEVEL_DEBUG=y`

### Debug Modes

**Enable verbose logging:**
```bash
idf.py menuconfig
# Component config -> Log output -> Default log verbosity -> Debug
```

**Memory debugging:**
```bash
# Add to sdkconfig
CONFIG_HEAP_POISONING_COMPREHENSIVE=y
CONFIG_HEAP_TRACING_DEST_UART=y
```

## 📊 Performance Metrics

### Memory Usage (Typical)
- **Flash:** ~800KB (including ESP-IDF)
- **RAM:** ~45KB static + ~15KB dynamic
- **Free Heap:** ~380KB available

### Test Performance
- **Total test time:** ~15-20 seconds
- **Individual tests:** 1-3 seconds each
- **System monitor:** 1Hz reporting

## 🤝 Contributing

1. Fork the repository
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open Pull Request

## 📝 License

This project is part of the HardFOC ecosystem. See the main hf-internal-interface-wrap license for details.

## 🆘 Support

- **Documentation:** Check the hf-internal-interface-wrap docs/
- **Issues:** Create issues in the main repository
- **ESP-IDF Help:** [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/)

---

**Built with ❤️ for ESP32-C6 and HardFOC**
