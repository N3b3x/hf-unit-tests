cmake_minimum_required(VERSION 3.16)

# Path to the hf-internal-interface-wrap library - use absolute path
set(HF_INTERNAL_PATH "/Users/nebex/Documents/GitHub/hf-unit-tests/tests-internal-interface-wrap/hf-internal-interface-wrap")

message(STATUS "HF Internal Interface Path: ${HF_INTERNAL_PATH}")

# Check if the path exists
if(NOT EXISTS "${HF_INTERNAL_PATH}")
    message(FATAL_ERROR "hf-internal-interface-wrap not found at: ${HF_INTERNAL_PATH}")
endif()

# List all ESP32 implementation source files
set(HF_ESP32_SRCS
    "${HF_INTERNAL_PATH}/src/mcu/esp32/EspAdc.cpp"
    "${HF_INTERNAL_PATH}/src/mcu/esp32/EspCan.cpp"
    "${HF_INTERNAL_PATH}/src/mcu/esp32/EspGpio.cpp"
    "${HF_INTERNAL_PATH}/src/mcu/esp32/EspI2c.cpp"
    "${HF_INTERNAL_PATH}/src/mcu/esp32/EspLogger.cpp"
    "${HF_INTERNAL_PATH}/src/mcu/esp32/EspNvs.cpp"
    "${HF_INTERNAL_PATH}/src/mcu/esp32/EspPeriodicTimer.cpp"
    "${HF_INTERNAL_PATH}/src/mcu/esp32/EspPio.cpp"
    "${HF_INTERNAL_PATH}/src/mcu/esp32/EspPwm.cpp"
    "${HF_INTERNAL_PATH}/src/mcu/esp32/EspSpi.cpp"
    "${HF_INTERNAL_PATH}/src/mcu/esp32/EspTemperature.cpp"
    "${HF_INTERNAL_PATH}/src/mcu/esp32/EspUart.cpp"
)

# List utility source files
set(HF_UTIL_SRCS
    "${HF_INTERNAL_PATH}/src/utils/AsciiArtGenerator.cpp"
    "${HF_INTERNAL_PATH}/src/utils/DigitalOutputGuard.cpp"
)

# Check which source files actually exist
set(EXISTING_SRCS "")
foreach(src_file ${HF_ESP32_SRCS} ${HF_UTIL_SRCS})
    if(EXISTS "${src_file}")
        list(APPEND EXISTING_SRCS "${src_file}")
        message(STATUS "Found source: ${src_file}")
    else()
        message(WARNING "Source file not found: ${src_file}")
    endif()
endforeach()

# Register the component
idf_component_register(
    SRCS 
        ${EXISTING_SRCS}
        
    INCLUDE_DIRS 
        "${HF_INTERNAL_PATH}/inc"
        "${HF_INTERNAL_PATH}/inc/base"
        "${HF_INTERNAL_PATH}/inc/mcu"
        "${HF_INTERNAL_PATH}/inc/mcu/esp32"
        "${HF_INTERNAL_PATH}/inc/mcu/esp32/utils"
        "${HF_INTERNAL_PATH}/inc/utils"
        
    REQUIRES 
        # ESP-IDF core components
        driver 
        esp_timer 
        freertos
        nvs_flash
        esp_system
        esp_common
        esp_hw_support
        hal
        soc
        log
        
        # ADC components
        esp_adc
        
        # Driver components
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_spi
        esp_driver_uart
        esp_driver_ledc
        esp_driver_rmt
        esp_driver_gptimer
        esp_driver_tsens
        
        # Network components
        esp_wifi
        esp_netif
        esp_event
        lwip
        
        # Storage components
        nvs_flash
        nvs_sec_provider
        
        # Other essential components
        bootloader_support
        app_update
        spi_flash
        partition_table
        
    PRIV_REQUIRES
        # Private requirements (internal use only)
        esp_pm
        esp_phy
        wpa_supplicant
)

# Set C++ standard
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)

# Add ESP32-C6 specific definitions
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    HF_TARGET_MCU_ESP32C6=1
    HF_MCU_ESP32C6=1
    HF_MCU_FAMILY_ESP32=1
    HF_THREAD_SAFE=1
    ESP32C6=1
)

# Compiler flags for ESP32-C6 optimization
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    -O2
    -g
    -ffunction-sections
    -fdata-sections
    -Wno-pedantic
)

# Add specific flags for C++ files
target_compile_options(${COMPONENT_LIB} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-error=pedantic>
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-pedantic>
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-variadic-macros>
)

# Link time optimization for better performance
if(CONFIG_COMPILER_OPTIMIZATION_SIZE OR CONFIG_COMPILER_OPTIMIZATION_PERF)
    target_compile_options(${COMPONENT_LIB} PRIVATE -flto)
    target_link_options(${COMPONENT_LIB} PRIVATE -flto)
endif()

message(STATUS "HF Internal Interface Wrap component configured successfully")
list(LENGTH EXISTING_SRCS NUM_SOURCES)
message(STATUS "Included ${NUM_SOURCES} source files")
message(STATUS "Target MCU: ESP32-C6")
message(STATUS "C++ Standard: C++17")
message(STATUS "Thread Safety: ENABLED")
