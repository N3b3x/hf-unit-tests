idf_component_register(
    SRCS 
        "main.cpp"
        "hf_gpio_config.cpp"
        # "test_communication.cpp"  # Temporarily disabled due to WiFi dependencies
        # "test_hardware.cpp"       # Temporarily disabled due to exception handling
    INCLUDE_DIRS 
        "."
        "include"
    REQUIRES 
        hf_internal_interface_wrap
        driver 
        esp_timer 
        freertos
        nvs_flash
        esp_wifi
        esp_netif
        esp_event
        esp_system
        log
)

# Set C++ standard for this component with GNU extensions
target_compile_options(${COMPONENT_LIB} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>
)

# Add compiler flags for better debugging and optimization
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    # -Wpedantic  # Disabled due to ESP-IDF internal macro usage
    -O2
    -g
    -DESP32C6
    # Suppress warnings from ESP-IDF headers
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-pedantic>
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-variadic-macros>
)

# Add definitions for ESP32-C6 specific features
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    CONFIG_ESP32C6_DEFAULT_CPU_FREQ_MHZ=160
    CONFIG_FREERTOS_HZ=1000
)
