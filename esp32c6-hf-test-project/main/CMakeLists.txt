idf_component_register(
    SRCS 
        "main.cpp"
        "hf_gpio_config.cpp"
        "test_communication.cpp"
        "test_hardware.cpp"
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
        esp_log
)

# Set C++ standard for this component
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)

# Add compiler flags for better debugging and optimization
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -O2
    -g
    -DESP32C6
)

# Add definitions for ESP32-C6 specific features
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    CONFIG_ESP32C6_DEFAULT_CPU_FREQ_MHZ=160
    CONFIG_FREERTOS_HZ=1000
)
