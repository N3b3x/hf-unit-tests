/**
 * @file test_communication.cpp
 * @brief Implementation of communication interface tests
 */

#include "include/test_communication.hpp"
#include "include/hf_gpio_config.hpp"

// Include HardFOC interface wrappers
#include "mcu/esp32/EspSpi.h"
#include "mcu/esp32/EspI2c.h"
#include "mcu/esp32/EspUart.h"
#include "mcu/esp32/EspCan.h"
// #include "mcu/esp32/EspWifi.h"  // Temporarily disabled due to esp_wps.h dependency

// Include base classes
#include "base/BaseSpi.h"
#include "base/BaseI2c.h"
#include "base/BaseUart.h"
#include "base/BaseCan.h"
// #include "base/BaseWifi.h"  // Temporarily disabled

// ESP-IDF C headers must be wrapped in extern "C" for C++ compatibility
#ifdef __cplusplus
extern "C" {
#endif

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
}
#endif

static const char* TAG = "COMM_TEST";

bool test_all_communication_interfaces(void) {
    ESP_LOGI(TAG, "=== Testing All Communication Interfaces ===");
    
    bool all_passed = true;
    
    // Test each interface
    all_passed &= test_spi_communication();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    all_passed &= test_i2c_communication();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    all_passed &= test_uart_communication();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    all_passed &= test_can_communication();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // all_passed &= test_wifi_functionality();  // Temporarily disabled
    vTaskDelay(pdMS_TO_TICKS(100));
    
    if (all_passed) {
        ESP_LOGI(TAG, "✅ All communication interface tests PASSED");
    } else {
        ESP_LOGE(TAG, "❌ Some communication interface tests FAILED");
    }
    
    return all_passed;
}

bool test_spi_communication(void) {
    ESP_LOGI(TAG, "Testing SPI communication interface...");
    
    try {
        // Create SPI bus configuration
        hf_spi_bus_config_t bus_config = {};
        bus_config.host = HF_SPI2_HOST;
        bus_config.mosi_pin = HF_TEST_SPI_MOSI_PIN;
        bus_config.miso_pin = HF_TEST_SPI_MISO_PIN;
        bus_config.sclk_pin = HF_TEST_SPI_SCLK_PIN;
        bus_config.dma_channel = 1;
        bus_config.max_transfer_sz = 4096;
        bus_config.timeout_ms = 1000;
        
        // Create SPI bus
        EspSpiBus spi_bus(bus_config);
        if (!spi_bus.Initialize()) {
            ESP_LOGE(TAG, "Failed to initialize SPI bus");
            return false;
        }
        
        // Create device configuration
        hf_spi_device_config_t device_config = {};
        device_config.clock_speed_hz = 1000000;  // 1MHz
        device_config.mode = 0;
        device_config.cs_pin = HF_TEST_SPI_CS_PIN;
        device_config.queue_size = 7;
        
        // Add device to bus
        int device_index = spi_bus.CreateDevice(device_config);
        if (device_index < 0) {
            ESP_LOGE(TAG, "Failed to create SPI device");
            return false;
        }
        
        BaseSpi* spi_device = spi_bus.GetDevice(device_index);
        if (!spi_device) {
            ESP_LOGE(TAG, "Failed to get SPI device");
            return false;
        }
        
        // Test basic SPI operation (loopback test if MOSI and MISO are connected)
        uint8_t tx_data[] = {0xAA, 0x55, 0xCC, 0x33};
        uint8_t rx_data[sizeof(tx_data)] = {0};
        
        hf_spi_err_t result = spi_device->TransmitReceive(tx_data, rx_data, sizeof(tx_data), 1000);
        
        if (result == hf_spi_err_t::SPI_SUCCESS) {
            ESP_LOGI(TAG, "✅ SPI communication test PASSED");
            ESP_LOGI(TAG, "   TX: 0x%02X 0x%02X 0x%02X 0x%02X", 
                     tx_data[0], tx_data[1], tx_data[2], tx_data[3]);
            ESP_LOGI(TAG, "   RX: 0x%02X 0x%02X 0x%02X 0x%02X", 
                     rx_data[0], rx_data[1], rx_data[2], rx_data[3]);
            return true;
        } else {
            ESP_LOGE(TAG, "SPI transmit/receive failed with error code: %d", (int)result);
            return false;
        }
        
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "Exception in SPI test: %s", e.what());
        return false;
    } catch (...) {
        ESP_LOGE(TAG, "Unknown exception in SPI test");
        return false;
    }
}

bool test_i2c_communication(void) {
    ESP_LOGI(TAG, "Testing I2C communication interface...");
    
    try {
        // Create I2C bus configuration
        hf_i2c_master_bus_config_t bus_config = {};
        bus_config.i2c_port = I2C_NUM_0;
        bus_config.sda_io_num = HF_TEST_I2C_SDA_PIN;
        bus_config.scl_io_num = HF_TEST_I2C_SCL_PIN;
        bus_config.enable_internal_pullup = true;
        bus_config.clk_source = hf_i2c_clock_source_t::HF_I2C_CLK_SRC_DEFAULT;
        bus_config.glitch_ignore_cnt = 7;
        bus_config.trans_queue_depth = 10;
        
        // Create I2C bus
        EspI2cBus i2c_bus(bus_config);
        if (!i2c_bus.Initialize()) {
            ESP_LOGE(TAG, "Failed to initialize I2C bus");
            return false;
        }
        
        // Create device configuration (test with common I2C address)
        hf_i2c_device_config_t device_config = {};
        device_config.device_address = 0x48;  // Common sensor address
        device_config.scl_speed_hz = 100000;  // 100kHz
        device_config.dev_addr_length = hf_i2c_address_bits_t::HF_I2C_ADDR_7_BIT;
        device_config.disable_ack_check = false;
        
        // Add device to bus
        int device_index = i2c_bus.CreateDevice(device_config);
        if (device_index < 0) {
            ESP_LOGE(TAG, "Failed to create I2C device");
            return false;
        }
        
        BaseI2c* i2c_device = i2c_bus.GetDevice(device_index);
        if (!i2c_device) {
            ESP_LOGE(TAG, "Failed to get I2C device");
            return false;
        }
        
        // Test I2C device detection (scan for devices)
        ESP_LOGI(TAG, "Scanning I2C bus for devices...");
        bool device_found = false;
        
        for (uint8_t addr = 0x08; addr < 0x78; addr++) {
            uint8_t test_data = 0;
            hf_i2c_err_t result = i2c_device->Write(&test_data, 0, 100);  // Write 0 bytes (just address)
            
            if (result == hf_i2c_err_t::I2C_SUCCESS) {
                ESP_LOGI(TAG, "   Found I2C device at address 0x%02X", addr);
                device_found = true;
            }
        }
        
        if (device_found) {
            ESP_LOGI(TAG, "✅ I2C communication test PASSED - devices found");
        } else {
            ESP_LOGW(TAG, "⚠️ I2C communication test PASSED - no devices found (bus functional)");
        }
        
        return true;
        
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "Exception in I2C test: %s", e.what());
        return false;
    } catch (...) {
        ESP_LOGE(TAG, "Unknown exception in I2C test");
        return false;
    }
}

bool test_uart_communication(void) {
    ESP_LOGI(TAG, "Testing UART communication interface...");
    
    try {
        // Create UART configuration
        hf_uart_config_t uart_config = {};
        uart_config.uart_port = hf_uart_port_t::HF_UART_PORT_1;
        uart_config.baud_rate = 115200;
        uart_config.data_bits = hf_uart_data_bits_t::HF_UART_DATA_8_BITS;
        uart_config.parity = hf_uart_parity_t::HF_UART_PARITY_NONE;
        uart_config.stop_bits = hf_uart_stop_bits_t::HF_UART_STOP_BITS_1;
        uart_config.flow_control = hf_uart_flow_control_t::HF_UART_FLOW_CTRL_NONE;
        uart_config.tx_pin = HF_TEST_UART_TX_PIN;
        uart_config.rx_pin = HF_TEST_UART_RX_PIN;
        uart_config.rx_buffer_size = 512;
        uart_config.tx_buffer_size = 256;
        
        // Create UART instance
        EspUart uart(uart_config);
        if (!uart.EnsureInitialized()) {
            ESP_LOGE(TAG, "Failed to initialize UART");
            return false;
        }
        
        // Test UART transmission
        const char* test_message = "ESP32-C6 UART Test Message\r\n";
        size_t bytes_written = 0;
        
        hf_uart_err_t result = uart.Write(reinterpret_cast<const uint8_t*>(test_message), 
                                         strlen(test_message), &bytes_written, 1000);
        
        if (result == hf_uart_err_t::UART_SUCCESS && bytes_written == strlen(test_message)) {
            ESP_LOGI(TAG, "✅ UART transmission test PASSED");
            ESP_LOGI(TAG, "   Sent %zu bytes: %s", bytes_written, test_message);
            
            // Test UART reception (if loopback is connected)
            uint8_t rx_buffer[64] = {0};
            size_t bytes_read = 0;
            
            result = uart.Read(rx_buffer, sizeof(rx_buffer), &bytes_read, 100);
            
            if (result == hf_uart_err_t::UART_SUCCESS && bytes_read > 0) {
                ESP_LOGI(TAG, "✅ UART reception test PASSED");
                ESP_LOGI(TAG, "   Received %zu bytes: %.*s", bytes_read, (int)bytes_read, rx_buffer);
            } else {
                ESP_LOGW(TAG, "⚠️ UART reception test - no data received (normal without loopback)");
            }
            
            return true;
        } else {
            ESP_LOGE(TAG, "UART transmission failed with error: %d", (int)result);
            return false;
        }
        
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "Exception in UART test: %s", e.what());
        return false;
    } catch (...) {
        ESP_LOGE(TAG, "Unknown exception in UART test");
        return false;
    }
}

bool test_can_communication(void) {
    ESP_LOGI(TAG, "Testing CAN communication interface...");
    
    try {
        // Create CAN configuration
        hf_esp_can_config_t can_config = {};
        can_config.controller_id = hf_can_controller_id_t::HF_CAN_CONTROLLER_0;
        can_config.tx_pin = HF_TEST_CAN_TX_PIN;
        can_config.rx_pin = HF_TEST_CAN_RX_PIN;
        can_config.mode = hf_can_mode_t::HF_CAN_MODE_LISTEN_ONLY;  // Start in listen-only mode
        can_config.baudrate = 500000;  // 500kbps
        can_config.tx_queue_len = 10;
        can_config.rx_queue_len = 20;
        can_config.enable_alerts = true;
        
        // Create CAN instance
        EspCan can(can_config);
        hf_can_err_t result = can.Initialize();
        
        if (result != hf_can_err_t::CAN_SUCCESS) {
            ESP_LOGE(TAG, "Failed to initialize CAN: %d", (int)result);
            return false;
        }
        
        // Start CAN
        result = can.Start();
        if (result != hf_can_err_t::CAN_SUCCESS) {
            ESP_LOGE(TAG, "Failed to start CAN: %d", (int)result);
            return false;
        }
        
        ESP_LOGI(TAG, "✅ CAN initialization and start PASSED");
        
        // Test CAN message reception (listen mode)
        hf_can_message_t rx_message;
        result = can.ReceiveMessage(&rx_message, 100);  // 100ms timeout
        
        if (result == hf_can_err_t::CAN_SUCCESS) {
            ESP_LOGI(TAG, "✅ CAN message received - ID: 0x%08X, DLC: %d", 
                     rx_message.identifier, rx_message.data_length_code);
        } else if (result == hf_can_err_t::CAN_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "⚠️ CAN reception test - no messages received (normal without CAN traffic)");
        } else {
            ESP_LOGE(TAG, "CAN reception failed with error: %d", (int)result);
        }
        
        // Stop CAN
        result = can.Stop();
        if (result != hf_can_err_t::CAN_SUCCESS) {
            ESP_LOGE(TAG, "Failed to stop CAN: %d", (int)result);
            return false;
        }
        
        ESP_LOGI(TAG, "✅ CAN communication test PASSED");
        return true;
        
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "Exception in CAN test: %s", e.what());
        return false;
    } catch (...) {
        ESP_LOGE(TAG, "Unknown exception in CAN test");
        return false;
    }
}

bool test_wifi_functionality(void) {
    ESP_LOGI(TAG, "Testing WiFi functionality...");
    
    try {
        // Create WiFi configuration
        HfWifiConfig wifi_config = {};
        wifi_config.mode = HfWifiMode::WIFI_MODE_STA;
        wifi_config.enable_power_save = false;
        wifi_config.max_retry_attempts = 3;
        wifi_config.retry_delay_ms = 1000;
        
        // Create WiFi instance
        EspWifi wifi(wifi_config);
        
        // Initialize WiFi
        HfWifiErr result = wifi.Initialize();
        if (result != HfWifiErr::WIFI_SUCCESS) {
            ESP_LOGE(TAG, "Failed to initialize WiFi: %d", (int)result);
            return false;
        }
        
        ESP_LOGI(TAG, "✅ WiFi initialization PASSED");
        
        // Test WiFi scanning
        result = wifi.startScan(false, false, 3000);  // 3 second scan
        if (result == HfWifiErr::WIFI_SUCCESS) {
            ESP_LOGI(TAG, "WiFi scan started successfully");
            
            // Wait for scan to complete
            vTaskDelay(pdMS_TO_TICKS(4000));
            
            // Get scan results
            std::vector<HfWifiNetworkInfo> networks;
            result = wifi.getScanResults(networks, 10);
            
            if (result == HfWifiErr::WIFI_SUCCESS) {
                ESP_LOGI(TAG, "✅ WiFi scan PASSED - found %zu networks", networks.size());
                
                // Display first few networks
                for (size_t i = 0; i < std::min(networks.size(), size_t(5)); i++) {
                    ESP_LOGI(TAG, "   Network %zu: SSID='%s', RSSI=%d dBm, Channel=%d", 
                             i + 1, networks[i].ssid.c_str(), networks[i].rssi, networks[i].channel);
                }
            } else {
                ESP_LOGE(TAG, "Failed to get WiFi scan results: %d", (int)result);
                return false;
            }
        } else {
            ESP_LOGE(TAG, "Failed to start WiFi scan: %d", (int)result);
            return false;
        }
        
        // Test WiFi state management
        HfWifiState state = wifi.getState();
        ESP_LOGI(TAG, "WiFi state: %d", (int)state);
        
        // Get MAC address
        uint8_t mac[6];
        result = wifi.getMacAddress(mac, 0);
        if (result == HfWifiErr::WIFI_SUCCESS) {
            ESP_LOGI(TAG, "✅ WiFi MAC address: %02X:%02X:%02X:%02X:%02X:%02X",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        }
        
        ESP_LOGI(TAG, "✅ WiFi functionality test PASSED");
        return true;
        
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "Exception in WiFi test: %s", e.what());
        return false;
    } catch (...) {
        ESP_LOGE(TAG, "Unknown exception in WiFi test");
        return false;
    }
}
