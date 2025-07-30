/**
 * @file test_hardware.cpp
 * @brief Implementation of hardware peripheral tests
 */

#include "include/test_hardware.hpp"
#include "include/hf_gpio_config.hpp"

// Include HardFOC interface wrappers
#include "mcu/esp32/EspAdc.h"
#include "mcu/esp32/EspGpio.h"
#include "mcu/esp32/EspPwm.h"
#include "mcu/esp32/EspNvs.h"
#include "mcu/esp32/EspPeriodicTimer.h"
#include "mcu/esp32/EspTemperature.h"

// Include base classes
#include "base/BaseAdc.h"
#include "base/BaseGpio.h"
#include "base/BasePwm.h"
#include "base/BaseNvs.h"
#include "base/BasePeriodicTimer.h"
#include "base/BaseTemperature.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "HW_TEST";

bool test_all_hardware_peripherals(void) {
    ESP_LOGI(TAG, "=== Testing All Hardware Peripherals ===");
    
    bool all_passed = true;
    
    // Test each peripheral
    all_passed &= test_gpio_functionality();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    all_passed &= test_adc_functionality();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    all_passed &= test_pwm_functionality();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    all_passed &= test_nvs_functionality();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    all_passed &= test_timer_functionality();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    all_passed &= test_temperature_sensor();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    if (all_passed) {
        ESP_LOGI(TAG, "✅ All hardware peripheral tests PASSED");
    } else {
        ESP_LOGE(TAG, "❌ Some hardware peripheral tests FAILED");
    }
    
    return all_passed;
}

bool test_gpio_functionality(void) {
    ESP_LOGI(TAG, "Testing GPIO functionality...");
    
    try {
        // Create GPIO configuration
        hf_gpio_config_t gpio_config = {};
        gpio_config.pin_assignments = {
            HF_TEST_LED_BUILTIN_PIN,
            HF_TEST_GPIO_OUTPUT_0,
            HF_TEST_GPIO_OUTPUT_1,
            HF_TEST_BUTTON_PIN,
            HF_TEST_GPIO_INPUT_0,
            HF_TEST_GPIO_INPUT_1
        };
        gpio_config.enable_interrupts = true;
        gpio_config.enable_rtc_domain = false;
        gpio_config.enable_hold_function = false;
        
        // Create GPIO instance
        EspGpio gpio(gpio_config);
        if (!gpio.EnsureInitialized()) {
            ESP_LOGE(TAG, "Failed to initialize GPIO");
            return false;
        }
        
        ESP_LOGI(TAG, "✅ GPIO initialization PASSED");
        
        // Test digital output
        hf_gpio_err_t result = gpio.SetPinMode(HF_TEST_LED_BUILTIN_PIN, hf_gpio_mode_t::HF_GPIO_MODE_OUTPUT);
        if (result != hf_gpio_err_t::GPIO_SUCCESS) {
            ESP_LOGE(TAG, "Failed to set LED pin mode");
            return false;
        }
        
        // Test LED blinking
        ESP_LOGI(TAG, "Testing LED blinking...");
        for (int i = 0; i < 5; i++) {
            gpio.DigitalWrite(HF_TEST_LED_BUILTIN_PIN, hf_gpio_state_t::HF_GPIO_STATE_HIGH);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio.DigitalWrite(HF_TEST_LED_BUILTIN_PIN, hf_gpio_state_t::HF_GPIO_STATE_LOW);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        
        ESP_LOGI(TAG, "✅ GPIO digital output test PASSED");
        
        // Test digital input
        result = gpio.SetPinMode(HF_TEST_BUTTON_PIN, hf_gpio_mode_t::HF_GPIO_MODE_INPUT_PULLUP);
        if (result != hf_gpio_err_t::GPIO_SUCCESS) {
            ESP_LOGE(TAG, "Failed to set button pin mode");
            return false;
        }
        
        hf_gpio_state_t button_state = gpio.DigitalRead(HF_TEST_BUTTON_PIN);
        ESP_LOGI(TAG, "✅ GPIO digital input test PASSED - Button state: %s", 
                 (button_state == hf_gpio_state_t::HF_GPIO_STATE_HIGH) ? "HIGH" : "LOW");
        
        // Test multiple output pins
        gpio.SetPinMode(HF_TEST_GPIO_OUTPUT_0, hf_gpio_mode_t::HF_GPIO_MODE_OUTPUT);
        gpio.SetPinMode(HF_TEST_GPIO_OUTPUT_1, hf_gpio_mode_t::HF_GPIO_MODE_OUTPUT);
        
        ESP_LOGI(TAG, "Testing multiple GPIO outputs...");
        gpio.DigitalWrite(HF_TEST_GPIO_OUTPUT_0, hf_gpio_state_t::HF_GPIO_STATE_HIGH);
        gpio.DigitalWrite(HF_TEST_GPIO_OUTPUT_1, hf_gpio_state_t::HF_GPIO_STATE_LOW);
        vTaskDelay(pdMS_TO_TICKS(500));
        
        gpio.DigitalWrite(HF_TEST_GPIO_OUTPUT_0, hf_gpio_state_t::HF_GPIO_STATE_LOW);
        gpio.DigitalWrite(HF_TEST_GPIO_OUTPUT_1, hf_gpio_state_t::HF_GPIO_STATE_HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // Reset outputs
        gpio.DigitalWrite(HF_TEST_GPIO_OUTPUT_0, hf_gpio_state_t::HF_GPIO_STATE_LOW);
        gpio.DigitalWrite(HF_TEST_GPIO_OUTPUT_1, hf_gpio_state_t::HF_GPIO_STATE_LOW);
        
        ESP_LOGI(TAG, "✅ GPIO functionality test PASSED");
        return true;
        
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "Exception in GPIO test: %s", e.what());
        return false;
    } catch (...) {
        ESP_LOGE(TAG, "Unknown exception in GPIO test");
        return false;
    }
}

bool test_adc_functionality(void) {
    ESP_LOGI(TAG, "Testing ADC functionality...");
    
    try {
        // Create ADC configuration
        hf_adc_unit_config_t adc_config = {};
        adc_config.unit_id = hf_adc_unit_t::HF_ADC_UNIT_1;
        adc_config.resolution = hf_adc_bitwidth_t::HF_ADC_BITWIDTH_12;
        adc_config.reference_voltage_mv = 1100;  // ESP32-C6 internal reference
        adc_config.enable_calibration = true;
        adc_config.enable_continuous_mode = false;
        
        // Create ADC instance
        EspAdc adc(adc_config);
        if (!adc.EnsureInitialized()) {
            ESP_LOGE(TAG, "Failed to initialize ADC");
            return false;
        }
        
        ESP_LOGI(TAG, "✅ ADC initialization PASSED");
        
        // Configure ADC channels
        hf_adc_channel_config_t channel_configs[] = {
            {hf_adc_channel_t::HF_ADC_CHANNEL_0, hf_adc_attenuation_t::HF_ADC_ATTEN_DB_11},
            {hf_adc_channel_t::HF_ADC_CHANNEL_1, hf_adc_attenuation_t::HF_ADC_ATTEN_DB_11},
            {hf_adc_channel_t::HF_ADC_CHANNEL_2, hf_adc_attenuation_t::HF_ADC_ATTEN_DB_11},
        };
        
        for (const auto& config : channel_configs) {
            hf_adc_err_t result = adc.ConfigureChannel(config);
            if (result != hf_adc_err_t::ADC_SUCCESS) {
                ESP_LOGE(TAG, "Failed to configure ADC channel %d", (int)config.channel);
                return false;
            }
        }
        
        ESP_LOGI(TAG, "✅ ADC channel configuration PASSED");
        
        // Test ADC readings
        ESP_LOGI(TAG, "Reading ADC channels...");
        for (const auto& config : channel_configs) {
            hf_adc_raw_t raw_value;
            hf_adc_voltage_t voltage_mv;
            
            hf_adc_err_t result = adc.ReadRaw(config.channel, &raw_value);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                result = adc.ReadVoltage(config.channel, &voltage_mv);
                if (result == hf_adc_err_t::ADC_SUCCESS) {
                    ESP_LOGI(TAG, "   Channel %d: Raw=%d, Voltage=%d mV", 
                             (int)config.channel, raw_value, voltage_mv);
                } else {
                    ESP_LOGE(TAG, "Failed to read voltage from channel %d", (int)config.channel);
                    return false;
                }
            } else {
                ESP_LOGE(TAG, "Failed to read raw value from channel %d", (int)config.channel);
                return false;
            }
        }
        
        ESP_LOGI(TAG, "✅ ADC functionality test PASSED");
        return true;
        
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "Exception in ADC test: %s", e.what());
        return false;
    } catch (...) {
        ESP_LOGE(TAG, "Unknown exception in ADC test");
        return false;
    }
}

bool test_pwm_functionality(void) {
    ESP_LOGI(TAG, "Testing PWM functionality...");
    
    try {
        // Create PWM configuration
        hf_pwm_unit_config_t pwm_config = {};
        pwm_config.unit_id = 0;
        pwm_config.mode = hf_pwm_mode_t::HF_PWM_MODE_FADE;
        pwm_config.base_clock_hz = HF_PWM_APB_CLOCK_HZ;
        pwm_config.clock_source = hf_pwm_clock_source_t::HF_PWM_CLK_SRC_DEFAULT;
        pwm_config.enable_fade = true;
        pwm_config.enable_interrupts = false;
        
        // Create PWM instance
        EspPwm pwm(pwm_config);
        if (!pwm.EnsureInitialized()) {
            ESP_LOGE(TAG, "Failed to initialize PWM");
            return false;
        }
        
        ESP_LOGI(TAG, "✅ PWM initialization PASSED");
        
        // Configure PWM channels
        struct {
            hf_pwm_channel_t channel;
            hf_pin_num_t pin;
            const char* name;
        } pwm_channels[] = {
            {hf_pwm_channel_t::HF_PWM_CHANNEL_0, HF_TEST_PWM_CHANNEL_0_PIN, "Channel 0"},
            {hf_pwm_channel_t::HF_PWM_CHANNEL_1, HF_TEST_PWM_CHANNEL_1_PIN, "Channel 1"},
            {hf_pwm_channel_t::HF_PWM_CHANNEL_2, HF_TEST_PWM_CHANNEL_2_PIN, "Channel 2"},
        };
        
        for (const auto& ch : pwm_channels) {
            hf_pwm_channel_config_t channel_config = {};
            channel_config.channel = ch.channel;
            channel_config.pin = ch.pin;
            channel_config.timer_id = 0;
            channel_config.frequency_hz = 1000;  // 1kHz
            channel_config.resolution_bits = 10;  // 10-bit resolution (0-1023)
            channel_config.duty_cycle = 0;       // Start at 0%
            channel_config.phase_shift_degrees = 0;
            
            hf_pwm_err_t result = pwm.ConfigureChannel(channel_config);
            if (result != hf_pwm_err_t::PWM_SUCCESS) {
                ESP_LOGE(TAG, "Failed to configure PWM %s", ch.name);
                return false;
            }
        }
        
        ESP_LOGI(TAG, "✅ PWM channel configuration PASSED");
        
        // Test PWM output with varying duty cycles
        ESP_LOGI(TAG, "Testing PWM duty cycle sweep...");
        for (const auto& ch : pwm_channels) {
            ESP_LOGI(TAG, "   Testing %s (GPIO%d)", ch.name, ch.pin);
            
            // Start PWM
            hf_pwm_err_t result = pwm.Start(ch.channel);
            if (result != hf_pwm_err_t::PWM_SUCCESS) {
                ESP_LOGE(TAG, "Failed to start PWM %s", ch.name);
                return false;
            }
            
            // Sweep duty cycle from 0% to 100% and back
            for (uint32_t duty = 0; duty <= 1023; duty += 100) {
                pwm.SetDutyCycle(ch.channel, duty);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            
            for (int32_t duty = 1023; duty >= 0; duty -= 100) {
                pwm.SetDutyCycle(ch.channel, duty);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            
            // Stop PWM
            pwm.Stop(ch.channel);
        }
        
        ESP_LOGI(TAG, "✅ PWM functionality test PASSED");
        return true;
        
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "Exception in PWM test: %s", e.what());
        return false;
    } catch (...) {
        ESP_LOGE(TAG, "Unknown exception in PWM test");
        return false;
    }
}

bool test_nvs_functionality(void) {
    ESP_LOGI(TAG, "Testing NVS functionality...");
    
    try {
        // Create NVS configuration
        hf_nvs_config_t nvs_config = {};
        nvs_config.partition_name = "nvs";
        nvs_config.namespace_name = "test_storage";
        nvs_config.enable_encryption = false;
        nvs_config.enable_compression = false;
        nvs_config.max_key_length = 15;
        nvs_config.max_value_size = 4000;
        
        // Create NVS instance
        EspNvs nvs(nvs_config);
        if (!nvs.EnsureInitialized()) {
            ESP_LOGE(TAG, "Failed to initialize NVS");
            return false;
        }
        
        ESP_LOGI(TAG, "✅ NVS initialization PASSED");
        
        // Test different data types
        const char* test_key_str = "test_string";
        const char* test_value_str = "ESP32-C6 Test Value";
        const char* test_key_int = "test_integer";
        int32_t test_value_int = 12345;
        const char* test_key_blob = "test_blob";
        uint8_t test_value_blob[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
        
        // Test string storage
        hf_nvs_err_t result = nvs.SetString(test_key_str, test_value_str);
        if (result != hf_nvs_err_t::NVS_SUCCESS) {
            ESP_LOGE(TAG, "Failed to store string value");
            return false;
        }
        
        std::string retrieved_str;
        result = nvs.GetString(test_key_str, retrieved_str);
        if (result != hf_nvs_err_t::NVS_SUCCESS || retrieved_str != test_value_str) {
            ESP_LOGE(TAG, "Failed to retrieve string value correctly");
            return false;
        }
        
        ESP_LOGI(TAG, "✅ NVS string storage test PASSED: '%s' = '%s'", 
                 test_key_str, retrieved_str.c_str());
        
        // Test integer storage
        result = nvs.SetInteger(test_key_int, test_value_int);
        if (result != hf_nvs_err_t::NVS_SUCCESS) {
            ESP_LOGE(TAG, "Failed to store integer value");
            return false;
        }
        
        int32_t retrieved_int;
        result = nvs.GetInteger(test_key_int, retrieved_int);
        if (result != hf_nvs_err_t::NVS_SUCCESS || retrieved_int != test_value_int) {
            ESP_LOGE(TAG, "Failed to retrieve integer value correctly");
            return false;
        }
        
        ESP_LOGI(TAG, "✅ NVS integer storage test PASSED: '%s' = %d", 
                 test_key_int, retrieved_int);
        
        // Test blob storage
        result = nvs.SetBlob(test_key_blob, test_value_blob, sizeof(test_value_blob));
        if (result != hf_nvs_err_t::NVS_SUCCESS) {
            ESP_LOGE(TAG, "Failed to store blob value");
            return false;
        }
        
        uint8_t retrieved_blob[sizeof(test_value_blob)];
        size_t blob_size = sizeof(retrieved_blob);
        result = nvs.GetBlob(test_key_blob, retrieved_blob, blob_size);
        if (result != hf_nvs_err_t::NVS_SUCCESS || 
            blob_size != sizeof(test_value_blob) ||
            memcmp(retrieved_blob, test_value_blob, blob_size) != 0) {
            ESP_LOGE(TAG, "Failed to retrieve blob value correctly");
            return false;
        }
        
        ESP_LOGI(TAG, "✅ NVS blob storage test PASSED: '%s' = [%zu bytes]", 
                 test_key_blob, blob_size);
        
        // Test key existence
        bool exists = nvs.HasKey(test_key_str);
        if (!exists) {
            ESP_LOGE(TAG, "Key existence check failed");
            return false;
        }
        
        // Test key deletion
        result = nvs.EraseKey(test_key_str);
        if (result != hf_nvs_err_t::NVS_SUCCESS) {
            ESP_LOGE(TAG, "Failed to delete key");
            return false;
        }
        
        exists = nvs.HasKey(test_key_str);
        if (exists) {
            ESP_LOGE(TAG, "Key still exists after deletion");
            return false;
        }
        
        ESP_LOGI(TAG, "✅ NVS functionality test PASSED");
        return true;
        
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "Exception in NVS test: %s", e.what());
        return false;
    } catch (...) {
        ESP_LOGE(TAG, "Unknown exception in NVS test");
        return false;
    }
}

bool test_timer_functionality(void) {
    ESP_LOGI(TAG, "Testing Timer functionality...");
    
    try {
        // Create timer configuration
        hf_periodic_timer_config_t timer_config = {};
        timer_config.timer_id = hf_timer_id_t::HF_TIMER_0;
        timer_config.period_us = 1000000;  // 1 second
        timer_config.auto_reload = true;
        timer_config.enable_interrupts = true;
        timer_config.callback = nullptr;  // No callback for this test
        
        // Create timer instance
        EspPeriodicTimer timer(timer_config);
        if (!timer.EnsureInitialized()) {
            ESP_LOGE(TAG, "Failed to initialize Timer");
            return false;
        }
        
        ESP_LOGI(TAG, "✅ Timer initialization PASSED");
        
        // Test timer start/stop functionality
        hf_timer_err_t result = timer.Start();
        if (result != hf_timer_err_t::TIMER_SUCCESS) {
            ESP_LOGE(TAG, "Failed to start timer");
            return false;
        }
        
        ESP_LOGI(TAG, "Timer started, running for 3 seconds...");
        vTaskDelay(pdMS_TO_TICKS(3000));
        
        // Check if timer is running
        bool is_running = timer.IsRunning();
        if (!is_running) {
            ESP_LOGE(TAG, "Timer should be running but reports as stopped");
            return false;
        }
        
        // Get timer count
        uint64_t count = timer.GetTimerCount();
        ESP_LOGI(TAG, "Timer count: %llu", count);
        
        // Stop timer
        result = timer.Stop();
        if (result != hf_timer_err_t::TIMER_SUCCESS) {
            ESP_LOGE(TAG, "Failed to stop timer");
            return false;
        }
        
        // Verify timer is stopped
        is_running = timer.IsRunning();
        if (is_running) {
            ESP_LOGE(TAG, "Timer should be stopped but reports as running");
            return false;
        }
        
        ESP_LOGI(TAG, "✅ Timer functionality test PASSED");
        return true;
        
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "Exception in Timer test: %s", e.what());
        return false;
    } catch (...) {
        ESP_LOGE(TAG, "Unknown exception in Timer test");
        return false;
    }
}

bool test_temperature_sensor(void) {
    ESP_LOGI(TAG, "Testing Temperature Sensor functionality...");
    
    try {
        // Create temperature sensor configuration
        hf_temperature_config_t temp_config = {};
        temp_config.sensor_id = hf_temperature_sensor_id_t::HF_TEMP_SENSOR_INTERNAL;
        temp_config.sample_rate_hz = 1;  // 1Hz sampling
        temp_config.enable_interrupts = false;
        temp_config.high_threshold_celsius = 85.0f;
        temp_config.low_threshold_celsius = -40.0f;
        
        // Create temperature sensor instance
        EspTemperature temp_sensor(temp_config);
        if (!temp_sensor.EnsureInitialized()) {
            ESP_LOGE(TAG, "Failed to initialize Temperature Sensor");
            return false;
        }
        
        ESP_LOGI(TAG, "✅ Temperature Sensor initialization PASSED");
        
        // Test temperature reading
        float temperature_celsius;
        hf_temperature_err_t result = temp_sensor.ReadTemperature(&temperature_celsius);
        
        if (result == hf_temperature_err_t::TEMPERATURE_SUCCESS) {
            ESP_LOGI(TAG, "✅ Temperature reading PASSED: %.2f°C", temperature_celsius);
            
            // Sanity check - temperature should be within reasonable range
            if (temperature_celsius >= -40.0f && temperature_celsius <= 125.0f) {
                ESP_LOGI(TAG, "✅ Temperature value is within reasonable range");
            } else {
                ESP_LOGW(TAG, "⚠️ Temperature value seems out of range: %.2f°C", temperature_celsius);
            }
            
        } else {
            ESP_LOGE(TAG, "Failed to read temperature: %d", (int)result);
            return false;
        }
        
        // Test multiple readings for stability
        ESP_LOGI(TAG, "Taking multiple temperature readings...");
        for (int i = 0; i < 5; i++) {
            result = temp_sensor.ReadTemperature(&temperature_celsius);
            if (result == hf_temperature_err_t::TEMPERATURE_SUCCESS) {
                ESP_LOGI(TAG, "   Reading %d: %.2f°C", i + 1, temperature_celsius);
            } else {
                ESP_LOGE(TAG, "Failed to read temperature on attempt %d", i + 1);
                return false;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        ESP_LOGI(TAG, "✅ Temperature Sensor functionality test PASSED");
        return true;
        
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "Exception in Temperature Sensor test: %s", e.what());
        return false;
    } catch (...) {
        ESP_LOGE(TAG, "Unknown exception in Temperature Sensor test");
        return false;
    }
}
