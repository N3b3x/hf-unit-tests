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
        hf_gpio_err_t result = gpio.SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
        if (result != hf_gpio_err_t::GPIO_SUCCESS) {
            ESP_LOGE(TAG, "Failed to set LED pin direction");
            return false;
        }
        
        // Test LED blinking
        ESP_LOGI(TAG, "Testing LED blinking...");
        for (int i = 0; i < 5; i++) {
            gpio.SetState(hf_gpio_state_t::HF_GPIO_STATE_ACTIVE);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio.SetState(hf_gpio_state_t::HF_GPIO_STATE_INACTIVE);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        
        ESP_LOGI(TAG, "✅ GPIO digital output test PASSED");
        
        // Test digital input
        result = gpio.SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
        gpio.SetPullMode(hf_gpio_pull_mode_t::HF_GPIO_PULL_UP);
        if (result != hf_gpio_err_t::GPIO_SUCCESS) {
            ESP_LOGE(TAG, "Failed to set button pin direction");
            return false;
        }
        
        hf_gpio_state_t button_state = gpio.GetCurrentState();
        ESP_LOGI(TAG, "✅ GPIO digital input test PASSED - Button state: %s", 
                 (button_state == hf_gpio_state_t::HF_GPIO_STATE_ACTIVE) ? "ACTIVE" : "INACTIVE");
        
        // Test multiple output pins
        gpio.SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
        gpio.SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
        
        ESP_LOGI(TAG, "Testing multiple GPIO outputs...");
        gpio.SetState(hf_gpio_state_t::HF_GPIO_STATE_ACTIVE);
        gpio.SetState(hf_gpio_state_t::HF_GPIO_STATE_INACTIVE);
        vTaskDelay(pdMS_TO_TICKS(500));
        
        gpio.SetState(hf_gpio_state_t::HF_GPIO_STATE_INACTIVE);
        gpio.SetState(hf_gpio_state_t::HF_GPIO_STATE_ACTIVE);
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // Reset outputs
        gpio.SetState(hf_gpio_state_t::HF_GPIO_STATE_INACTIVE);
        gpio.SetState(hf_gpio_state_t::HF_GPIO_STATE_INACTIVE);
        
        ESP_LOGI(TAG, "✅ GPIO functionality test PASSED");
        return true;
}
    }
}

bool test_adc_functionality(void) {
    ESP_LOGI(TAG, "Testing ADC functionality...");
    
    // Create ADC configuration
    hf_adc_unit_config_t adc_config = {};
    adc_config.unit_id = 1;  // ADC unit 1
    adc_config.mode = hf_adc_mode_t::ONESHOT;
    adc_config.bit_width = hf_adc_bitwidth_t::WIDTH_DEFAULT;
    
    // Create ADC instance with configuration
    EspAdc adc(adc_config);
    if (!adc.EnsureInitialized()) {
        ESP_LOGE(TAG, "Failed to initialize ADC");
        return false;
    }
    
    ESP_LOGI(TAG, "✅ ADC initialization PASSED");
    
    // Configure ADC channels
    hf_adc_channel_config_t adc_channel_configs[] = {
        {0, hf_adc_atten_t::ATTEN_DB_12, hf_adc_bitwidth_t::WIDTH_DEFAULT, true},
        {1, hf_adc_atten_t::ATTEN_DB_12, hf_adc_bitwidth_t::WIDTH_DEFAULT, true},
        {2, hf_adc_atten_t::ATTEN_DB_12, hf_adc_bitwidth_t::WIDTH_DEFAULT, true},
    };
    
    for (const auto& config : adc_channel_configs) {
        hf_adc_err_t result = adc.ConfigureChannel(config.channel_id, config.attenuation, config.bitwidth);
        if (result != hf_adc_err_t::ADC_SUCCESS) {
            ESP_LOGE(TAG, "Failed to configure ADC channel %d", (int)config.channel_id);
            return false;
        }
    }
    
    ESP_LOGI(TAG, "✅ ADC channel configuration PASSED");
    
    // Test ADC readings from all configured channels
    ESP_LOGI(TAG, "Testing ADC channel readings...");
    for (const auto& config : adc_channel_configs) {
        hf_u32_t raw_value;
        hf_u32_t voltage_mv;
        
        hf_adc_err_t result = adc.ReadSingleRaw(config.channel_id, raw_value);
        if (result == hf_adc_err_t::ADC_SUCCESS) {
            result = adc.RawToVoltage(raw_value, config.attenuation, voltage_mv);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                ESP_LOGI(TAG, "✅ Channel %d: Raw=%lu, Voltage=%lu mV", 
                         (int)config.channel_id, (unsigned long)raw_value, (unsigned long)voltage_mv);
            } else {
                ESP_LOGE(TAG, "Failed to convert raw to voltage from channel %d", (int)config.channel_id);
                return false;
            }
        } else {
            ESP_LOGE(TAG, "Failed to read raw value from channel %d", (int)config.channel_id);
            return false;
        }
    }
    
    ESP_LOGI(TAG, "✅ ADC functionality test PASSED");
    return true;
}

bool test_pwm_functionality(void) {
    ESP_LOGI(TAG, "Testing PWM functionality...");
    
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
        hf_channel_id_t channel_id;
        hf_pin_num_t pin;
        const char* name;
    } pwm_channels[] = {
        {0, HF_TEST_PWM_CHANNEL_0_PIN, "Channel 0"},
        {1, HF_TEST_PWM_CHANNEL_1_PIN, "Channel 1"},
        {2, HF_TEST_PWM_CHANNEL_2_PIN, "Channel 2"},
    };
    
    for (const auto& ch : pwm_channels) {
        hf_pwm_channel_config_t channel_config = {};
        channel_config.gpio_pin = static_cast<hf_gpio_num_t>(ch.pin);
        channel_config.channel_id = ch.channel_id;
        channel_config.timer_id = 0;
            channel_config.speed_mode = hf_pwm_mode_t::HF_PWM_MODE_BASIC;
            channel_config.duty_initial = 0;       // Start at 0%
            channel_config.intr_type = hf_pwm_intr_type_t::HF_PWM_INTR_DISABLE;
            channel_config.hpoint = 0;
            
            hf_pwm_err_t result = pwm.ConfigureChannel(ch.channel_id, channel_config);
            if (result != hf_pwm_err_t::PWM_SUCCESS) {
                ESP_LOGE(TAG, "Failed to configure PWM %s", ch.name);
                return false;
            }
        }
        
        ESP_LOGI(TAG, "✅ PWM channel configuration PASSED");
        
        // Start all PWM channels
        hf_pwm_err_t result = pwm.StartAll();
        if (result != hf_pwm_err_t::PWM_SUCCESS) {
            ESP_LOGE(TAG, "Failed to start PWM channels");
            return false;
        }
        
        // Test PWM output with varying duty cycles
        ESP_LOGI(TAG, "Testing PWM duty cycle sweep...");
        for (const auto& ch : pwm_channels) {
            ESP_LOGI(TAG, "   Testing %s (GPIO%d)", ch.name, ch.pin);
            
            // Sweep duty cycle from 0% to 100% and back
            for (float duty = 0.0f; duty <= 1.0f; duty += 0.1f) {
                pwm.SetDutyCycle(ch.channel_id, duty);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            
            for (float duty = 1.0f; duty >= 0.0f; duty -= 0.1f) {
                pwm.SetDutyCycle(ch.channel_id, duty);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        
        // Stop all PWM channels
        pwm.StopAll();
        
        ESP_LOGI(TAG, "✅ PWM functionality test PASSED");
        return true;
}
}

bool test_nvs_functionality(void) {
    ESP_LOGI(TAG, "Testing NVS functionality...");
    
    // Create NVS instance with namespace
    EspNvs nvs("test_storage");
    if (!nvs.EnsureInitialized()) {
        ESP_LOGE(TAG, "Failed to initialize NVS");
        return false;
    }
    
    ESP_LOGI(TAG, "✅ NVS initialization PASSED");
    
    // Test different data types
    const char* test_key_str = "test_string";
    const char* test_value_str = "ESP32-C6 Test Value";
    const char* test_key_int = "test_u32";
    hf_u32_t test_value_int = 12345;
    const char* test_key_blob = "test_blob";
    uint8_t test_value_blob[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    
    // Test string storage
    hf_nvs_err_t result = nvs.SetString(test_key_str, test_value_str);
    if (result != hf_nvs_err_t::NVS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to store string value");
        return false;
    }
        
        char retrieved_str[100] = {0};
        size_t str_len = sizeof(retrieved_str);
        result = nvs.GetString(test_key_str, retrieved_str, str_len);
        if (result != hf_nvs_err_t::NVS_SUCCESS || strcmp(retrieved_str, test_value_str) != 0) {
            ESP_LOGE(TAG, "Failed to retrieve string value correctly");
            return false;
        }
        
        ESP_LOGI(TAG, "✅ NVS string storage test PASSED: '%s' = '%s'", 
                 test_key_str, retrieved_str);
        
        // Test U32 storage
        result = nvs.SetU32(test_key_int, test_value_int);
        if (result != hf_nvs_err_t::NVS_SUCCESS) {
            ESP_LOGE(TAG, "Failed to store U32 value");
            return false;
        }
        
        hf_u32_t retrieved_int;
        result = nvs.GetU32(test_key_int, retrieved_int);
        if (result != hf_nvs_err_t::NVS_SUCCESS || retrieved_int != test_value_int) {
            ESP_LOGE(TAG, "Failed to retrieve U32 value correctly");
            return false;
        }
        
        ESP_LOGI(TAG, "✅ NVS U32 storage test PASSED: '%s' = %u", 
                 test_key_int, retrieved_int);
        
        // Test blob storage
        result = nvs.SetBlob(test_key_blob, test_value_blob, sizeof(test_value_blob));
        if (result != hf_nvs_err_t::NVS_SUCCESS) {
            ESP_LOGE(TAG, "Failed to store blob value");
            return false;
        }
        
        uint8_t retrieved_blob[sizeof(test_value_blob)];
        size_t blob_size = sizeof(retrieved_blob);
        result = nvs.GetBlob(test_key_blob, retrieved_blob, blob_size, &blob_size);
        if (result != hf_nvs_err_t::NVS_SUCCESS || 
            blob_size != sizeof(test_value_blob) ||
            memcmp(retrieved_blob, test_value_blob, blob_size) != 0) {
            ESP_LOGE(TAG, "Failed to retrieve blob value correctly");
            return false;
        }
        
        ESP_LOGI(TAG, "✅ NVS blob storage test PASSED: '%s' = [%zu bytes]", 
                 test_key_blob, blob_size);
        
        // Test key deletion
        result = nvs.EraseKey(test_key_str);
        if (result != hf_nvs_err_t::NVS_SUCCESS) {
            ESP_LOGE(TAG, "Failed to delete key");
            return false;
        }
        
        // Verify key was deleted by trying to retrieve it
        char verify_str[100] = {0};
        size_t verify_len = sizeof(verify_str);
        result = nvs.GetString(test_key_str, verify_str, verify_len);
        if (result != hf_nvs_err_t::NVS_ERR_KEY_NOT_FOUND) {
            ESP_LOGE(TAG, "Key was not properly deleted");
            return false;
        }
        
        ESP_LOGI(TAG, "✅ NVS functionality test PASSED");
        return true;
        
    }
        ESP_LOGE(TAG, "Unknown exception in NVS test");
        return false;
    }
}

bool test_timer_functionality(void) {
    ESP_LOGI(TAG, "Testing Timer functionality...");
    
        ESP_LOGI(TAG, "🔧 Testing Timer functionality");
        
        // Create timer instance
        EspPeriodicTimer timer;
        if (!timer.IsInitialized()) {
            ESP_LOGE(TAG, "Failed to initialize Timer");
            return false;
        }
        
        ESP_LOGI(TAG, "✅ Timer initialization PASSED");
        
        // Test timer start/stop functionality with 1 second period
        hf_timer_err_t result = timer.Start(1000000); // 1 second in microseconds
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
        
        // Get timer statistics
        hf_u64_t callback_count, missed_callbacks;
        hf_timer_err_t last_error;
        hf_timer_err_t stat_result = timer.GetStats(callback_count, missed_callbacks, last_error);
        if (stat_result == hf_timer_err_t::TIMER_SUCCESS) {
            ESP_LOGI(TAG, "Timer stats - Callbacks: %llu, Missed: %llu", 
                     static_cast<unsigned long long>(callback_count),
                     static_cast<unsigned long long>(missed_callbacks));
        }
        
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
        
    }
        ESP_LOGE(TAG, "Unknown exception in Timer test");
        return false;
    }
}

bool test_temperature_sensor(void) {
    ESP_LOGI(TAG, "Testing Temperature Sensor functionality...");
    
        ESP_LOGI(TAG, "🔧 Testing Temperature Sensor functionality");
        
        // Create temperature sensor instance  
        EspTemperature temp_sensor;
        if (!temp_sensor.IsInitialized()) {
            ESP_LOGE(TAG, "Failed to initialize Temperature Sensor");
            return false;
        }
        
        ESP_LOGI(TAG, "✅ Temperature Sensor initialization PASSED");
        
        // Test temperature reading
        float temperature_celsius;
        hf_temp_err_t result = temp_sensor.ReadTemperatureCelsius(&temperature_celsius);
        
        if (result == hf_temp_err_t::TEMP_SUCCESS) {
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
            result = temp_sensor.ReadTemperatureCelsius(&temperature_celsius);
            if (result == hf_temp_err_t::TEMP_SUCCESS) {
                ESP_LOGI(TAG, "   Reading %d: %.2f°C", i + 1, temperature_celsius);
            } else {
                ESP_LOGE(TAG, "Failed to read temperature on attempt %d", i + 1);
                return false;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        ESP_LOGI(TAG, "✅ Temperature Sensor functionality test PASSED");
        return true;
        
    }
        ESP_LOGE(TAG, "Unknown exception in Temperature Sensor test");
        return false;
    }
}
