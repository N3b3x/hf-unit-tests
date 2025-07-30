/**
 * @file hf_gpio_config.cpp
 * @brief ESP32-C6 GPIO pin configuration implementation
 * 
 * This file implements the GPIO pin configuration for ESP32-C6 test board
 * with all communication interfaces properly set up.
 */

#include "include/hf_gpio_config.hpp"
#include "driver/gpio.h"
#include "esp_log.h"

static const char* TAG = "GPIO_CONFIG";

bool init_mcu_pinconfig(void) {
    ESP_LOGI(TAG, "Initializing ESP32-C6 GPIO configuration for testing");
    
    bool result = true;
    
    // Configure all pin groups
    result &= configure_digital_io_pins();
    result &= configure_adc_pins();
    result &= configure_spi_pins();
    result &= configure_i2c_pins();
    result &= configure_uart_pins();
    result &= configure_can_pins();
    result &= configure_pwm_pins();
    
    if (result) {
        ESP_LOGI(TAG, "GPIO configuration completed successfully");
    } else {
        ESP_LOGE(TAG, "GPIO configuration failed");
    }
    
    return result;
}

bool configure_adc_pins(void) {
    ESP_LOGI(TAG, "Configuring ADC pins (GPIO0-6)");
    
    // ADC pins are automatically configured when ADC is initialized
    // Just ensure they are not configured as outputs
    gpio_config_t adc_config = {
        .pin_bit_mask = (1ULL << HF_TEST_ADC_CHANNEL_0_PIN) |
                       (1ULL << HF_TEST_ADC_CHANNEL_1_PIN) |
                       (1ULL << HF_TEST_ADC_CHANNEL_2_PIN) |
                       (1ULL << HF_TEST_ADC_CHANNEL_3_PIN) |
                       (1ULL << HF_TEST_ADC_CHANNEL_4_PIN) |
                       (1ULL << HF_TEST_ADC_CHANNEL_5_PIN) |
                       (1ULL << HF_TEST_ADC_CHANNEL_6_PIN),
        .mode = GPIO_MODE_DISABLE,  // Will be configured by ADC driver
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    esp_err_t ret = gpio_config(&adc_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC pins: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "ADC pins configured successfully");
    return true;
}

bool configure_spi_pins(void) {
    ESP_LOGI(TAG, "Configuring SPI pins");
    
    // SPI pins will be configured by the SPI driver
    // Just ensure they are available and not conflicting
    gpio_config_t spi_config = {
        .pin_bit_mask = (1ULL << HF_TEST_SPI_CS_PIN),  // Only CS needs manual config
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,  // CS should be pulled up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    esp_err_t ret = gpio_config(&spi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure SPI CS pin: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Set CS high initially
    gpio_set_level(HF_TEST_SPI_CS_PIN, 1);
    
    ESP_LOGI(TAG, "SPI pins configured successfully");
    return true;
}

bool configure_i2c_pins(void) {
    ESP_LOGI(TAG, "Configuring I2C pins");
    
    // I2C pins will be configured by the I2C driver
    // Just log the pin assignments
    ESP_LOGI(TAG, "I2C SDA: GPIO%d, SCL: GPIO%d", HF_TEST_I2C_SDA_PIN, HF_TEST_I2C_SCL_PIN);
    return true;
}

bool configure_uart_pins(void) {
    ESP_LOGI(TAG, "Configuring UART pins");
    
    // UART pins will be configured by the UART driver
    // Just log the pin assignments  
    ESP_LOGI(TAG, "UART TX: GPIO%d, RX: GPIO%d", HF_TEST_UART_TX_PIN, HF_TEST_UART_RX_PIN);
    return true;
}

bool configure_can_pins(void) {
    ESP_LOGI(TAG, "Configuring CAN pins");
    
    // CAN pins will be configured by the TWAI driver
    // Just log the pin assignments
    ESP_LOGI(TAG, "CAN TX: GPIO%d, RX: GPIO%d", HF_TEST_CAN_TX_PIN, HF_TEST_CAN_RX_PIN);
    return true;
}

bool configure_pwm_pins(void) {
    ESP_LOGI(TAG, "Configuring PWM pins");
    
    // PWM pins will be configured by the LEDC driver
    // Just log the pin assignments
    ESP_LOGI(TAG, "PWM pins: GPIO%d, GPIO%d, GPIO%d, GPIO%d", 
             HF_TEST_PWM_CHANNEL_0_PIN, HF_TEST_PWM_CHANNEL_1_PIN,
             HF_TEST_PWM_CHANNEL_2_PIN, HF_TEST_PWM_CHANNEL_3_PIN);
    return true;
}

bool configure_digital_io_pins(void) {
    ESP_LOGI(TAG, "Configuring digital I/O pins");
    
    // Configure LED pin as output
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << HF_TEST_LED_BUILTIN_PIN) |
                       (1ULL << HF_TEST_GPIO_OUTPUT_0) |
                       (1ULL << HF_TEST_GPIO_OUTPUT_1),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    esp_err_t ret = gpio_config(&led_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure output pins: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Configure button and input pins
    gpio_config_t input_config = {
        .pin_bit_mask = (1ULL << HF_TEST_BUTTON_PIN) |
                       (1ULL << HF_TEST_GPIO_INPUT_0) |
                       (1ULL << HF_TEST_GPIO_INPUT_1) |
                       (1ULL << HF_TEST_INT_PIN_0) |
                       (1ULL << HF_TEST_INT_PIN_1),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    ret = gpio_config(&input_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure input pins: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Initialize outputs to known state
    gpio_set_level(HF_TEST_LED_BUILTIN_PIN, 0);
    gpio_set_level(HF_TEST_GPIO_OUTPUT_0, 0);
    gpio_set_level(HF_TEST_GPIO_OUTPUT_1, 0);
    
    ESP_LOGI(TAG, "Digital I/O pins configured successfully");
    return true;
}
