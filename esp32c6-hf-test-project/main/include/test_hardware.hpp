/**
 * @file test_hardware.hpp
 * @brief Hardware peripherals testing header
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Test all hardware peripherals
 * @return true if all tests pass, false otherwise
 */
bool test_all_hardware_peripherals(void);

/**
 * @brief Test ADC functionality
 * @return true if test passes, false otherwise
 */
bool test_adc_functionality(void);

/**
 * @brief Test GPIO functionality
 * @return true if test passes, false otherwise
 */
bool test_gpio_functionality(void);

/**
 * @brief Test PWM functionality
 * @return true if test passes, false otherwise
 */
bool test_pwm_functionality(void);

/**
 * @brief Test NVS functionality
 * @return true if test passes, false otherwise
 */
bool test_nvs_functionality(void);

/**
 * @brief Test Timer functionality
 * @return true if test passes, false otherwise
 */
bool test_timer_functionality(void);

/**
 * @brief Test temperature sensor functionality
 * @return true if test passes, false otherwise
 */
bool test_temperature_sensor(void);

#ifdef __cplusplus
}
#endif
