/**
 * @file test_communication.hpp
 * @brief Communication interfaces testing header
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Test all communication interfaces
 * @return true if all tests pass, false otherwise
 */
bool test_all_communication_interfaces(void);

/**
 * @brief Test SPI communication
 * @return true if test passes, false otherwise
 */
bool test_spi_communication(void);

/**
 * @brief Test I2C communication  
 * @return true if test passes, false otherwise
 */
bool test_i2c_communication(void);

/**
 * @brief Test UART communication
 * @return true if test passes, false otherwise
 */
bool test_uart_communication(void);

/**
 * @brief Test CAN communication
 * @return true if test passes, false otherwise
 */
bool test_can_communication(void);

/**
 * @brief Test WiFi functionality
 * @return true if test passes, false otherwise
 */
bool test_wifi_functionality(void);

#ifdef __cplusplus
}
#endif
