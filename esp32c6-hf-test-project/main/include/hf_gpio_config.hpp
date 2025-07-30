/**
 * @file hf_gpio_config.hpp
 * @brief ESP32-C6 GPIO pin configuration for HardFOC testing
 * 
 * This file defines the GPIO pin mapping for ESP32-C6 test board
 * with all communication interfaces properly configured.
 * 
 * @author Test System
 * @date 2025
 */

#pragma once

#include "driver/gpio.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ESP32-C6 GPIO Pin Definitions for Testing
 * 
 * Pin mapping optimized for ESP32-C6 DevKit and communication testing:
 * - GPIO0-6: ADC capable pins
 * - GPIO7-8: Available for digital I/O  
 * - GPIO9-11: SPI interface
 * - GPIO12-13: I2C interface
 * - GPIO14-15: UART interface
 * - GPIO16-17: CAN interface
 * - GPIO18-21: PWM/LEDC outputs
 * - GPIO22-23: Digital I/O with interrupt capability
 */

//==============================================================================
// ADC PIN CONFIGURATION (GPIO0-6 are ADC capable on ESP32-C6)
//==============================================================================
#define HF_TEST_ADC_CHANNEL_0_PIN     GPIO_NUM_0   ///< ADC1_CH0
#define HF_TEST_ADC_CHANNEL_1_PIN     GPIO_NUM_1   ///< ADC1_CH1  
#define HF_TEST_ADC_CHANNEL_2_PIN     GPIO_NUM_2   ///< ADC1_CH2
#define HF_TEST_ADC_CHANNEL_3_PIN     GPIO_NUM_3   ///< ADC1_CH3
#define HF_TEST_ADC_CHANNEL_4_PIN     GPIO_NUM_4   ///< ADC1_CH4
#define HF_TEST_ADC_CHANNEL_5_PIN     GPIO_NUM_5   ///< ADC1_CH5
#define HF_TEST_ADC_CHANNEL_6_PIN     GPIO_NUM_6   ///< ADC1_CH6

//==============================================================================
// DIGITAL I/O CONFIGURATION
//==============================================================================
#define HF_TEST_LED_BUILTIN_PIN       GPIO_NUM_8   ///< On-board LED
#define HF_TEST_BUTTON_PIN            GPIO_NUM_9   ///< Test button (with pullup)

//==============================================================================
// SPI INTERFACE CONFIGURATION (SPI2)
//==============================================================================
#define HF_TEST_SPI_MOSI_PIN          GPIO_NUM_11  ///< SPI2 MOSI
#define HF_TEST_SPI_MISO_PIN          GPIO_NUM_13  ///< SPI2 MISO  
#define HF_TEST_SPI_SCLK_PIN          GPIO_NUM_12  ///< SPI2 SCLK
#define HF_TEST_SPI_CS_PIN            GPIO_NUM_10  ///< SPI2 CS

//==============================================================================
// I2C INTERFACE CONFIGURATION (I2C0)
//==============================================================================
#define HF_TEST_I2C_SDA_PIN           GPIO_NUM_15  ///< I2C0 SDA
#define HF_TEST_I2C_SCL_PIN           GPIO_NUM_14  ///< I2C0 SCL

//==============================================================================
// UART INTERFACE CONFIGURATION (UART1)
//==============================================================================
#define HF_TEST_UART_TX_PIN           GPIO_NUM_16  ///< UART1 TX
#define HF_TEST_UART_RX_PIN           GPIO_NUM_17  ///< UART1 RX
#define HF_TEST_UART_RTS_PIN          GPIO_NUM_18  ///< UART1 RTS (optional)
#define HF_TEST_UART_CTS_PIN          GPIO_NUM_19  ///< UART1 CTS (optional)

//==============================================================================
// CAN/TWAI INTERFACE CONFIGURATION (TWAI0)
//==============================================================================
#define HF_TEST_CAN_TX_PIN            GPIO_NUM_20  ///< TWAI0 TX
#define HF_TEST_CAN_RX_PIN            GPIO_NUM_21  ///< TWAI0 RX

//==============================================================================
// PWM/LEDC CONFIGURATION
//==============================================================================
#define HF_TEST_PWM_CHANNEL_0_PIN     GPIO_NUM_22  ///< LEDC Channel 0
#define HF_TEST_PWM_CHANNEL_1_PIN     GPIO_NUM_23  ///< LEDC Channel 1
#define HF_TEST_PWM_CHANNEL_2_PIN     GPIO_NUM_7   ///< LEDC Channel 2
#define HF_TEST_PWM_CHANNEL_3_PIN     GPIO_NUM_8   ///< LEDC Channel 3 (shared with LED)

//==============================================================================
// ADDITIONAL DIGITAL I/O
//==============================================================================
#define HF_TEST_GPIO_OUTPUT_0         GPIO_NUM_24  ///< General purpose output 0
#define HF_TEST_GPIO_OUTPUT_1         GPIO_NUM_25  ///< General purpose output 1
#define HF_TEST_GPIO_INPUT_0          GPIO_NUM_26  ///< General purpose input 0
#define HF_TEST_GPIO_INPUT_1          GPIO_NUM_27  ///< General purpose input 1

//==============================================================================
// INTERRUPT CAPABLE PINS  
//==============================================================================
#define HF_TEST_INT_PIN_0             GPIO_NUM_28  ///< Interrupt input 0
#define HF_TEST_INT_PIN_1             GPIO_NUM_29  ///< Interrupt input 1

//==============================================================================
// SPECIAL FUNCTION PINS (Use with caution)
//==============================================================================
#define HF_TEST_BOOT_PIN              GPIO_NUM_30  ///< Boot button (GPIO30)

//==============================================================================
// FUNCTION DECLARATIONS
//==============================================================================

/**
 * @brief Initialize all GPIO pins for testing
 * @return true on success, false on failure
 */
bool init_mcu_pinconfig(void);

/**
 * @brief Configure ADC pins 
 * @return true on success, false on failure
 */
bool configure_adc_pins(void);

/**
 * @brief Configure SPI pins
 * @return true on success, false on failure  
 */
bool configure_spi_pins(void);

/**
 * @brief Configure I2C pins
 * @return true on success, false on failure
 */
bool configure_i2c_pins(void);

/**
 * @brief Configure UART pins
 * @return true on success, false on failure
 */
bool configure_uart_pins(void);

/**
 * @brief Configure CAN pins
 * @return true on success, false on failure
 */
bool configure_can_pins(void);

/**
 * @brief Configure PWM pins
 * @return true on success, false on failure
 */
bool configure_pwm_pins(void);

/**
 * @brief Configure general purpose digital I/O pins
 * @return true on success, false on failure
 */
bool configure_digital_io_pins(void);

#ifdef __cplusplus
}
#endif
