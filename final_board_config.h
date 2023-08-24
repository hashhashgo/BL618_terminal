/*
 * final_board_config.h
 *
 *      Author: yu_ha
 */

#include "bflb_uart.h"
#include "bflb_gpio.h"
#include "bflb_i2c.h"
#include "bflb_pwm_v2.h"
#include "bflb_clock.h"
#include "board.h"

#ifndef EXAMPLES_FINAL_FINAL_BOARD_CONFIG_H_
#define EXAMPLES_FINAL_FINAL_BOARD_CONFIG_H_

#define I2C0_SCL_PIN 18
#define I2C0_SDA_PIN 19

#define UART1_TX_PIN 23
#define UART1_RX_PIN 24

#define PWM0_CH2P_PIN 30
#define PWM0_CH3P_PIN 31

#define AHT20_ENABLE	1
#define BMP280_ENABLE	1
#define SOIL_ENABLE		1
#define MOTOR_ENABLE	1
#define DS18B20_ENABLE	1
#define SERVO_ENABLE	1

#define _NAME_CONNECT(pre, x) pre##x
#define NAME_CONNECT(pre, x) _NAME_CONNECT(pre, x)

#if (AHT20_ENABLE)
#define I2C0_ENABLE
#define AHT20_I2C			I2C0
#define AHT20_SCL_PIN		I2C0_SCL_PIN
#define AHT20_SDA_PIN		I2C0_SDA_PIN
#endif

#if (BMP280_ENABLE)
#define I2C0_ENABLE
#define BMP280_I2C			I2C0
#define BMP280_SCL_PIN 		I2C0_SCL_PIN
#define BMP280_SDA_PIN 		I2C0_SDA_PIN
#endif

#if (SOIL_ENABLE)
#define UART1_ENABLE
#define UART1_BAUDRATE		(4800)
#define UART1_DIRECTION		UART_DIRECTION_TXRX
#define UART1_DATA_BITS		UART_DATA_BITS_8
#define UART1_STOP_BITS		UART_STOP_BITS_1
#define UART1_PARITY		UART_PARITY_NONE
#define UART1_BIT_ORDER		UART_LSB_FIRST
#define UART1_FLOW_CTRL		(0)
#define UART1_TX_FIFO_TH	(7)
#define UART1_RX_FIFO_TH	(7)
#define SOIL_UART			UART1
#define SOIL_UART_TX_PIN	UART1_TX_PIN
#define SOIL_UART_RX_PIN	UART1_RX_PIN
#endif

#if (MOTOR_ENABLE)
#define MOTOR_AIN1_PIN 25
#define MOTOR_AIN2_PIN 26
#define MOTOR_BIN1_PIN 27
#define MOTOR_BIN2_PIN 28
#endif

#if (DS18B20_ENABLE)
#define DS18B20_PIN 30
#endif

#if (SERVO_ENABLE)
#define PWM0_ENABLE
#define SERVO_UP_CHANNEL		PWM_CH2
#define SERVO_UP_PIN			PWM0_CH2P_PIN
#define SERVO_UP_MIN_THREHOLD	(500)
#define SERVO_UP_MAX_THREHOLD	(2500)
#define SERVO_UP_MIN_POS		(0.0)
#define SERVO_UP_MAX_POS		(180.0)
#define SERVO_DOWN_CHANNEL		PWM_CH3
#define SERVO_DOWN_PIN			PWM0_CH3P_PIN
#define SERVO_DOWN_MIN_THREHOLD	(500)
#define SERVO_DOWN_MAX_THREHOLD	(2500)
#define SERVO_DOWN_MIN_POS		(0.0)
#define SERVO_DOWN_MAX_POS		(180.0)
#endif

void final_board_init();
void final_board_gpio_init();
void final_board_i2c0_init();
void final_board_uart1_init();
void final_board_pwm0_init();

#endif /* EXAMPLES_FINAL_FINAL_BOARD_CONFIG_H_ */
