/*
 * command.h
 */

#ifndef __COMMAND_H_
#define __COMMAND_H_

#define COMMAND_MAX_LEN 128
#define COMMAND_DEVICE_MAX_LEN 32

#define COMMAND_PRINT_MAX_LEN 1024

void command_line(char cmd[]);
void command_line_nonprint(char cmd[]);
void command_line_middleware(char cmd[]);
int print_middleware(const char *fmt, ...);
void print_clear();

void command_reset();

/**************** gpio ****************/
void command_parse_gpio(char cmd[], uint16_t pos);
void command_gpio_init(uint8_t pin, uint32_t cfgset);
void command_gpio_deinit(uint8_t pin);
void command_gpio_set(uint8_t pin);
void command_gpio_reset(uint8_t pin);

/**************** i2c0 ****************/
void command_parse_i2c0(char cmd[], uint16_t pos);
void command_i2c0_init(uint8_t scl_pin, uint8_t sda_pin, uint32_t frequency);
void command_i2c0_deinit(uint8_t scl_pin, uint8_t sda_pin);

/**************** uart1 ****************/
void command_parse_uart1(char cmd[], uint16_t pos);
void command_uart1_init(
	uint8_t tx_pin,
	uint8_t rx_pin,
	uint32_t baudrate,
	uint8_t direction,
	uint8_t data_bits,
	uint8_t stop_bits,
	uint8_t parity,
	uint8_t bit_order,
	uint8_t flow_ctrl,
	uint8_t tx_fifo_threshold,
	uint8_t rx_fifo_threshold
);
void command_uart1_deinit(
	uint8_t tx_pin,
	uint8_t rx_pin
);

/**************** pwm0 ****************/
void command_parse_pwm0(char cmd[], uint16_t pos);
void command_pwm0_init(
	uint8_t clk_source,
	uint16_t clk_div,
	uint16_t period
);
void command_pwm0_start(uint8_t ch, uint8_t pin);
void command_pwm0_stop(uint8_t ch, uint8_t pin);
void command_pwm0_threshold(uint8_t ch, uint16_t low_threhold, uint16_t high_threhold);
void command_pwm0_deinit();

/**************** AHT20 ****************/
void command_parse_AHT20(char cmd[], uint16_t pos);
void command_AHT20_init(uint8_t scl_pin, uint8_t sda_pin);
void command_AHT20_deinit();
void command_AHT20_read();

/**************** BMP280 ****************/
void command_parse_BMP280(char cmd[], uint16_t pos);
void command_BMP280_init(uint8_t scl_pin, uint8_t sda_pin);
void command_BMP280_deinit();
void command_BMP280_read();

/**************** DS18B20 ****************/
void command_parse_DS18B20(char cmd[], uint16_t pos);
void command_DS18B20_init(uint8_t pin);
void command_DS18B20_deinit();
void command_DS18B20_read();

/**************** SOIL5 ****************/
void command_parse_SOIL5(char cmd[], uint16_t pos);
void command_SOIL5_init(uint8_t tx_pin, uint8_t rx_pin, uint8_t addr);
void command_SOIL5_deinit();
void command_SOIL5_read();

/**************** SERVO ****************/
void command_parse_SERVO(char cmd[], uint16_t pos);
void command_SERVO_init(
	uint8_t ch,
	uint8_t pin
);
void command_SERVO_deinit();
void command_SERVO_setpos(float pos);
void command_SERVO_getpos();

/**************** MOTOR ****************/
void command_parse_MOTOR_A(char cmd[], uint16_t pos);
void command_MOTOR_A_init(uint8_t pin1, uint8_t pin2);
void command_MOTOR_A_deinit();
void command_MOTOR_A_neutral();
void command_MOTOR_A_positive();
void command_MOTOR_A_negative();
void command_MOTOR_A_brake();

void command_parse_MOTOR_B(char cmd[], uint16_t pos);
void command_MOTOR_B_init(uint8_t pin1, uint8_t pin2);
void command_MOTOR_B_deinit();
void command_MOTOR_B_neutral();
void command_MOTOR_B_positive();
void command_MOTOR_B_negative();
void command_MOTOR_B_brake();

/**************** MAGNET ****************/
void command_parse_MAGNET(char cmd[], uint16_t pos);
void command_MAGNET_init(uint8_t pin);
void command_MAGNET_deinit();
void command_MAGNET_on();
void command_MAGNET_off();

/**************** ALL ****************/
void command_parse_ALL(char cmd[], uint16_t pos);

#endif /* __COMMAND_H_ */
