/*
 * command.c
 */

#include "AHT20.h"
#include "BMP280.h"
#include "DS18B20.h"
#include "SOIL5.h"
#include "gimbal.h"

#include "motor.h"

#include "final_board_config.h"

#include "command.h"
#include "stdarg.h"

extern struct bflb_device_s *gpio;
extern struct bflb_device_s *i2c0;
extern struct bflb_device_s *uart1;
extern struct bflb_device_s *pwm0;

struct AHT20_handle_s AHT20_0;
struct BMP280_handle_s BMP280_0;
struct DS18B20_handle_s DS18B20_0;
struct MOTOR_handle_s MOTOR_A, MOTOR_B;
struct SOIL5_handle_s SOIL5_0;
//struct GIMBAL_handle_s GIMBAL_0;
struct SERVO_handle_s SERVO_DOWN;

extern struct bflb_device_s *console;

char print_lines[COMMAND_PRINT_MAX_LEN] = {0};
uint16_t print_lines_pos = 0;

void command_line(char cmd[]) {
	print_clear();
	command_line_middleware(cmd);
	puts(print_lines);
}

void command_line_nonprint(char cmd[]) {
	print_clear();
	command_line_middleware(cmd);
}

void command_line_middleware(char cmd[]) {
	if (strncmp(cmd, "reset", 5) == 0) {
		command_reset();
	}
	else if (strncmp(cmd, "gpio", 4) == 0){
		command_parse_gpio(cmd, 4);
	}
	else if (strncmp(cmd, "i2c0", 4) == 0) {
		command_parse_i2c0(cmd, 4);
	}
	else if (strncmp(cmd, "uart1", 5) == 0) {
		command_parse_uart1(cmd, 5);
	}
	else if (strncmp(cmd, "pwm0", 4) == 0) {
		command_parse_pwm0(cmd, 4);
	}
	else if (strncmp(cmd, "AHT20", 5) == 0) {
		command_parse_AHT20(cmd, 5);
	}
	else if (strncmp(cmd, "BMP280", 6) == 0) {
		command_parse_BMP280(cmd, 6);
	}
	else if (strncmp(cmd, "DS18B20", 7) == 0) {
		command_parse_DS18B20(cmd, 7);
	}
	else if (strncmp(cmd, "SOIL5", 5) == 0) {
		command_parse_SOIL5(cmd, 5);
	}
	else if (strncmp(cmd, "SERVO", 5) == 0) {
		command_parse_SERVO(cmd, 5);
	}
	else if (strncmp(cmd, "MOTOR A", 7) == 0) {
		command_parse_MOTOR_A(cmd, 7);
	}
	else if (strncmp(cmd, "MOTOR B", 7) == 0) {
		command_parse_MOTOR_B(cmd, 7);
	}
	else if (strncmp(cmd, "MAGNET", 6) == 0) {
		command_parse_MAGNET(cmd, 6);
	}
	else if (strncmp(cmd, "ALL", 3) == 0) {
		command_parse_ALL(cmd, 3);
	}
	else {
		print_middleware("[F4U] Command not found!:\r\n");
		print_middleware("%s\r\n", cmd);
	}
}

int print_middleware(const char *fmt, ...) {
    int len;
    va_list ap;

    if (console == NULL) {
        return 0;
    }

    va_start(ap, fmt);
	len = vsnprintf(print_lines + print_lines_pos, COMMAND_PRINT_MAX_LEN, fmt, ap);
	print_lines_pos += len;
	print_lines[print_lines_pos] = 0;
    va_end(ap);

    return len;
}

void print_clear() {
	print_lines_pos = 0;
}

void command_reset() {
	final_board_init();

	bflb_gpio_init(gpio, GPIO_PIN_20, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
	bflb_gpio_init(gpio, GPIO_PIN_32, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
	bflb_gpio_reset(gpio, GPIO_PIN_32);
	bflb_gpio_init(gpio, GPIO_PIN_3, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
	bflb_gpio_set(gpio, GPIO_PIN_3);
	bflb_gpio_init(gpio, GPIO_PIN_34, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
	bflb_gpio_reset(gpio, GPIO_PIN_34);

	AHT20_init(&AHT20_0, i2c0, 0x38, "AHT20_0");
	BMP280_init(&BMP280_0, i2c0, 0x77, "BMP280_0");
//	DS18B20_init(&DS18B20_0, DS18B20_PIN, "DS18B20_0");
	SERVO_init(
		&SERVO_DOWN,
		SERVO_DOWN_CHANNEL,
		SERVO_DOWN_MIN_POS,
		SERVO_DOWN_MAX_POS,
		(SERVO_DOWN_MAX_POS - SERVO_DOWN_MIN_POS) / 2,
		SERVO_DOWN_MIN_THREHOLD,
		SERVO_DOWN_MAX_THREHOLD,
		"SERVO_DOWN"
	);
	SOIL5_init(&SOIL5_0, uart1, 0x01, "SOIL5_0");

	MOTOR_init(&MOTOR_A, MOTOR_AIN1_PIN, MOTOR_AIN2_PIN, "MOTOR_A");
	MOTOR_init(&MOTOR_B, MOTOR_BIN1_PIN, MOTOR_BIN2_PIN, "MOTOR_B");

	print_middleware("[F4U] terminal has reset\r\n");
}

/**************** gpio ****************/
static char command_gpio_usage[] =
	"[F4U] Usage:\r\n"
	"[F4U] gpio init\r\n"
	"[F4U] gpio deinit\r\n"
	"[F4U] gpio set\r\n"
	"[F4U] gpio reset\r\n";

void command_parse_gpio(char cmd[], uint16_t pos) {
	uint8_t param_num = 0;
	while (cmd[pos] == ' ') {++pos;}
	if (cmd[pos] == 0) {print_middleware(command_gpio_usage);return;}
	if (strncmp(cmd + pos, "init", 4) == 0) {
		uint8_t pin;
		uint32_t cfgset;
		param_num = sscanf(cmd + pos + 4, "%d%d", &pin, &cfgset);
		if (param_num != 2) {print_middleware(command_gpio_usage);return;}
		command_gpio_init(pin ,cfgset);
	}
	else if (strncmp(cmd + pos, "deinit", 6) == 0) {
		uint8_t pin;
		param_num = sscanf(cmd + pos + 6, "%d", &pin);
		if (param_num != 1) {print_middleware(command_gpio_usage);return;}
		command_gpio_deinit(pin);
	}
	else if (strncmp(cmd + pos, "set", 3) == 0) {
		uint8_t pin;
		param_num = sscanf(cmd + pos + 3, "%d", &pin);
		if (param_num != 1) {print_middleware(command_gpio_usage);return;}
		command_gpio_set(pin);
	}
	else if (strncmp(cmd + pos, "reset", 5) == 0) {
		uint8_t pin;
		param_num = sscanf(cmd + pos + 5, "%d", &pin);
		if (param_num != 1) {print_middleware(command_gpio_usage);return;}
		command_gpio_reset(pin);
	}
	else {
		print_middleware(command_gpio_usage);return;
	}
}

void command_gpio_init(uint8_t pin, uint32_t cfgset) {
	gpio = bflb_device_get_by_name("gpio");
	bflb_gpio_init(gpio, pin, cfgset);
	print_middleware("[F4U] gpio %u init cfg: %u\r\n", pin, cfgset);
}

void command_gpio_deinit(uint8_t pin) {
	bflb_gpio_deinit(gpio, pin);
	print_middleware("[F4U] gpio %u deinit\r\n", pin);
}

void command_gpio_set(uint8_t pin) {
	bflb_gpio_set(gpio, pin);
	print_middleware("[F4U] gpio %u HIGH\r\n", pin);
}

void command_gpio_reset(uint8_t pin){
	bflb_gpio_reset(gpio, pin);
	print_middleware("[F4U] gpio %u LOW\r\n", pin);
}

/**************** i2c0 ****************/
static char command_i2c0_usage[] =
	"[F4U] Usage:\r\n"
	"[F4U] i2c0 init\r\n"
	"[F4U] i2c0 deinit\r\n";

void command_parse_i2c0(char cmd[], uint16_t pos) {
	uint8_t param_num = 0;
	while (cmd[pos] == ' ') {++pos;}
	if (cmd[pos] == 0) {print_middleware(command_i2c0_usage);return;}
	if (strncmp(cmd + pos, "init", 4) == 0) {
		uint8_t scl_pin; uint8_t sda_pin; uint32_t frequency;
		param_num = sscanf(cmd + pos + 4, "%d%d%d", &scl_pin, &sda_pin, &frequency);
		if (param_num != 3) {print_middleware(command_i2c0_usage);return;}
		command_i2c0_init(scl_pin, sda_pin, frequency);
	}
	else if (strncmp(cmd + pos, "deinit", 6) == 0) {
		uint8_t scl_pin; uint8_t sda_pin;
		param_num = sscanf(cmd + pos + 6, "%d%d", &scl_pin, &sda_pin);
		if (param_num != 2) {print_middleware(command_i2c0_usage);return;}
		command_i2c0_deinit(scl_pin, sda_pin);
	}
	else {print_middleware(command_i2c0_usage);return;}
}

void command_i2c0_init(uint8_t scl_pin, uint8_t sda_pin, uint32_t frequency) {
	i2c0 = bflb_device_get_by_name("i2c0");
	command_gpio_init(scl_pin, GPIO_FUNC_I2C0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
	command_gpio_init(sda_pin, GPIO_FUNC_I2C0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
	bflb_i2c_init(i2c0, frequency);
	print_middleware("[F4U] i2c0 init scl: %u sda: %u freq: %u\r\n", scl_pin, sda_pin, frequency);
}

void command_i2c0_deinit(uint8_t scl_pin, uint8_t sda_pin) {
	bflb_i2c_deinit(i2c0);
	command_gpio_deinit(scl_pin);
	command_gpio_deinit(sda_pin);
	print_middleware("[F4U] i2c0 deinit scl: %u sda: %u\r\n", scl_pin, sda_pin);
}

/**************** uart1 ****************/
static char command_uart1_usage[] =
	"[F4U] Usage:\r\n"
	"[F4U] uart1 init\r\n"
	"[F4U] uart1 deinit\r\n";

void command_parse_uart1(char cmd[], uint16_t pos) {
	uint8_t param_num = 0;
	while (cmd[pos] == ' ') {++pos;}
	if (cmd[pos] == 0) {print_middleware(command_uart1_usage);return;}
	if (strncmp(cmd + pos, "init", 4) == 0) {
		uint8_t tx_pin;
		uint8_t rx_pin;
		uint32_t baudrate;
		uint8_t direction;
		uint8_t data_bits;
		uint8_t stop_bits;
		uint8_t parity;
		uint8_t bit_order;
		uint8_t flow_ctrl;
		uint8_t tx_fifo_threshold;
		uint8_t rx_fifo_threshold;
		param_num = sscanf(cmd + pos + 4, "%d%d%d%d%d%d%d%d%d%d%d",
			&tx_pin,
			&rx_pin,
			&baudrate,
			&direction,
			&data_bits,
			&stop_bits,
			&parity,
			&bit_order,
			&flow_ctrl,
			&tx_fifo_threshold,
			&rx_fifo_threshold
		);
		if (param_num != 11) {print_middleware(command_uart1_usage);return;}
		command_uart1_init(
			tx_pin,
			rx_pin,
			baudrate,
			direction,
			data_bits,
			stop_bits,
			parity,
			bit_order,
			flow_ctrl,
			tx_fifo_threshold,
			rx_fifo_threshold
		);
	}
	else if (strncmp(cmd + pos, "deinit", 6) == 0) {
		uint8_t tx_pin;
		uint8_t rx_pin;
		param_num = sscanf(cmd + pos + 6, "%d %d", &tx_pin, &rx_pin);
		if (param_num != 2) {print_middleware(command_uart1_usage);return;}
		command_uart1_deinit(tx_pin, rx_pin);
	}
	else {print_middleware(command_uart1_usage);return;}
}

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
) {
    struct bflb_uart_config_s cfg;

    uart1 = bflb_device_get_by_name("uart1");

    command_gpio_init(tx_pin, GPIO_UART_FUNC_UART1_TX);
    command_gpio_init(rx_pin, GPIO_UART_FUNC_UART1_RX);
	cfg.baudrate = baudrate;
	cfg.direction = direction;
	cfg.data_bits = data_bits;
	cfg.stop_bits = stop_bits;
	cfg.parity = parity;
	cfg.bit_order = bit_order;
	cfg.flow_ctrl = flow_ctrl;
	cfg.tx_fifo_threshold = tx_fifo_threshold;
	cfg.rx_fifo_threshold = rx_fifo_threshold;
	bflb_uart_init(uart1, &cfg);
	print_middleware("[F4U] uart1 init tx: %u rx: %u cfg: %u %u %u %u %u %u %u %u %u\r\n",
		tx_pin,
		rx_pin,
		baudrate,
		direction,
		data_bits,
		stop_bits,
		parity,
		bit_order,
		flow_ctrl,
		tx_fifo_threshold,
		rx_fifo_threshold
	);
}

void command_uart1_deinit(
	uint8_t tx_pin,
	uint8_t rx_pin
) {
	bflb_uart_deinit(uart1);
	command_gpio_deinit(tx_pin);
	command_gpio_deinit(rx_pin);
	print_middleware("[F4U] uart1 init tx: %u rx: %u\r\n", tx_pin, rx_pin);
}

/**************** pwm0 ****************/
static char command_pwm0_usage[] =
	"[F4U] Usage:\r\n"
	"[F4U] pwm0 init\r\n"
	"[F4U] pwm0 deinit\r\n"
	"[F4U] pwm0 start\r\n"
	"[F4U] pwm0 stop\r\n"
	"[F4U] pwm0 threshold\r\n";

void command_parse_pwm0(char cmd[], uint16_t pos) {
	uint8_t param_num = 0;
	while (cmd[pos] == ' ') {++pos;}
	if (cmd[pos] == 0) {print_middleware(command_pwm0_usage);return;}
	if (strncmp(cmd + pos, "init", 4) == 0) {
		uint8_t clk_source;
		uint16_t clk_div;
		uint16_t period;
		param_num = sscanf(cmd + pos + 4, "%d%d%d", &clk_source, &clk_div, &period);
		if (param_num != 3) {print_middleware(command_pwm0_usage);return;}
		command_pwm0_init(clk_source, clk_div, period);
	}
	else if (strncmp(cmd + pos, "deinit", 6) == 0) {
		command_pwm0_deinit();
	}
	else if (strncmp(cmd + pos, "start", 5) == 0) {
		uint8_t ch; uint8_t pin;
		param_num = sscanf(cmd + pos + 5, "%d%d", &ch, &pin);
		if (param_num != 2) {print_middleware(command_pwm0_usage);return;}
		command_pwm0_start(ch, pin);
	}
	else if (strncmp(cmd + pos, "stop", 4) == 0) {
		uint8_t ch; uint8_t pin;
		param_num = sscanf(cmd + pos + 4, "%d%d", &ch, &pin);
		if (param_num != 2) {print_middleware(command_pwm0_usage);return;}
		command_pwm0_stop(ch, pin);
	}
	else if (strncmp(cmd + pos, "threshold", 9) == 0) {
		uint8_t ch; uint16_t low_threhold; uint16_t high_threhold;
		param_num = sscanf(cmd + pos + 9, "%d%d%d", &ch, &low_threhold, &high_threhold);
		if (param_num != 3) {print_middleware(command_pwm0_usage);return;}
		command_pwm0_threshold(ch, low_threhold, high_threhold);
	}
	else {print_middleware(command_pwm0_usage);return;}
}

void command_pwm0_init(
	uint8_t clk_source,
	uint16_t clk_div,
	uint16_t period
) {
	struct bflb_pwm_v2_config_s cfg = {
		.clk_source = clk_source,
		.clk_div = clk_div,
		.period = period,
	};

	pwm0 = bflb_device_get_by_name("pwm_v2_0");
	bflb_pwm_v2_init(pwm0, &cfg);
	bflb_pwm_v2_start(pwm0);
	print_middleware("[F4U] pwm0 init clk_source: %u clk_div: %u period: %u\r\n", clk_source, clk_div, period);
}

void command_pwm0_start(uint8_t ch, uint8_t pin) {
	command_gpio_init(pin, GPIO_FUNC_PWM0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
	bflb_pwm_v2_channel_set_threshold(pwm0, ch, 0, 0);
	bflb_pwm_v2_channel_positive_start(pwm0, ch);
	print_middleware("[F4U] pwm0 start ch: %u pin: %u threshold: 0 0\r\n", ch, pin);
}

void command_pwm0_stop(uint8_t ch, uint8_t pin) {
	bflb_pwm_v2_channel_positive_stop(pwm0, ch);
	command_gpio_deinit(pin);
	print_middleware("[F4U] pwm0 stop ch: %u pin: %u\r\n", ch, pin);
}

void command_pwm0_threshold(uint8_t ch, uint16_t low_threhold, uint16_t high_threhold) {
	bflb_pwm_v2_channel_set_threshold(pwm0, ch, low_threhold, high_threhold);
	print_middleware("[F4U] pwm0 ch: %u threshold: %u %u\r\n", ch, low_threhold, high_threhold);
}

void command_pwm0_deinit() {
	bflb_pwm_v2_stop(pwm0);
	bflb_pwm_v2_deinit(pwm0);
	print_middleware("[F4U] pwm0 deinit\r\n");
}

/**************** AHT20 ****************/
static char command_AHT20_usage[] =
	"[F4U] Usage:\r\n"
	"[F4U] AHT20 init\r\n"
	"[F4U] AHT20 deinit\r\n"
	"[F4U] AHT20 read\r\n";

void command_parse_AHT20(char cmd[], uint16_t pos) {
	uint8_t param_num = 0;
	while (cmd[pos] == ' ') {++pos;}
	if (cmd[pos] == 0) {print_middleware(command_AHT20_usage);return;}
	if (strncmp(cmd + pos, "init", 4) == 0) {
		uint8_t scl_pin; uint8_t sda_pin;
		param_num = sscanf(cmd + pos + 4, "%d%d", &scl_pin, &sda_pin);
		if (param_num != 2) {print_middleware(command_AHT20_usage);return;}
		command_AHT20_init(scl_pin, sda_pin);
	}
	else if (strncmp(cmd + pos, "deinit", 6) == 0) {
		command_AHT20_deinit();
	}
	else if (strncmp(cmd + pos, "read", 4) == 0) {
		command_AHT20_read();
	}
	else {print_middleware(command_AHT20_usage);return;}
}

static uint8_t I2C0_scl_pin = 18, I2C0_sda_pin = 19;
void command_AHT20_init(uint8_t scl_pin, uint8_t sda_pin) {
	I2C0_scl_pin = scl_pin;
	I2C0_sda_pin = sda_pin;
	command_i2c0_init(I2C0_scl_pin, I2C0_sda_pin, 400000);
	AHT20_init(&AHT20_0, i2c0, 0x38, "AHT20_0");
	print_middleware("[F4U] AHT20 init scl: %u sda: %u\r\n", I2C0_scl_pin, I2C0_sda_pin);
}

void command_AHT20_deinit() {
	command_i2c0_deinit(I2C0_scl_pin, I2C0_sda_pin);
	print_middleware("[F4U] AHT20 deinit scl: %u sda: %u\r\n", I2C0_scl_pin, I2C0_sda_pin);
}

void command_AHT20_read() {
	float AHT20_humidity, AHT20_temperature;
	AHT20_humidity_temperature(&AHT20_0, &AHT20_humidity, &AHT20_temperature);
	print_middleware("[F4U] Humidity: %f Temperature: %f \r\n", AHT20_humidity, AHT20_temperature);
}

/**************** BMP280 ****************/
static char command_BMP280_usage[] =
	"[F4U] Usage:\r\n"
	"[F4U] BMP280 init\r\n"
	"[F4U] BMP280 deinit\r\n"
	"[F4U] BMP280 read\r\n";

void command_parse_BMP280(char cmd[], uint16_t pos) {
	uint8_t param_num = 0;
	while (cmd[pos] == ' ') {++pos;}
	if (cmd[pos] == 0) {print_middleware(command_BMP280_usage);return;}
	if (strncmp(cmd + pos, "init", 4) == 0) {
		uint8_t scl_pin; uint8_t sda_pin;
		param_num = sscanf(cmd + pos + 4, "%d%d", &scl_pin, &sda_pin);
		if (param_num != 2) {print_middleware(command_BMP280_usage);return;}
		command_BMP280_init(scl_pin, sda_pin);
	}
	else if (strncmp(cmd + pos, "deinit", 6) == 0) {
		command_BMP280_deinit();
	}
	else if (strncmp(cmd + pos, "read", 4) == 0) {
		command_BMP280_read();
	}
	else {print_middleware(command_BMP280_usage);return;}
}

void command_BMP280_init(uint8_t scl_pin, uint8_t sda_pin){
	I2C0_scl_pin = scl_pin;
	I2C0_sda_pin = sda_pin;
	command_i2c0_init(I2C0_scl_pin, I2C0_sda_pin, 400000);
	BMP280_init(&BMP280_0, i2c0, 0x77, "BMP280_0");
	print_middleware("[F4U] BMP280 init scl: %u sda: %u\r\n", I2C0_scl_pin, I2C0_sda_pin);
}

void command_BMP280_deinit() {
	command_i2c0_deinit(I2C0_scl_pin, I2C0_sda_pin);
	print_middleware("[F4U] BMP280 deinit scl: %u sda: %u\r\n", I2C0_scl_pin, I2C0_sda_pin);
}

void command_BMP280_read() {
	float BMP280_temperature, BMP280_pressure;
	BMP280_temperature_pressure(&BMP280_0, &BMP280_temperature, &BMP280_pressure);
	print_middleware("[F4U] Temperature: %f Pressure: %f\r\n", BMP280_temperature, BMP280_pressure);
}

/**************** DS18B20 ****************/
static char command_DS18B20_usage[] =
	"[F4U] Usage:\r\n"
	"[F4U] DS18B20 init\r\n"
	"[F4U] DS18B20 deinit\r\n"
	"[F4U] DS18B20 read\r\n";

void command_parse_DS18B20(char cmd[], uint16_t pos) {
	uint8_t param_num = 0;
	while (cmd[pos] == ' ') {++pos;}
	if (cmd[pos] == 0) {print_middleware(command_DS18B20_usage);return;}
	if (strncmp(cmd + pos, "init", 4) == 0) {
		uint8_t pin;
		param_num = sscanf(cmd + pos + 4, "%d", &pin);
		if (param_num != 1) {print_middleware(command_DS18B20_usage);return;}
		command_DS18B20_init(pin);
	}
	else if (strncmp(cmd + pos, "deinit", 6) == 0) {
		command_DS18B20_deinit();
	}
	else if (strncmp(cmd + pos, "read", 4) == 0) {
		command_DS18B20_read();
	}
	else {print_middleware(command_DS18B20_usage);return;}
}

static uint8_t DS18B20_pin = 30;

void command_DS18B20_init(uint8_t pin) {
	DS18B20_pin = pin;
	DS18B20_init(&DS18B20_0, DS18B20_pin, "DS18B20_0");
	print_middleware("[F4U] DS18B20 init pin: %u\r\n", DS18B20_pin);
}
void command_DS18B20_deinit() {
	command_gpio_deinit(DS18B20_pin);
	print_middleware("[F4U] DS18B20 deinit pin: %u\r\n", DS18B20_pin);
}

void command_DS18B20_read() {
	float DS18B20_temp;
	DS18B20_temperature(&DS18B20_0, &DS18B20_temp);
	print_middleware("[F4U] Temperature: %f\r\n", DS18B20_temp);
}

/**************** SOIL5 ****************/
static char command_SOIL5_usage[] =
	"[F4U] Usage:\r\n"
	"[F4U] SOIL5 init\r\n"
	"[F4U] SOIL5 deinit\r\n"
	"[F4U] SOIL5 read\r\n";

void command_parse_SOIL5(char cmd[], uint16_t pos) {
	uint8_t param_num = 0;
	while (cmd[pos] == ' ') {++pos;}
	if (cmd[pos] == 0) {print_middleware(command_SOIL5_usage);return;}
	if (strncmp(cmd + pos, "init", 4) == 0) {
		uint8_t tx_pin; uint8_t rx_pin; uint8_t addr;
		param_num = sscanf(cmd + pos + 4, "%d%d%d", &tx_pin, &rx_pin, &addr);
		if (param_num != 3) {print_middleware(command_SOIL5_usage);return;}
		command_SOIL5_init(tx_pin, rx_pin, addr);
	}
	else if (strncmp(cmd + pos, "deinit", 6) == 0) {
		command_SOIL5_deinit();
	}
	else if (strncmp(cmd + pos, "read", 4) == 0) {
		command_SOIL5_read();
	}
	else {print_middleware(command_SOIL5_usage);return;}
}

static uint8_t SOIL5_tx_pin = 23, SOIL5_rx_pin = 24;

void command_SOIL5_init(uint8_t tx_pin, uint8_t rx_pin, uint8_t addr) {
	command_uart1_init(
		tx_pin,
		rx_pin,
		UART1_BAUDRATE,
		UART1_DIRECTION,
		UART1_DATA_BITS,
		UART1_STOP_BITS,
		UART1_PARITY,
		UART1_BIT_ORDER,
		UART1_FLOW_CTRL,
		UART1_TX_FIFO_TH,
		UART1_RX_FIFO_TH
	);
	SOIL5_tx_pin = tx_pin;
	SOIL5_rx_pin = rx_pin;
	SOIL5_init(&SOIL5_0, uart1, addr, "SOIL5_0");
	print_middleware("[F4U] SOIL5 init tx: %u rx: %u addr %u\r\n", tx_pin, rx_pin, addr);
}

void command_SOIL5_deinit() {
	command_uart1_deinit(SOIL5_tx_pin, SOIL5_rx_pin);
	print_middleware("[F4U] SOIL5 deinit tx: %u rx: %u\r\n", SOIL5_tx_pin, SOIL5_rx_pin);
}

void command_SOIL5_read() {
	print_middleware("[F4U] %s: water: %d temperature: %d conductivity: %d\r\n",
		SOIL5_0.name,
		SOIL5_read(&SOIL5_0, SOIL5_WATER),
		SOIL5_read(&SOIL5_0, SOIL5_TEMPERATURE),
		SOIL5_read(&SOIL5_0, SOIL5_CONDUCTIVITY)
	);
}

/**************** SERVO ****************/
static char command_SERVO_usage[] =
	"[F4U] Usage:\r\n"
	"[F4U] SERVO init\r\n"
	"[F4U] SERVO deinit\r\n"
	"[F4U] SERVO setpos\r\n"
	"[F4U] SERVO getpos\r\n";

void command_parse_SERVO(char cmd[], uint16_t pos) {
	uint8_t param_num = 0;
	while (cmd[pos] == ' ') {++pos;}
	if (cmd[pos] == 0) {print_middleware(command_SERVO_usage);return;}
	if (strncmp(cmd + pos, "init", 4) == 0) {
		uint8_t ch; uint8_t pin;
		param_num = sscanf(cmd + pos + 4, "%d%d", &ch, &pin);
		if (param_num != 2) {print_middleware(command_SERVO_usage);return;}
		command_SERVO_init(ch, pin);
	}
	else if (strncmp(cmd + pos, "deinit", 6) == 0) {
		command_SERVO_deinit();
	}
	else if (strncmp(cmd + pos, "setpos", 6) == 0) {
		uint16_t p;
		param_num = sscanf(cmd + pos + 6, "%d", &p);
		if (param_num != 1) {print_middleware(command_SERVO_usage);return;}
		command_SERVO_setpos(p);
	}
	else if (strncmp(cmd + pos, "getpos", 6) == 0) {
		command_SERVO_getpos();
	}
	else {print_middleware(command_SERVO_usage);return;}
}

static uint8_t SERVO_pin = 31, SERVO_ch = 3;

void command_SERVO_init(
	uint8_t ch,
	uint8_t pin
) {
	SERVO_ch = ch;
	SERVO_pin = pin;
	command_gpio_init(SERVO_pin, GPIO_FUNC_PWM0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
	command_pwm0_init(BFLB_SYSTEM_PBCLK, 80, 20000);
	command_pwm0_start(SERVO_ch, SERVO_pin);
	SERVO_init(
		&SERVO_DOWN,
		ch,
		SERVO_DOWN_MIN_POS,
		SERVO_DOWN_MAX_POS,
		(SERVO_DOWN_MAX_POS - SERVO_DOWN_MIN_POS) / 2,
		SERVO_DOWN_MIN_THREHOLD,
		SERVO_DOWN_MAX_THREHOLD,
		"SERVO_DOWN"
	);
	print_middleware("[F4U] SERVO init ch: %u pin: %u pos: %u~%u\r\n", ch, pin, SERVO_DOWN_MIN_POS, SERVO_DOWN_MAX_POS);
}

void command_SERVO_deinit() {
	command_pwm0_stop(SERVO_ch, SERVO_pin);
	command_pwm0_deinit();
	command_gpio_deinit(SERVO_pin);
	print_middleware("[F4U] SERVO deinit ch: %u pin: %u\r\n", SERVO_ch, SERVO_pin);
}

void command_SERVO_setpos(float pos) {
	SERVO_setpos(&SERVO_DOWN, pos);
	print_middleware("[F4U] SERVO at: %f\r\n", pos);
}

void command_SERVO_getpos() {
	float pos;
	SERVO_getpos(&SERVO_DOWN, &pos);
	print_middleware("[F4U] SERVO at: %f\r\n", pos);
}

/**************** MOTOR ****************/
static char command_MOTOR_A_usage[] =
	"[F4U] Usage:\r\n"
	"[F4U] MOTOR A init\r\n"
	"[F4U] MOTOR A deinit\r\n"
	"[F4U] MOTOR A neutral\r\n"
	"[F4U] MOTOR A positive\r\n"
	"[F4U] MOTOR A negative\r\n"
	"[F4U] MOTOR A brake\r\n";

void command_parse_MOTOR_A(char cmd[], uint16_t pos) {
	uint8_t param_num = 0;
	while (cmd[pos] == ' ') {++pos;}
	if (cmd[pos] == 0) {print_middleware(command_MOTOR_A_usage);return;}
	if (strncmp(cmd + pos, "init", 4) == 0) {
		uint8_t pin1; uint8_t pin2;
		param_num = sscanf(cmd + pos + 4, "%d%d", &pin1, &pin2);
		if (param_num != 2) {print_middleware(command_MOTOR_A_usage);return;}
		command_MOTOR_A_init(pin1, pin2);
	}
	else if (strncmp(cmd + pos, "deinit", 6) == 0) {
		command_MOTOR_A_deinit();
	}
	else if (strncmp(cmd + pos, "neutral", 7) == 0) {
		command_MOTOR_A_neutral();
	}
	else if (strncmp(cmd + pos, "positive", 8) == 0) {
		command_MOTOR_A_positive();
	}
	else if (strncmp(cmd + pos, "negative", 8) == 0) {
		command_MOTOR_A_negative();
	}
	else if (strncmp(cmd + pos, "brake", 5) == 0) {
		command_MOTOR_A_brake();
	}
	else {print_middleware(command_MOTOR_A_usage);return;}
}

static uint8_t MOTOR_A_pin1 = 25, MOTOR_A_pin2 = 26;

void command_MOTOR_A_init(uint8_t pin1, uint8_t pin2) {
	MOTOR_A_pin1 = pin1, MOTOR_A_pin2 = pin2;
	command_gpio_init(MOTOR_A_pin1, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
	command_gpio_init(MOTOR_A_pin2, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
	MOTOR_init(&MOTOR_A, MOTOR_A_pin1, MOTOR_A_pin2, "MOTOR_A");
	print_middleware("[F4U] MOTOR A init AIN1: %u AIN2: %u\r\n", MOTOR_A_pin1, MOTOR_A_pin2);
}

void command_MOTOR_A_deinit() {
	command_gpio_deinit(MOTOR_A_pin1);
	command_gpio_deinit(MOTOR_A_pin2);
	print_middleware("[F4U] MOTOR A init AIN1: %u AIN2: %u\r\n", MOTOR_A_pin1, MOTOR_A_pin2);
}

void command_MOTOR_A_neutral() {
	MOTOR_neu(&MOTOR_A);
	print_middleware("[F4U] MOTOR A neutral\r\n");
}

void command_MOTOR_A_positive(){
	MOTOR_pos(&MOTOR_A);
	print_middleware("[F4U] MOTOR A positive\r\n");
}

void command_MOTOR_A_negative() {
	MOTOR_neg(&MOTOR_A);
	print_middleware("[F4U] MOTOR A negative\r\n");
}

void command_MOTOR_A_brake() {
	MOTOR_brake(&MOTOR_A);
	print_middleware("[F4U] MOTOR A brake\r\n");
}

static char command_MOTOR_B_usage[] =
	"[F4U] Usage:\r\n"
	"[F4U] MOTOR B init\r\n"
	"[F4U] MOTOR B deinit\r\n"
	"[F4U] MOTOR B neutral\r\n"
	"[F4U] MOTOR B positive\r\n"
	"[F4U] MOTOR B negative\r\n"
	"[F4U] MOTOR B brake\r\n";

void command_parse_MOTOR_B(char cmd[], uint16_t pos) {
	uint8_t param_num = 0;
	while (cmd[pos] == ' ') {++pos;}
	if (cmd[pos] == 0) {print_middleware(command_MOTOR_B_usage);return;}
	if (strncmp(cmd + pos, "init", 4) == 0) {
		uint8_t pin1; uint8_t pin2;
		param_num = sscanf(cmd + pos + 4, "%d%d", &pin1, &pin2);
		if (param_num != 2) {print_middleware(command_MOTOR_B_usage);return;}
		command_MOTOR_B_init(pin1, pin2);
	}
	else if (strncmp(cmd + pos, "deinit", 6) == 0) {
		command_MOTOR_B_deinit();
	}
	else if (strncmp(cmd + pos, "neutral", 7) == 0) {
		command_MOTOR_B_neutral();
	}
	else if (strncmp(cmd + pos, "positive", 8) == 0) {
		command_MOTOR_B_positive();
	}
	else if (strncmp(cmd + pos, "negative", 8) == 0) {
		command_MOTOR_B_negative();
	}
	else if (strncmp(cmd + pos, "brake", 5) == 0) {
		command_MOTOR_B_brake();
	}
	else {print_middleware(command_MOTOR_B_usage);return;}
}

static uint8_t MOTOR_B_pin1 = 27, MOTOR_B_pin2 = 28;

void command_MOTOR_B_init(uint8_t pin1, uint8_t pin2) {
	MOTOR_B_pin1 = pin1;
	MOTOR_B_pin2 = pin2;
	command_gpio_init(MOTOR_B_pin1, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
	command_gpio_init(MOTOR_B_pin2, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
	MOTOR_init(&MOTOR_B, MOTOR_B_pin1, MOTOR_B_pin2, "MOTOR_B");
	print_middleware("[F4U] MOTOR B init BIN1: %u BIN2: %u\r\n", MOTOR_B_pin1, MOTOR_B_pin2);
}

void command_MOTOR_B_deinit() {
	command_gpio_deinit(MOTOR_B_pin1);
	command_gpio_deinit(MOTOR_B_pin2);
	print_middleware("[F4U] MOTOR B init BIN1: %u BIN2: %u\r\n", MOTOR_B_pin1, MOTOR_B_pin2);
}

void command_MOTOR_B_neutral() {
	MOTOR_neu(&MOTOR_B);
	print_middleware("[F4U] MOTOR B neutral\r\n");
}

void command_MOTOR_B_positive(){
	MOTOR_pos(&MOTOR_B);
	print_middleware("[F4U] MOTOR B positive\r\n");
}

void command_MOTOR_B_negative() {
	MOTOR_neg(&MOTOR_B);
	print_middleware("[F4U] MOTOR B negative\r\n");
}

void command_MOTOR_B_brake() {
	MOTOR_brake(&MOTOR_B);
	print_middleware("[F4U] MOTOR B brake\r\n");
}

/**************** MAGNET ****************/
static char command_MAGNET_usage[] =
	"[F4U] Usage:\r\n"
	"[F4U] MAGNET init\r\n"
	"[F4U] MAGNET deinit\r\n"
	"[F4U] MAGNET on\r\n"
	"[F4U] MAGNET off\r\n";

void command_parse_MAGNET(char cmd[], uint16_t pos) {
	uint8_t param_num = 0;
	while (cmd[pos] == ' ') {++pos;}
	if (cmd[pos] == 0) {print_middleware(command_MAGNET_usage);return;}
	if (strncmp(cmd + pos, "init", 4) == 0) {
		uint8_t pin;
		param_num = sscanf(cmd + pos + 4, "%d", &pin);
		if (param_num != 1) {print_middleware(command_MAGNET_usage);return;}
		command_MAGNET_init(pin);
	}
	else if (strncmp(cmd + pos, "deinit", 6) == 0) {
		command_MAGNET_deinit();
	}
	else if (strncmp(cmd + pos, "on", 2) == 0) {
		command_MAGNET_on();
	}
	else if (strncmp(cmd + pos, "off", 3) == 0) {
		command_MAGNET_off();
	}
	else {print_middleware(command_MAGNET_usage);return;}
}


static uint8_t MAGNET_pin = 20;

void command_MAGNET_init(uint8_t pin) {
	MAGNET_pin = pin;
	command_gpio_init(MAGNET_pin, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
	print_middleware("[F4U] MAGNET init pin: %u\r\n", MAGNET_pin);
}

void command_MAGNET_deinit() {
	command_gpio_deinit(MAGNET_pin);
	print_middleware("[F4U] MAGNET deinit pin: %u\r\n", MAGNET_pin);
}

void command_MAGNET_on() {
	command_gpio_set(MAGNET_pin);
	print_middleware("[F4U] MAGNET on\r\n");
}
void command_MAGNET_off() {
	command_gpio_reset(MAGNET_pin);
	print_middleware("[F4U] MAGNET off\r\n");
}

/**************** ALL ****************/
static char command_ALL_usage[] =
	"[F4U] Usage:\r\n"
	"[F4U] ALL read\r\n";

void command_parse_ALL(char cmd[], uint16_t pos) {
	while (cmd[pos] == ' ') {++pos;}
	if (cmd[pos] == 0) {print_middleware(command_ALL_usage);return;}
	if (strncmp(cmd + pos, "read", 4) == 0) {
		float AHT20_humidity, AHT20_temperature;
		float BMP280_temperature, BMP280_pressure;
		AHT20_humidity_temperature(&AHT20_0, &AHT20_humidity, &AHT20_temperature);
		BMP280_temperature_pressure(&BMP280_0, &BMP280_temperature, &BMP280_pressure);
		print_middleware("[F4U] all: %f %f %f %f %d %d %d %d\r\n",
			AHT20_humidity, AHT20_temperature,
			BMP280_temperature, BMP280_pressure,
			SOIL5_read(&SOIL5_0, SOIL5_WATER),
			SOIL5_read(&SOIL5_0, SOIL5_TEMPERATURE),
			SOIL5_read(&SOIL5_0, SOIL5_CONDUCTIVITY),
			SOIL5_read(&SOIL5_0, SOIL5_PH)
		);
	}
	else {print_middleware(command_ALL_usage);return;}
}

#ifdef CONFIG_SHELL
#include "shell.h"
static void test_all(int argc, char **argv) {
	float AHT20_humidity, AHT20_temperature;
	float BMP280_temperature, BMP280_pressure;
	AHT20_humidity_temperature(&AHT20_0, &AHT20_humidity, &AHT20_temperature);
	BMP280_temperature_pressure(&BMP280_0, &BMP280_temperature, &BMP280_pressure);
	printf("[F4U] all: %f %f %f %f %d %d %d %d\r\n",
		AHT20_humidity, AHT20_temperature,
		BMP280_temperature, BMP280_pressure,
		SOIL5_read(&SOIL5_0, SOIL5_WATER),
		SOIL5_read(&SOIL5_0, SOIL5_TEMPERATURE),
		SOIL5_read(&SOIL5_0, SOIL5_CONDUCTIVITY),
		SOIL5_read(&SOIL5_0, SOIL5_PH)
	);
}

SHELL_CMD_EXPORT_ALIAS(test_all, test_all, test all);
#endif
