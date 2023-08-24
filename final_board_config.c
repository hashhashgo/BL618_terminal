/*
 * final_board_config.c
 *
 *      Author: yu_ha
 */

#include "final_board_config.h"

struct bflb_device_s *gpio;
struct bflb_device_s *i2c0;
struct bflb_device_s *uart0;
struct bflb_device_s *uart1;
struct bflb_device_s *pwm0;

void final_board_init() {
    board_init();
	final_board_gpio_init();
	final_board_i2c0_init();
	final_board_uart1_init();
	final_board_pwm0_init();
}

void final_board_gpio_init() {
	gpio = bflb_device_get_by_name("gpio");
#ifdef I2C0_ENABLE
	bflb_gpio_init(gpio, I2C0_SCL_PIN, GPIO_FUNC_I2C0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
	bflb_gpio_init(gpio, I2C0_SDA_PIN, GPIO_FUNC_I2C0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
#endif
#ifdef UART1_ENABLE
	bflb_gpio_uart_init(gpio, UART1_TX_PIN, GPIO_UART_FUNC_UART1_TX);
	bflb_gpio_uart_init(gpio, UART1_RX_PIN, GPIO_UART_FUNC_UART1_RX);
#endif
#if (MOTOR_ENABLE)
	bflb_gpio_init(gpio, MOTOR_AIN1_PIN, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
	bflb_gpio_init(gpio, MOTOR_AIN2_PIN, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
	bflb_gpio_init(gpio, MOTOR_BIN1_PIN, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
	bflb_gpio_init(gpio, MOTOR_BIN2_PIN, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
#endif
//#if (DS18B20_ENABLE)
//	bflb_gpio_init(gpio, DS18B20_PIN, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
//#endif
#if (SERVO_ENABLE)
	bflb_gpio_init(gpio, PWM0_CH2P_PIN, GPIO_FUNC_PWM0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
	bflb_gpio_init(gpio, PWM0_CH3P_PIN, GPIO_FUNC_PWM0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
#endif
}

void final_board_i2c0_init() {
#ifdef I2C0_ENABLE
	i2c0 = bflb_device_get_by_name("i2c0");
	bflb_i2c_init(i2c0, 400000);
#endif
}

//void uart1_isr(int irq, void *arg)
//{
//    uint32_t intstatus = bflb_uart_get_intstatus(uart1);
//
//    if (intstatus & UART_INTSTS_RX_FIFO) {
//        while (bflb_uart_rxavailable(uart1)) {
//            printf("0x%02x\r\n", bflb_uart_getchar(uart1));
//        }
//    }
//    if (intstatus & UART_INTSTS_RTO) {
//        while (bflb_uart_rxavailable(uart1)) {
//            printf("0x%02x\r\n", bflb_uart_getchar(uart1));
//        }
//        bflb_uart_int_clear(uart1, UART_INTCLR_RTO);
//    }
//    if (intstatus & UART_INTSTS_TX_END) {
//        printf("tx end\r\n");
//        bflb_uart_int_clear(uart1, UART_INTCLR_TX_END);
//    }
//    if (intstatus & UART_INTSTS_RX_END) {
//        printf("rx end\r\n");
//        bflb_uart_int_clear(uart1, UART_INTCLR_RX_END);
//    }
//}

void final_board_uart1_init() {
    struct bflb_uart_config_s cfg;

#ifdef UART1_ENABLE
	uart1 = bflb_device_get_by_name("uart1");

	cfg.baudrate = UART1_BAUDRATE;
	cfg.data_bits = UART1_DATA_BITS;
	cfg.stop_bits = UART1_STOP_BITS;
	cfg.parity = UART1_PARITY;
	cfg.flow_ctrl = UART1_FLOW_CTRL;
	cfg.tx_fifo_threshold = UART1_TX_FIFO_TH;
	cfg.rx_fifo_threshold = UART1_RX_FIFO_TH;
	bflb_uart_init(uart1, &cfg);

//	bflb_uart_feature_control(uart1, UART_CMD_SET_RX_TRANSFER_LEN, 5);
//    bflb_uart_feature_control(uart1, UART_CMD_SET_RX_END_INTERRUPT, true);
//    bflb_uart_rxint_mask(uart1, false);
//    bflb_irq_attach(uart1->irq_num, uart1_isr, NULL);
//    bflb_irq_enable(uart1->irq_num);
#endif
}

void final_board_pwm0_init() {
	pwm0 = bflb_device_get_by_name("pwm_v2_0");
#ifdef PWM0_ENABLE
	/* period = .PBCLK / .clk_div / .period = 80MHz / 80 / 20000 = 50Hz */
	struct bflb_pwm_v2_config_s cfg = {
		.clk_source = BFLB_SYSTEM_PBCLK,
		.clk_div = 80,
		.period = 20000,
	};
	bflb_pwm_v2_init(pwm0, &cfg);
	bflb_pwm_v2_channel_set_threshold(pwm0, SERVO_DOWN_CHANNEL, 0, 0); /* duty = (999-0)/20000 = 2.5% */
	bflb_pwm_v2_channel_set_threshold(pwm0, SERVO_UP_CHANNEL, 0, 0); /* duty = (500-0)/20000 = 2.5% */
	bflb_pwm_v2_channel_positive_start(pwm0, SERVO_DOWN_CHANNEL);
	bflb_pwm_v2_channel_positive_start(pwm0, SERVO_UP_CHANNEL);
	bflb_pwm_v2_start(pwm0);
#endif
}
