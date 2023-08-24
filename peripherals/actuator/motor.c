/*
 * motor.c
 *
 *      Author: yu_ha
 */

#include "motor.h"

extern struct bflb_device_s *gpio;

void __MOTOR_unlock(struct MOTOR_handle_s *handle) {
	uint64_t start_time = bflb_mtimer_get_time_ms();
	while (bflb_mtimer_get_time_ms() - start_time < MOTOR_MAX_WAITING_TIME && !handle->free); // Maximum waiting time 100ms
	if (!handle->free) {
		LOG_E("%s: Waiting too long for unlocking\r\n", handle->name);
		return;
	}
	handle->free = 0;
}

void MOTOR_init(struct MOTOR_handle_s *handle, uint8_t IN1, uint8_t IN2, char *name) {
	handle->free = 0;
	handle->IN1 = IN1;
	handle->IN2 = IN2;
	strncpy(handle->name, name, 21);
	handle->name[20] = 0;
	bflb_gpio_reset(gpio, handle->IN1);
	bflb_gpio_reset(gpio, handle->IN2);
	handle->stat = MOTOR_NEU;
	handle->free = 1;
}

void MOTOR_neu(struct MOTOR_handle_s *handle) {
	__MOTOR_unlock(handle);

	if (handle->stat != MOTOR_NEU) {
		bflb_gpio_reset(gpio, handle->IN1);
		bflb_gpio_reset(gpio, handle->IN2);
		handle->stat = MOTOR_NEU;
	}
	__MOTOR_release(handle);
}

void MOTOR_pos(struct MOTOR_handle_s *handle) {
	__MOTOR_unlock(handle);
	if (handle->stat != MOTOR_POS) {
		bflb_gpio_set(gpio, handle->IN1);
		bflb_gpio_reset(gpio, handle->IN2);
		handle->stat = MOTOR_POS;
	}
	__MOTOR_release(handle);
}

void MOTOR_neg(struct MOTOR_handle_s *handle) {
	__MOTOR_unlock(handle);
	if (handle->stat != MOTOR_NEG) {
		bflb_gpio_reset(gpio, handle->IN1);
		bflb_gpio_set(gpio, handle->IN2);
		handle->stat = MOTOR_NEG;
	}
	__MOTOR_release(handle);
}

void MOTOR_brake(struct MOTOR_handle_s *handle) {
	__MOTOR_unlock(handle);
	if (handle->stat != MOTOR_BRAKE) {
		bflb_gpio_set(gpio, handle->IN1);
		bflb_gpio_set(gpio, handle->IN2);
		handle->stat = MOTOR_BRAKE;
	}
	__MOTOR_release(handle);
}
