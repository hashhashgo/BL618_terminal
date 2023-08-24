/*
 * SERVO.c
 *
 *      Author: yu_ha
 */

#include "servo.h"

extern struct bflb_device_s *pwm0;

void __SERVO_unlock(struct SERVO_handle_s *handle) {
	uint64_t start_time = bflb_mtimer_get_time_ms();
	while (bflb_mtimer_get_time_ms() - start_time < SERVO_MAX_WAITING_TIME && !handle->free); // Maximum waiting time 100ms
	if (!handle->free) {
		LOG_E("%s: Waiting too long for unlocking\r\n", handle->name);
		return;
	}
	handle->free = 0;
}

void SERVO_init(struct SERVO_handle_s *handle, uint8_t channel, float min_pos, float max_pos, float reset_pos, uint16_t min_threhold, uint16_t max_threhold, char *name) {
	handle->free = 0;
	strncpy(handle->name, name, 21);
	handle->name[20] = 0;
	handle->channel = channel;
	handle->min_pos = min_pos;
	handle->max_pos = max_pos;
	handle->reset_pos = reset_pos;
	handle->min_threhold = min_threhold;
	handle->max_threhold = max_threhold;
	handle->pos = reset_pos;
	__SERVO_setpos(handle, handle->reset_pos);
	handle->free = 1;
}

void SERVO_reset(struct SERVO_handle_s *handle) {
	__SERVO_unlock(handle);
	__SERVO_setpos(handle, handle->reset_pos);
	__SERVO_release(handle);
}

void SERVO_setpos(struct SERVO_handle_s *handle, float pos) {
	__SERVO_unlock(handle);
	__SERVO_setpos(handle, pos);
	__SERVO_release(handle);
}

void SERVO_getpos(struct SERVO_handle_s *handle, float *pos) {
	__SERVO_unlock(handle);
	*pos = handle->pos;
	__SERVO_release(handle);
}
