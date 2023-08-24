/*
 * gimbal.c
 *
 *      Author: yu_ha
 */

#include "gimbal.h"

void __GIMBAL_unlock(struct GIMBAL_handle_s *handle) {
	uint64_t start_time = bflb_mtimer_get_time_ms();
	while (bflb_mtimer_get_time_ms() - start_time < GIMBAL_MAX_WAITING_TIME && !handle->free); // Maximum waiting time 100ms
	if (!handle->free) {
		LOG_E("%s: Waiting too long for unlocking\r\n", handle->name);
		return;
	}
	handle->free = 0;
}

void GIMBAL_init(
	struct GIMBAL_handle_s *handle,
	uint8_t down_channel, float down_min_pos, float down_max_pos, float down_reset_pos, uint16_t down_min_threhold, uint16_t down_max_threhold,
	uint8_t up_channel, float up_min_pos, float up_max_pos, float up_reset_pos, uint16_t up_min_threhold, uint16_t up_max_threhold,
	char *name) {
	handle->free = 0;
	strncpy(handle->name, name, 21);
	handle->name[20] = 0;
	handle->SERVO_DOWN.channel = down_channel;
	handle->SERVO_DOWN.min_pos = down_min_pos;
	handle->SERVO_DOWN.max_pos = down_max_pos;
	handle->SERVO_DOWN.reset_pos = down_reset_pos;
	handle->SERVO_DOWN.min_threhold = down_min_threhold;
	handle->SERVO_DOWN.max_threhold = down_max_threhold;
	handle->SERVO_DOWN.pos = down_reset_pos;
	SERVO_setpos(&handle->SERVO_DOWN, handle->SERVO_DOWN.reset_pos);
	handle->SERVO_UP.channel = up_channel;
	handle->SERVO_UP.min_pos = up_min_pos;
	handle->SERVO_UP.max_pos = up_max_pos;
	handle->SERVO_UP.reset_pos = up_reset_pos;
	handle->SERVO_UP.min_threhold = up_min_threhold;
	handle->SERVO_UP.max_threhold = up_max_threhold;
	handle->SERVO_UP.pos = up_reset_pos;
	SERVO_setpos(&handle->SERVO_UP, handle->SERVO_UP.reset_pos);
	handle->free = 1;
}
void GIMBAL_reset(struct GIMBAL_handle_s *handle) {
	__GIMBAL_unlock(handle);
	SERVO_reset(&handle->SERVO_DOWN);
	SERVO_reset(&handle->SERVO_UP);
	__GIMBAL_release(handle);
}

void GIMBAL_setpos(struct GIMBAL_handle_s *handle, float pos_down, float pos_up) {
	__GIMBAL_unlock(handle);
	SERVO_setpos(&handle->SERVO_DOWN, pos_down);
	SERVO_setpos(&handle->SERVO_UP, pos_up);
	__GIMBAL_release(handle);
}

void GIMBAL_getpos(struct GIMBAL_handle_s *handle, float *pos_down, float *pos_up) {
	__GIMBAL_unlock(handle);
	SERVO_getpos(&handle->SERVO_DOWN, pos_down);
	SERVO_getpos(&handle->SERVO_UP, pos_up);
	__GIMBAL_release(handle);
}
