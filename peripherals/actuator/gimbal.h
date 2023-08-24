/*
 * gimbal.h
 *
 *      Author: yu_ha
 */

#ifndef EXAMPLES_FINAL_ACTUATOR_GIMBAL_H_
#define EXAMPLES_FINAL_ACTUATOR_GIMBAL_H_

#include "bflb_mtimer.h"
#include "servo.h"

#include "log.h"

#define GIMBAL_MAX_WAITING_TIME 100

struct GIMBAL_handle_s {
	char name[21];
	struct SERVO_handle_s SERVO_DOWN, SERVO_UP;
	uint8_t free;
};

void __GIMBAL_unlock(struct GIMBAL_handle_s *handle);
#define __GIMBAL_release(handle) {handle->free = 1;}

void GIMBAL_init(
	struct GIMBAL_handle_s *handle,
	uint8_t down_channel, float down_min_pos, float down_max_pos, float down_reset_pos, uint16_t down_min_threhold, uint16_t down_max_threhold,
	uint8_t up_channel, float up_min_pos, float up_max_pos, float up_reset_pos, uint16_t up_min_threhold, uint16_t up_max_threhold,
	char *name);
void GIMBAL_reset(struct GIMBAL_handle_s *handle);
void GIMBAL_setpos(struct GIMBAL_handle_s *handle, float pos_down, float pos_up);
void GIMBAL_getpos(struct GIMBAL_handle_s *handle, float *pos_down, float *pos_up);

#endif /* EXAMPLES_FINAL_ACTUATOR_GIMBAL_H_ */
