/*
 * SERVO.h
 *
 *      Author: yu_ha
 */

#ifndef EXAMPLES_FINAL_ACTUATOR_SERVO_H_
#define EXAMPLES_FINAL_ACTUATOR_SERVO_H_

#include "bflb_mtimer.h"
#include "bflb_pwm_v2.h"

#include "log.h"

#define SERVO_MAX_WAITING_TIME 100

struct SERVO_handle_s {
	char name[21];
	uint8_t channel;
	float pos, reset_pos, min_pos, max_pos;
	uint16_t min_threhold, max_threhold;
	uint8_t free;
};

void __SERVO_unlock(struct SERVO_handle_s *handle);
#define __SERVO_release(handle) {handle->free = 1;}

#define __SERVO_pos2threhold(h, p) ((__SERVO_pos_clip(h, p) - (h)->min_pos) * 1.0 / ((h)->max_pos - (h)->min_pos) * ((h)->max_threhold - (h)->min_threhold) + (h)->min_threhold)
#define __SERVO_pos_clip(h, p) (((p) < (h)->min_pos) ? (h)->min_pos : (((p) > (h)->max_pos)) ? (h)->max_pos : (p))
#define __SERVO_setpos(h, p) {(h)->pos = (p); bflb_pwm_v2_channel_set_threshold(pwm0, (h)->channel, 0, __SERVO_pos2threhold((h), (h)->pos));}

void SERVO_init(struct SERVO_handle_s *handle, uint8_t channel, float min_pos, float max_pos, float reset_pos, uint16_t min_threhold, uint16_t max_threhold, char *name);
void SERVO_reset(struct SERVO_handle_s *handle);
void SERVO_setpos(struct SERVO_handle_s *handle, float pos);
void SERVO_getpos(struct SERVO_handle_s *handle, float *pos);

#endif /* EXAMPLES_FINAL_ACTUATOR_SERVO_H_ */
