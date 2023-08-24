/*
 * motor.h
 *
 *      Author: yu_ha
 */

#ifndef EXAMPLES_FINAL_ACTUATOR_MOTOR_H_
#define EXAMPLES_FINAL_ACTUATOR_MOTOR_H_

#include "bflb_mtimer.h"
#include "bflb_gpio.h"

#include "log.h"

#define MOTOR_MAX_WAITING_TIME 100

enum MOTOR_STAT {
	MOTOR_NEU,
	MOTOR_POS,
	MOTOR_NEG,
	MOTOR_BRAKE
};

struct MOTOR_handle_s {
	char name[21];
	uint8_t IN1, IN2;
	uint8_t stat;
	uint8_t free;
};

void __MOTOR_unlock(struct MOTOR_handle_s *handle);
#define __MOTOR_release(handle) {handle->free = 1;}

void MOTOR_init(struct MOTOR_handle_s *handle, uint8_t IN1, uint8_t IN2, char *name);
void MOTOR_neu(struct MOTOR_handle_s *handle);
void MOTOR_pos(struct MOTOR_handle_s *handle);
void MOTOR_neg(struct MOTOR_handle_s *handle);
void MOTOR_brake(struct MOTOR_handle_s *handle);

#endif /* EXAMPLES_FINAL_ACTUATOR_MOTOR_H_ */
