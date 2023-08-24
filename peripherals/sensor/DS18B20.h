/*
 * DS18B20.h
 *
 *      Author: yu_ha
 */

#ifndef EXAMPLES_FINAL_SENSOR_DS18B20_H_
#define EXAMPLES_FINAL_SENSOR_DS18B20_H_

#include "bflb_gpio.h"
#include "bflb_mtimer.h"

#include "log.h"

#define DS18B20_MAX_WAITING_TIME 100

struct DS18B20_handle_s {
	char name[21];
	uint8_t pin;
	uint8_t free;
};

#define DS18B20_IO_OUT(handle)	bflb_gpio_init(gpio, handle->pin, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
#define DS18B20_IO_IN(handle)	bflb_gpio_init(gpio, handle->pin, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);

#define DS18B20_IO_SET(handle)	bflb_gpio_set(gpio, handle->pin)
#define DS18B20_IO_RESET(handle) bflb_gpio_reset(gpio, handle->pin)
#define DS18B20_IO_READ(handle)	bflb_gpio_read(gpio, handle->pin)

void __DS18B20_unlock(struct DS18B20_handle_s *handle);
#define __DS18B20_release(handle) {handle->free = 1;}

void DS18B20_init(struct DS18B20_handle_s *handle, uint8_t pin, char *name);
void DS18B20_temperature(struct DS18B20_handle_s *handle, float *temperature);

#define DS18B20_reset(handle) {__DS18B20_unlock(handle);__DS18B20_reset(handle);__DS18B20_release(handle);}
uint8_t DS18B20_check(struct DS18B20_handle_s *handle);

void __DS18B20_write_byte(struct DS18B20_handle_s *handle, uint8_t date);
uint8_t __DS18B20_read_byte(struct DS18B20_handle_s *handle);
void __DS18B20_reset(struct DS18B20_handle_s *handle);
uint8_t __DS18B20_check(struct DS18B20_handle_s *handle);

#endif /* EXAMPLES_FINAL_SENSOR_DS18B20_H_ */
