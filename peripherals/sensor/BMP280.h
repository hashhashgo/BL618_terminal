/*
 * BMP280.h
 *
 *      Author: yu_ha
 */

#ifndef EXAMPLES_FINAL_SENSOR_BMP280_H_
#define EXAMPLES_FINAL_SENSOR_BMP280_H_

#include "bflb_core.h"
#include "bflb_i2c.h"
#include "bflb_mtimer.h"
#include "log.h"

#define BMP280_MAX_WAITING_TIME 100

struct BMP280_handle_s {
	char name[21];
	struct bflb_device_s *device;
	uint16_t addr;
	int T[3];
	int P[9];
	uint8_t free;
};

void __BMP280_unlock(struct BMP280_handle_s *handle);
#define __BMP280_release(handle) {handle->free = 1;}

void BMP280_init(struct BMP280_handle_s *handle, struct bflb_device_s *device, uint16_t addr, char *name);
void BMP280_temperature_pressure(struct BMP280_handle_s *handle, float *temperature, float *pressure);
void __BMP280_set_register(struct BMP280_handle_s *handle, uint8_t oversampling, uint8_t standby);

#endif /* EXAMPLES_FINAL_SENSOR_BMP280_H_ */
