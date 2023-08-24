/*
 * AHT20.h
 *
 *      Author: yu_ha
 */

#ifndef EXAMPLES_FINAL_AHT20_AHT20_H_
#define EXAMPLES_FINAL_AHT20_AHT20_H_

#include "bflb_core.h"
#include "bflb_i2c.h"
#include "bflb_mtimer.h"
#include "log.h"

#define AHT20_MAX_WAITING_TIME 100

struct AHT20_handle_s {
	char name[21];
	struct bflb_device_s *device;
	uint16_t addr;
	uint8_t free;
};

void __AHT20_unlock(struct AHT20_handle_s *handle);
#define __AHT20_release(handle) {handle->free = 1;}

void AHT20_init(struct AHT20_handle_s *handle, struct bflb_device_s *device, uint16_t addr, char *name);
void AHT20_humidity_temperature(struct AHT20_handle_s *handle, float *humidity, float *temperature);
void AHT20_reset(struct AHT20_handle_s *handle);

#endif /* EXAMPLES_FINAL_AHT20_AHT20_H_ */
