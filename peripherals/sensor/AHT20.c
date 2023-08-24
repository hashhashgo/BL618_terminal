/*
 * AHT20.c
 *
 *      Author: yu_ha
 */

#include "AHT20.h"

#define AHT20_COMD_STATUS_LEN	1
#define AHT20_COMD_INIT_LEN 	3
#define AHT20_COMD_MEASURE_LEN 	3
#define AHT20_COMD_RESET_LEN 	1

uint8_t AHT20_COMD_STATUS[] = 	{0x71};
uint8_t AHT20_COMD_INIT[] = 	{0xBE, 0x08, 0x00};
uint8_t AHT20_COMD_MEASURE[] = 	{0xAC, 0x33, 0x00};
uint8_t AHT20_COMD_RESET[] = 	{0xBA};

void __AHT20_unlock(struct AHT20_handle_s *handle) {
	uint64_t start_time = bflb_mtimer_get_time_ms();
	while (bflb_mtimer_get_time_ms() - start_time < AHT20_MAX_WAITING_TIME && !handle->free); // Maximum waiting time 100ms
	if (!handle->free) {
		LOG_E("%s: Waiting too long for unlocking on 0x%.2X of %s\r\n", handle->name, handle->addr, handle->device->name);
		return;
	}
	handle->free = 0;
}

void AHT20_init(struct AHT20_handle_s *handle, struct bflb_device_s *device, uint16_t addr, char *name) {
	handle->free = 0;
	handle->addr = addr;
	handle->device = device;
	strncpy(handle->name, name, 21);
	handle->name[20] = 0;
	handle->free = 1;
}

void AHT20_humidity_temperature(struct AHT20_handle_s *handle, float *humidity, float *temperature) {
	uint64_t start_time;
	struct bflb_i2c_msg_s msgs[2];
	uint8_t read_data[6];

	__AHT20_unlock(handle);

	start_time = bflb_mtimer_get_time_ms();
	while (bflb_mtimer_get_time_ms() - start_time < AHT20_MAX_WAITING_TIME) {
		msgs[0].addr = handle->addr;
	    msgs[0].flags = I2C_M_NOSTOP;
	    msgs[0].buffer = AHT20_COMD_STATUS;
	    msgs[0].length = AHT20_COMD_STATUS_LEN;
		msgs[1].addr = handle->addr;
	    msgs[1].flags = I2C_M_READ;
	    msgs[1].buffer = read_data;
	    msgs[1].length = 1;
	    bflb_i2c_transfer(handle->device, msgs, 2);
	    if (!(read_data[0] & (1 << 3))) {
	    	LOG_I("AHT20 havn't calibrated. Calibrating ...");
	    	msgs[0].addr = handle->addr;
	    	msgs[0].flags = 0;
	        msgs[0].buffer = AHT20_COMD_INIT;
	        msgs[0].length = AHT20_COMD_INIT_LEN;
	        bflb_i2c_transfer(handle->device, msgs, 1);
	        bflb_mtimer_delay_ms(10);
	    }
	    else break;
	}

	if (!(read_data[0] & (1 << 3))) {
		LOG_E("%s: Waiting too long for calibrating on 0x%.2X of %s\r\n", handle->name, handle->addr, handle->device->name);
		handle->free = 1;
		return;
	}

	msgs[0].addr = handle->addr;
	msgs[0].flags = 0;
	msgs[0].buffer = AHT20_COMD_MEASURE;
	msgs[0].length = AHT20_COMD_MEASURE_LEN;
	bflb_i2c_transfer(handle->device, msgs, 1);
    bflb_mtimer_delay_ms(80);

	msgs[0].addr = handle->addr;
    msgs[0].flags = I2C_M_NOSTOP;
    msgs[0].buffer = AHT20_COMD_STATUS;
    msgs[0].length = AHT20_COMD_STATUS_LEN;
	msgs[1].addr = handle->addr;
    msgs[1].flags = I2C_M_READ;
    msgs[1].buffer = read_data;
    msgs[1].length = 1;
    bflb_i2c_transfer(handle->device, msgs, 2);
    if (read_data[0] & (1 << 7)) {
    	bflb_mtimer_delay_ms(80);
        bflb_i2c_transfer(handle->device, msgs, 2);
        if (read_data[0] & (1 << 7)) {
        	LOG_E("%s: Waiting too long for measuring on 0x%.2X of %s\r\n", handle->name, handle->addr, handle->device->name);
        	handle->free = 1;
        	return;
        }
    }

    msgs[0].addr = handle->addr;
	msgs[0].flags = I2C_M_NOSTOP;
	msgs[0].buffer = AHT20_COMD_STATUS;
	msgs[0].length = AHT20_COMD_STATUS_LEN;
	msgs[1].addr = handle->addr;
	msgs[1].flags = I2C_M_READ;
	msgs[1].buffer = read_data;
	msgs[1].length = 6;
    bflb_i2c_transfer(handle->device, msgs, 2);

    *humidity = (float)(((uint32_t)read_data[1] << 12) + ((uint32_t)read_data[2] << 4) + ((read_data[3] & 0xf0) >> 4)) / (1 << 20) * 100;
    *temperature = (float)((((uint32_t)read_data[3] & 0x0f) << 16) + ((uint32_t)read_data[4] << 8) + read_data[5]) / (1 << 20) * 200 - 50;

    __AHT20_release(handle);
}

void AHT20_reset(struct AHT20_handle_s *handle) {
	struct bflb_i2c_msg_s msgs[1];

	__AHT20_unlock(handle);

	msgs[0].addr = handle->addr;
	msgs[0].flags = 0;
    msgs[0].buffer = AHT20_COMD_RESET;
    msgs[0].length = AHT20_COMD_RESET_LEN;
    bflb_i2c_transfer(handle->device, msgs, 1);
    bflb_mtimer_delay_ms(20);

    __AHT20_release(handle);
}
