/*
 * DS18B20.c
 *
 *      Author: yu_ha
 */

#include "DS18B20.h"

extern struct bflb_device_s *gpio;

void __DS18B20_unlock(struct DS18B20_handle_s *handle) {
	uint64_t start_time = bflb_mtimer_get_time_ms();
	while (bflb_mtimer_get_time_ms() - start_time < DS18B20_MAX_WAITING_TIME && !handle->free); // Maximum waiting time 100ms
	if (!handle->free) {
		LOG_E("%s: Waiting too long for unlocking on GPIO_PIN_%d\r\n", handle->name, handle->pin);
		return;
	}
	handle->free = 0;
}

void DS18B20_init(struct DS18B20_handle_s *handle, uint8_t pin, char *name) {
	handle->free = 0;
	handle->pin = pin;
	strncpy(handle->name, name, 21);
	handle->name[20] = 0;
	handle->free = 1;
}

void __DS18B20_reset(struct DS18B20_handle_s *handle) {
	DS18B20_IO_OUT(handle);
	DS18B20_IO_RESET(handle);
	bflb_mtimer_delay_us(750);
	DS18B20_IO_SET(handle);
	bflb_mtimer_delay_us(15);
}

uint8_t DS18B20_check(struct DS18B20_handle_s *handle) {
	uint8_t ret;
	__DS18B20_unlock(handle);
	ret = __DS18B20_check(handle);
	__DS18B20_release(handle);
	return ret;
}

uint8_t __DS18B20_check(struct DS18B20_handle_s *handle) {
	uint64_t start_time;

	DS18B20_IO_IN(handle);
	start_time = bflb_mtimer_get_time_us();
	while (bflb_mtimer_get_time_us() - start_time < 200 && DS18B20_IO_READ(handle));
	if (DS18B20_IO_READ(handle)) return 1;

	start_time = bflb_mtimer_get_time_us();
	while (bflb_mtimer_get_time_us() - start_time < 240 && !DS18B20_IO_READ(handle));
	if (!DS18B20_IO_READ(handle)) return 1;

	return 0;
}

void DS18B20_temperature(struct DS18B20_handle_s *handle, float *temperature) {
	uint8_t TL, TH;

	__DS18B20_unlock(handle);

	__DS18B20_reset(handle);
	if (__DS18B20_check(handle) != 0) {
		LOG_E("%s: Not found on GPIO_PIN_%d\r\n", handle->name, handle->pin);
		return;
	}

	__DS18B20_write_byte(handle, 0xCC);
	__DS18B20_write_byte(handle, 0x44);

	__DS18B20_reset(handle);
	if (__DS18B20_check(handle) != 0) {
		LOG_E("%s: Not found on GPIO_PIN_%d\r\n", handle->name, handle->pin);
		return;
	}

	__DS18B20_write_byte(handle, 0xCC);
	__DS18B20_write_byte(handle, 0xBE);

	TL = __DS18B20_read_byte(handle);
	TH = __DS18B20_read_byte(handle);
	if (TH > 8) *temperature = -(float)(((~TH) << 8) | (~TL)) * 0.0625;
	else *temperature = (float)((TH << 8) | TL) * 0.0625;

	__DS18B20_release(handle);
}

void __DS18B20_write_byte(struct DS18B20_handle_s *handle, uint8_t date) {
	uint8_t i;

	DS18B20_IO_OUT(handle);
	for (i = 0; i < 8; ++i) {
		if (date & 1) {
			DS18B20_IO_RESET(handle);
			bflb_mtimer_delay_us(2);
			DS18B20_IO_SET(handle);
			bflb_mtimer_delay_us(60);
		}
		else {
			DS18B20_IO_RESET(handle);
			bflb_mtimer_delay_us(60);
			DS18B20_IO_SET(handle);
			bflb_mtimer_delay_us(2);
		}
		date >>= 1;
	}
}

uint8_t __DS18B20_read_byte(struct DS18B20_handle_s *handle) {
	uint8_t i;
	uint8_t ret = 0;

	for (i = 0; i < 8; ++i) {
		DS18B20_IO_OUT(handle);
		DS18B20_IO_RESET(handle);
		bflb_mtimer_delay_us(2);
		DS18B20_IO_SET(handle);
		DS18B20_IO_IN(handle);
		bflb_mtimer_delay_us(12);
		ret = (DS18B20_IO_READ(handle) << 7) | (ret >> 1);
		bflb_mtimer_delay_us(50);
	}
	return ret;
}
