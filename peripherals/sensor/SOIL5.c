/*
 * SOIL5.c
 *
 *  Created on: May 27, 2023
 *      Author: yu_ha
 */

#include "SOIL5.h"


void __SOIL5_unlock(struct SOIL5_handle_s *handle) {
	uint64_t start_time = bflb_mtimer_get_time_ms();
	while (bflb_mtimer_get_time_ms() - start_time < SOIL5_MAX_WAITING_TIME && !handle->free); // Maximum waiting time 100ms
	if (!handle->free) {
		LOG_E("%s: Waiting too long for unlocking on %s\r\n", handle->name, handle->device->name);
		return;
	}
	handle->free = 0;
}

uint16_t __crc_16_modbus(uint8_t *data, uint16_t len) {
	uint8_t j;
	uint16_t i;
	uint16_t crc = 0xffff;

	for (i = 0; i < len; ++i) {
		crc ^= data[i];
		for (j = 0; j < 8; ++j) {
			if (crc & 1) crc = (crc >> 1) ^ 0xA001;
			else crc >>= 1;
		}
	}
	return crc;
}

void SOIL5_init(struct SOIL5_handle_s *handle, struct bflb_device_s *device, uint8_t addr, char *name) {
	handle->free = 0;
	handle->device = device;
	strncpy(handle->name, name, 21);
	handle->name[20] = 0;
	handle->addr = addr;
	handle->free = 1;
}

uint16_t SOIL5_read(struct SOIL5_handle_s *handle, uint16_t reg) {
	uint8_t i;
	uint16_t crc;
	uint64_t start_time;
	uint8_t data[20] = {handle->addr, 0x03, (reg & 0xff00) >> 8, reg & 0x00ff, 0x00, 0x01, 0x00, 0x00};

	__SOIL5_unlock(handle);

	crc = __crc_16_modbus(data, 6);
	data[7] = (crc & 0xff00) >> 8;
	data[6] = crc & 0x00ff;

	bflb_mtimer_delay_ms(20);

	for (i = 0; i < 8; ++i) bflb_uart_putchar(handle->device, data[i]);

	bflb_mtimer_delay_ms(20);

	start_time = bflb_mtimer_get_time_ms();
	for (i = 0; i < 3 && bflb_mtimer_get_time_ms() - start_time < SOIL5_MAX_WAITING_TIME; i = (data[i] != 255) ? i + 1 : i)
		data[i] = bflb_uart_getchar(handle->device);

	for (; i < data[2] + 5 && bflb_mtimer_get_time_ms() - start_time < SOIL5_MAX_WAITING_TIME; i = (data[i] != 255) ? i + 1 : i)
		data[i] = bflb_uart_getchar(handle->device);

	if (i < data[2] + 5) {
		LOG_E("%s: UART read error on %s\r\n", handle->name, handle->device->name);
		for (; i != 255; --i) {
			LOG_E("%s: data[%d] %d\r\n", handle->name, i, data[i]);
		}
		__SOIL5_release(handle);
		return 0xffff;
	}

	crc = __crc_16_modbus(data, data[2] + 3);
	if (data[data[2] + 4] != ((crc & 0xff00) >> 8) || data[data[2] + 3] != (crc & 0x00ff)) {
		LOG_E("%s: CRC error\r\n", handle->name);
		__SOIL5_release(handle);
		return 0xffff;
	}

	__SOIL5_release(handle);
	return (data[3] << 8) | data[4];
}

void SOIL5_write(struct SOIL5_handle_s *handle, uint16_t reg, uint16_t d) {
	uint8_t i, flag = 1;
	uint16_t crc;
	uint64_t start_time;
	uint8_t data[] = {handle->addr, 0x03, (reg & 0xff00) >> 8, reg & 0x00ff, (d & 0xff00) >> 8, d & 0x00ff, 0x00, 0x00};

	__SOIL5_unlock(handle);

	crc = __crc_16_modbus(data, 6);
	data[7] = (crc & 0xff00) >> 8;
	data[6] = crc & 0x00ff;

	bflb_mtimer_delay_ms(20);

	for (i = 0; i < 8; ++i) bflb_uart_putchar(handle->device, data[i]);

	bflb_mtimer_delay_ms(20);

	start_time = bflb_mtimer_get_time_ms();
	for (i = 0; i < 8 && bflb_mtimer_get_time_ms() - start_time < SOIL5_MAX_WAITING_TIME; i = (data[i] != -1) ? i + 1 : i)
		if (data[i] != bflb_uart_getchar(handle->device)) flag = 0;

	if (i < 8) {
		LOG_E("%s: UART read error on %s\r\n", handle->name, handle->device->name);
		__SOIL5_release(handle);
		return;
	}

	if (!flag) {
		LOG_E("%s: reg write error\r\n", handle->name);
		__SOIL5_release(handle);
		return;
	}

	__SOIL5_release(handle);
}
