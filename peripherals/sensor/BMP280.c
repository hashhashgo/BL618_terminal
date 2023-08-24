/*
 * BMP280.c
 *
 *      Author: yu_ha
 */

#include "BMP280.h"

#define BMP280_COMD_INIT_LEN 1

uint8_t BMP280_COMD_INIT[] = {0x88};
uint8_t BMP280_TEMP_REG[] = {0xF7};

void __BMP280_unlock(struct BMP280_handle_s *handle) {
	uint64_t start_time = bflb_mtimer_get_time_ms();
	while (bflb_mtimer_get_time_ms() - start_time < BMP280_MAX_WAITING_TIME && !handle->free); // Maximum waiting time 100ms
	if (!handle->free) {
		LOG_E("%s: Waiting too long for unlocking on 0x%.2X of %s\r\n", handle->name, handle->addr, handle->device->name);
		return;
	}
	handle->free = 0;
}

void BMP280_init(struct BMP280_handle_s *handle, struct bflb_device_s *device, uint16_t addr, char *name) {
	struct bflb_i2c_msg_s msgs[2];
	uint8_t read_data[24];

	handle->free = 0;
	handle->addr = addr;
	handle->device = device;
	strncpy(handle->name, name, 21);
	handle->name[20] = 0;

	msgs[0].addr = handle->addr;
	msgs[0].flags = I2C_M_NOSTOP;
	msgs[0].buffer = BMP280_COMD_INIT;
	msgs[0].length = BMP280_COMD_INIT_LEN;
	msgs[1].addr = handle->addr;
	msgs[1].flags = I2C_M_READ;
	msgs[1].buffer = read_data;
	msgs[1].length = 24;
    bflb_i2c_transfer(handle->device, msgs, 2);

    handle->T[0] = ((uint16_t)read_data[1] << 8) + read_data[0];
    handle->T[1] = ((uint16_t)read_data[3] << 8) + read_data[2];
    if (handle->T[1] > 32767) handle->T[1] -= 65536;
    handle->T[2] = ((uint16_t)read_data[5] << 8) + read_data[4];
    if (handle->T[2] > 32767) handle->T[2] -= 65536;

    handle->P[0] = ((uint16_t)read_data[7] << 8) + read_data[6];
    handle->P[1] = ((uint16_t)read_data[9] << 8) + read_data[8];
    if (handle->P[1] > 32767) handle->P[1] -= 65536;
    handle->P[2] = ((uint16_t)read_data[11] << 8) + read_data[10];
    if (handle->P[2] > 32767) handle->P[2] -= 65536;
    handle->P[3] = ((uint16_t)read_data[13] << 8) + read_data[12];
    if (handle->P[3] > 32767) handle->P[3] -= 65536;
    handle->P[4] = ((uint16_t)read_data[15] << 8) + read_data[14];
    if (handle->P[4] > 32767) handle->P[4] -= 65536;
    handle->P[5] = ((uint16_t)read_data[17] << 8) + read_data[16];
    if (handle->P[5] > 32767) handle->P[5] -= 65536;
    handle->P[6] = ((uint16_t)read_data[19] << 8) + read_data[18];
    if (handle->P[6] > 32767) handle->P[6] -= 65536;
    handle->P[7] = ((uint16_t)read_data[21] << 8) + read_data[20];
    if (handle->P[7] > 32767) handle->P[7] -= 65536;
    handle->P[8] = ((uint16_t)read_data[23] << 8) + read_data[22];
    if (handle->P[8] > 32767) handle->P[8] -= 65536;

	handle->free = 1;
}

void BMP280_temperature_pressure(struct BMP280_handle_s *handle, float *temperature, float *pressure) {
	struct bflb_i2c_msg_s msgs[2];
	uint8_t read_data[8];

	float adc_t, var1, var2, t_fine, adc_p, p;

	__BMP280_unlock(handle);

	__BMP280_set_register(handle, 0x27, 0xA0);

	/*
	 * BMP280 address, self.i2c_addr(118)
     * Read data back from 0xF7(247), 8 bytes
     * Pressure MSB, Pressure LSB, Pressure xLSB, Temperature MSB, Temperature LSB
     * Temperature xLSB, Humidity MSB, Humidity LSB
	 */
	msgs[0].addr = handle->addr;
	msgs[0].flags = I2C_M_NOSTOP;
	msgs[0].buffer = BMP280_TEMP_REG;
	msgs[0].length = 1;
	msgs[1].addr = handle->addr;
	msgs[1].flags = I2C_M_READ;
	msgs[1].buffer = read_data;
	msgs[1].length = 8;
	bflb_i2c_transfer(handle->device, msgs, 2);

	// Convert temperature data to 19-bits
	adc_t = ((read_data[3] << 16) + (read_data[4] << 8) + (read_data[5] & 0xF0)) >> 4;

	// Temperature offset calculations
	var1 = (adc_t / 16384.0 - handle->T[0] / 1024.0) * handle->T[1];
	var2 = ((adc_t / 131072.0 - handle->T[0] / 8192.0) * (adc_t / 131072.0 - handle->T[0] / 8192.0)) * handle->T[2];
	t_fine = var1 + var2;
	*temperature = t_fine / 5120.0;

	// Convert pressure and temperature data to 19-bits
	adc_p = ((1.0 * read_data[0] * 65536) + (1.0 * read_data[1] * 256) + (read_data[2] & 0xF0)) / 16;

	// Pressure offset calculations
	var1 = (t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * handle->P[5] / 32768.0;
	var2 = var2 + var1 * handle->P[4] * 2.0;
	var2 = (var2 / 4.0) + (handle->P[3] * 65536.0);
	var1 = (handle->P[2] * var1 * var1 / 524288.0 + handle->P[1] * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * handle->P[0];
	p = 1048576.0 - adc_p;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = handle->P[8] * p * p / 2147483648.0;
	var2 = p * handle->P[7] / 32768.0;
	*pressure = (p + (var1 + var2 + handle->P[6]) / 16.0) / 100;

	__BMP280_release(handle);
}

void __BMP280_set_register(struct BMP280_handle_s *handle, uint8_t oversampling, uint8_t standby) {
	struct bflb_i2c_msg_s msgs[1];
	uint8_t os_data[] = {0xF4, oversampling};
	uint8_t sb_data[] = {0xF5, standby};

	/*
	 * BMP280 address, self.i2c_addr(118)
	 * Select Control measurement register, 0xF4(244)
	 * 		0x27(39)	Pressure and Temperature Oversampling rate = 1
	 * 					Normal mode
	 */
	msgs[0].addr = handle->addr;
	msgs[0].flags = 0;
	msgs[0].buffer = os_data;
	msgs[0].length = 2;
    bflb_i2c_transfer(handle->device, msgs, 1);

    /*
     *  BMP280 address, self.i2c_addr(118)
     * Select Configuration register, 0xF5(245)
     * 		0xA0(00)	Stand_by time = 1000 ms
     */
	msgs[0].addr = handle->addr;
	msgs[0].flags = 0;
	msgs[0].buffer = sb_data;
	msgs[0].length = 2;
    bflb_i2c_transfer(handle->device, msgs, 1);

    bflb_mtimer_delay_ms(500);
}
