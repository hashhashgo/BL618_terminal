/*
 * SOIL5.h
 *
 *  Created on: May 27, 2023
 *      Author: yu_ha
 */

/*
 * 寄存器地址	PLC或组态地址	内容	操作	定义说明
 * 0000 H	40001 (十进制)	含水率	只读	含水率实时值（扩大10倍）
 * 0001 H	40002 (十进制)	温度值	只读	温度实时值（扩大10倍）
 * 0002 H	40003 (十进制)	电导率	只读	电导率实时值
 * 0003 H	40004 (十进制)	PH值	只读	PH实时值（扩大十倍）
 * 0004 H	40005(十进制)	氮含量暂存值	读写	被写入的氮含量值或测试值1
 * 0005 H	40006(十进制)	磷含量暂存值	读写	被写入的磷含量值或测试值2
 * 0006 H	40007(十进制)	钾含量暂存值	读写	被写入的钾含量值或测试值3
 * 0007 H	40008(十进制)	盐度	只读	盐度实时值（仅供参考）
 * 0008 H	40009 (十进制)	总溶解固体 TDS	只读	TDS实时值（仅供参考）
 * 0022 H	40035 (十进制)	电导温度系数	读写	0-100对应0.0%-10.0%
 * 默认0.0%
 * 0023 H	40036 (十进制)	盐度系数	读写	0-100 对应 0.00-1.00
 * 默认55（0.55）
 * 0024 H	40037 (十进制)	TDS 系数	读写	0-100 对应 0.00-1.00
 * 默认50（0.5）
 * 0050 H	40081 (十进制)	温度校准值	读写	整数（扩大10倍）
 * 0051 H	40082 (十进制)	含水率校准值	读写	整数（扩大10倍）
 * 0052 H	40083 (十进制)	电导率校准值	读写	整数
 * 0053 H	40083 (十进制)	PH校准值	读写	整数
 * 04E8 H	41001 (十进制)	氮含量暂存值系数高十六位	读写	浮点数
 * （IEEE754标准 浮点型）
 * 04E9 H	41002 (十进制)	氮含量暂存值系数低十六位	读写
 * 04EA H	41003 (十进制)	氮含量暂存值的偏差值	读写	整数
 * 04F2 H	41011 (十进制)	磷含量暂存值系数高十六位	读写	浮点数
 * （IEEE754标准 浮点型）
 * 04F3 H	41012 (十进制)	磷含量暂存值系数低十六位	读写
 * 04F4 H	41013 (十进制)	磷含量暂存值的偏差值	读写	整数
 * 04FC H	41021 (十进制)	钾含量暂存值系数高十六位	读写	浮点数
 * （IEEE754标准 浮点型）
 * 04FD H	41022 (十进制)	钾含量暂存值系数低十六位	读写
 * 04FE H	41023 (十进制)	钾含量暂存值的偏差值	读写	整数
 * 07D0 H	42001 (十进制)	设备地址	读写	1~254（出厂默认1）
 * 07D1 H	42002 (十进制)	设备波特率	读写	0代表2400
 * 1代表4800
 * 2代表9600
 * 1：0004H寄存器未执行写入操作时，寄存器内数值为f1(电导率测量值)，0004H寄存器被执行写入操作后，寄存器存储写入值。
 * 2：0005H寄存器未执行写入操作时，寄存器内数值为f2(电导率测量值)，0005H寄存器被执行写入操作后，寄存器存储写入值。
 * 3：0006H寄存器未执行写入操作时，寄存器内数值为f3(电导率测量值)，0006H寄存器被执行写入操作后，寄存器存储写入值。
 */

#ifndef EXAMPLES_FINAL_SENSOR_SOIL5_H_
#define EXAMPLES_FINAL_SENSOR_SOIL5_H_

#include "bflb_core.h"
#include "bflb_mtimer.h"
#include "bflb_uart.h"
#include "log.h"

#define SOIL5_MAX_WAITING_TIME 100

enum SOIL5_REG {
	SOIL5_WATER				= 0x0000,	// 含水率
	SOIL5_TEMPERATURE		= 0x0001,	// 温度值
	SOIL5_CONDUCTIVITY		= 0x0002,	// 电导率
	SOIL5_PH				= 0x0003,	// PH值
	SOIL5_N_TMP				= 0x0004,	// 氮含量暂存值
	SOIL5_P_TMP				= 0x0005,	// 磷含量暂存值
	SOIL5_K_TMP				= 0x0006,	// 钾含量暂存值
	SOIL5_SALT				= 0x0007,	// 盐度
	SOIL5_TDS				= 0x0008,	// 总溶解固体 TDS

	SOIL5_COND_TEMP_FAC		= 0x0022,	// 电导温度系数
	SOIL5_SALT_FAC			= 0x0023,	// 盐度系数
	SOIL5_TDS_FAC			= 0x0024,	// TDS系数

	SOIL5_TEMP_CALI			= 0x0050,	// 温度校准值
	SOIL5_WATER_CALI		= 0x0051,	// 含水率校准值
	SOIL5_COND_CALI			= 0x0052,	// 电导率校准值
	SOIL5_PH_CALI			= 0x0053,	// PH校准值

	SOIL5_N_TMP_FAC_H		= 0x04E8,	// 氮含量暂存值系数高十六位
	SOIL5_N_TMP_FAC_L		= 0x04E9,	// 氮含量暂存值系数低十六位
	SOIL5_N_TMP_OFFSET		= 0x04EA,	// 氮含量暂存值的偏差值
	SOIL5_P_TMP_FAC_H		= 0x04F2,	// 磷含量暂存值系数高十六位
	SOIL5_P_TMP_FAC_L		= 0x04F3,	// 磷含量暂存值系数低十六位
	SOIL5_P_TMP_OFFSET		= 0x04F4,	// 磷含量暂存值的偏差值
	SOIL5_K_TMP_FAC_H		= 0x04FC,	// 钾含量暂存值系数高十六位
	SOIL5_K_TMP_FAC_L		= 0x04FD,	// 钾含量暂存值系数低十六位
	SOIL5_K_TMP_OFFSET		= 0x04FE,	// 钾含量暂存值的偏差值

	SOIL5_DEVICE_ADDR		= 0x07D0,	// 设备地址
	SOIL5_DEVICE_BAUD		= 0x07D1,	// 设备波特率
};

struct SOIL5_handle_s {
	char name[21];
	struct bflb_device_s *device;
	uint8_t addr;
	uint8_t free;
};

void __SOIL5_unlock(struct SOIL5_handle_s *handle);
#define __SOIL5_release(handle) {handle->free = 1;}

uint16_t __crc_16_modbus(uint8_t *data, uint16_t len);

void SOIL5_init(struct SOIL5_handle_s *handle, struct bflb_device_s *device, uint8_t addr, char *name);
uint16_t SOIL5_read(struct SOIL5_handle_s *handle, uint16_t reg);
void SOIL5_write(struct SOIL5_handle_s *handle, uint16_t reg, uint16_t data);

#endif /* EXAMPLES_FINAL_SENSOR_SOIL5_H_ */
