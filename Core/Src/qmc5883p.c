/*
 * qmc5883p.c
 *
 *  Created on: Apr 5, 2025
 *      Author: lapchong
 */

#include "main.h"
#include "stm32f4xx_hal.h"
#include "qmc5883p.h"
#include <stdint.h>
#include <stdbool.h>
#include "my_print.h"

#define QMC5883_I2C_ADDR 0x2C

bool qmc5883p_write(uint8_t reg, uint8_t data)
{
	uint8_t tx_buffer[2] = {reg, data};
	return (HAL_I2C_Master_Transmit(&hi2c2, QMC5883_I2C_ADDR << 1, tx_buffer, 2, HAL_MAX_DELAY) == HAL_OK);
}

bool qmc5883p_read(uint8_t reg, uint8_t *data)
{
	if (HAL_I2C_Master_Transmit(&hi2c2, QMC5883_I2C_ADDR << 1, &reg, 1, HAL_MAX_DELAY) != HAL_OK) {
		return false;
	}
	if (HAL_I2C_Master_Receive(&hi2c2, QMC5883_I2C_ADDR << 1, data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return false;
	}
	return true;
}

bool qmc5883p_init()
{
	if(HAL_I2C_IsDeviceReady(&hi2c2, (QMC5883_I2C_ADDR << 1), 1, 10) != HAL_OK)
	{
		my_printf("qmc not found\r\n");
		return false;
	}

	uint8_t dev_id = 0;
	qmc5883p_read(0x00, &dev_id);
	my_printf("qmc dev id: 0x%02X\r\n", dev_id);

	qmc5883p_write(0x29, 0x06);
	qmc5883p_write(0x0B, 0x08);
	qmc5883p_write(0x0A, 0xC3);
	return true;
}

void qmc5883p_read_all(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z)
{
	uint8_t status = 0;
	qmc5883p_read(0x09, &status);
	if (status & 0x01)
	{
		uint8_t reg_value = 0x01;
		HAL_I2C_Master_Transmit(&hi2c2, QMC5883_I2C_ADDR << 1, &reg_value, 1, HAL_MAX_DELAY);
		uint8_t rx_buffer[6];
		HAL_I2C_Master_Receive(&hi2c2, QMC5883_I2C_ADDR << 1, rx_buffer, 6, HAL_MAX_DELAY);
		*mag_x = (rx_buffer[1] << 8) | (rx_buffer[0]);
		*mag_y = (rx_buffer[3] << 8) | (rx_buffer[2]);
		*mag_z = (rx_buffer[5] << 8) | (rx_buffer[4]);
	} else {
		my_printf("mag data not ready\r\n");
	}
}

