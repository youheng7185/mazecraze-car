/*
 * tca9548a.c
 *
 *  Created on: Mar 26, 2025
 *      Author: lapchong
 */
#include "stm32f4xx_hal.h"
#include "main.h"
#include "tca9548.h"

void selectTCAChannel(uint8_t channel) {
    uint8_t cmd = (1 << channel);  // Enable only the selected channel
    HAL_I2C_Master_Transmit(&hi2c1, 0x77 << 1, &cmd, 1, HAL_MAX_DELAY);
}
