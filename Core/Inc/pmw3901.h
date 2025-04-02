/*
 * pmw3901.h
 *
 *  Created on: Apr 2, 2025
 *      Author: lapchong
 */

#ifndef INC_PMW3901_H_
#define INC_PMW3901_H_

#include <stdbool.h>

bool pmw3901_begin(void);
void readMotionCount(int16_t *deltaX, int16_t *deltaY);
void enableFrameBuffer();
void readFrameBuffer(char *FBuffer);
void initRegisters();
void setLed(bool ledOn);
uint8_t pmw3901_register_read(uint8_t reg);
void pmw3901_register_write(uint8_t reg, uint8_t value);

#endif /* INC_PMW3901_H_ */
