/*
 * sh1106.h
 *
 *  Created on: Feb 12, 2025
 *      Author: lapchong
 */

#ifndef INC_SH1106_H_
#define INC_SH1106_H_

#include <stdbool.h>

bool sh1106_init();
void sh1106_sendBuffer();
void sh1106_drawPixel(uint8_t pos_x, uint8_t pos_y, uint8_t colour);
void sh1106_setAll(uint8_t colour);
void sh1106_drawRec(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void sh1106_drawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void sh1106_drawCircle(uint8_t x0, uint8_t y0, uint8_t radius);
void sh1106_drawChar(uint8_t x, uint8_t y, char c);
void sh1106_print(uint8_t x, uint8_t y, const char* str);
void sh1106_drawPixelFast(uint8_t pos_x, uint8_t pos_y, uint8_t colour);

#endif /* INC_SH1106_H_ */
