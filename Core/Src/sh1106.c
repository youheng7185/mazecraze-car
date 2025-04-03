/*
 * sh1106.c
 *
 *  Created on: Feb 12, 2025
 *      Author: lapchong
 */


#include "main.h"
#include "sh1106.h"
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "font.h"

#define OLED_ADDR 0x3C

extern I2C_HandleTypeDef hi2c1;

bool sendCommand(uint8_t cmd)
{
	uint8_t stuff[2] = {0x80, cmd};
	return (HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR << 1, stuff, 2, HAL_MAX_DELAY) == HAL_OK);
}

bool sendData(uint8_t data)
{
	uint8_t stuff[2] = {0x40, data};
	return (HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR << 1, stuff, 2, HAL_MAX_DELAY) == HAL_OK);
}

uint8_t oled64_initbuf[]={0x00,0xae,0xa8,0x3f,0xd3,0x00,0x40,0xa1,0xc8,
      0xda,0x12,0x81,0xff,0xa4,0xa6,0xd5,0x80,0x8d,0x14,
      0xaf,0x20,0x02};

bool sh1106_init() {
	for (uint8_t i = 0; i < sizeof(oled64_initbuf); i++)
	{
		if(!sendCommand(oled64_initbuf[i]))
		{
			return false;
		}
	}
	return true;
}

uint8_t framebuffer[8][128] = {0};

void sh1106_sendBuffer()
{
	for (uint8_t page = 0; page < 8; page++)
	{
		sendCommand(0xB0 | page);

		sendCommand(0x02);
		sendCommand(0x10);

		uint8_t buf[129];
		buf[0] = 0x40;

		memcpy(&buf[1], framebuffer[page], 128);

		HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR << 1, buf, sizeof(buf), HAL_MAX_DELAY);
	}
}

void sh1106_drawPixel(uint8_t pos_x, uint8_t pos_y, uint8_t colour) {
	uint8_t page = pos_y / 8;
	uint8_t bit_pos = pos_y % 8;

	framebuffer[page][pos_x] |= (colour << bit_pos);
}

void sh1106_setAll(uint8_t colour)
{
	switch(colour)
	{
		case 1:
			memset(framebuffer, 0xFF, sizeof(framebuffer));
			break;
		case 0:
			memset(framebuffer, 0x00, sizeof(framebuffer));
			break;
	}
	//sh1106_sendBuffer();
}

void sh1106_drawRec(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
	for (int i = x0; i <= x1; i++)
	{
	  for (int j = y0; j <= y1; j++)
	  {
		  sh1106_drawPixel(i, j, 1);
	  }
	}
}

void sh1106_drawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (1) {
        sh1106_drawPixel(x0, y0, 1); // Draw the current pixel

        // Break if the end point is reached
        if (x0 == x1 && y0 == y1) break;

        // Calculate error term and adjust coordinates
        int e2 = err * 2;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void sh1106_drawCircle(uint8_t x0, uint8_t y0, uint8_t radius) {
    int x = 0;
    int y = radius;
    int d = 1 - radius;

    while (x <= y) {
        // Draw symmetric points in all 8 octants
        sh1106_drawPixel(x0 + x, y0 + y, 1);
        sh1106_drawPixel(x0 - x, y0 + y, 1);
        sh1106_drawPixel(x0 + x, y0 - y, 1);
        sh1106_drawPixel(x0 - x, y0 - y, 1);
        sh1106_drawPixel(x0 + y, y0 + x, 1);
        sh1106_drawPixel(x0 - y, y0 + x, 1);
        sh1106_drawPixel(x0 + y, y0 - x, 1);
        sh1106_drawPixel(x0 - y, y0 - x, 1);

        x++;

        // Check the decision parameter and update accordingly
        if (d < 0) {
            d += 2 * x + 1;
        } else {
            y--;
            d += 2 * (x - y) + 1;
        }
    }
}

void sh1106_drawChar(uint8_t x, uint8_t y, char c) {
	if (c < 32 || c > 126) return;
    uint8_t charIndex = (c - 32);

    for (uint8_t row = 0; row < 5; row++)
    {
        for (uint8_t col = 0; col < 7; col++)
        {
        	uint8_t fontByte = fontdata[charIndex][col];
            //if (fontByte & (1 << row))
        	if (fontByte & (1 << (4 - row)))
            {
                sh1106_drawPixel(x + row, y + col, 1);
            } else {
            	sh1106_drawPixel(x + row, y + col, 0);
            }
        }
    }

}

void sh1106_print(uint8_t x, uint8_t y, const char* str)
{
	while(*str)
	{
		sh1106_drawChar(x, y, *str++);
		x+=6;
		if(x > 122)
		{
			y+=8;
			x=0;
		}
	}
}

void sh1106_drawPixelFast(uint8_t pos_x, uint8_t pos_y, uint8_t colour)
{
	uint8_t page = pos_y / 8;
	uint8_t bit_pos = pos_y % 8;

	framebuffer[page][pos_x] |= (colour << bit_pos);

	sendCommand(0xB0 | page);

	sendCommand(0x00 | ((pos_x) & 0x0F));
	sendCommand(0x10 | ((pos_x) >> 4));

	sendData(framebuffer[page][pos_x]);

}
