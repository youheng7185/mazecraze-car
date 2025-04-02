/* PMW3901 Arduino driver
 * Copyright (c) 2017 Bitcraze AB
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "stm32f4xx_hal.h" // Change this include according to your STM32 series
#include "pmw3901.h"
#include "main.h"
#include <stdint.h>

#define CHIP_ID         0x49  // 01001001
#define CHIP_ID_INVERSE 0xB6  // 10110110

#define CS_PORT GPIOA  // Change to your CS GPIO port
#define CS_PIN  GPIO_PIN_4  // Change to your CS GPIO pin
#define RST_PORT GPIOA
#define RST_PIN GPIO_PIN_3

bool pmw3901_begin(void) {
	HAL_GPIO_WritePin(RST_PORT, RST_PIN, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(RST_PORT, RST_PIN, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(RST_PORT, RST_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	HAL_Delay(1);

	pmw3901_register_write(0x3A, 0x5A);
	HAL_Delay(5);

	uint8_t chipId = pmw3901_register_read(0x00);
	uint8_t dIpihc = pmw3901_register_read(0x5F);

	if (chipId != CHIP_ID || dIpihc != CHIP_ID_INVERSE) {
	return false;
	}

	pmw3901_register_read(0x02);
	pmw3901_register_read(0x03);
	pmw3901_register_read(0x04);
	pmw3901_register_read(0x05);
	pmw3901_register_read(0x06);
	HAL_Delay(1);

	initRegisters();
	return true;
}

void pmw3901_register_write(uint8_t reg, uint8_t value) {
  reg |= 0x80;
  uint8_t data[2] = {reg, value};

  HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
  //HAL_Delay(1);
  HAL_SPI_Transmit(&hspi1, data, 2, HAL_MAX_DELAY);
  //HAL_Delay(1);
  HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
  //HAL_Delay(2);
}

uint8_t pmw3901_register_read(uint8_t reg) {
  reg &= ~0x80;
  uint8_t value = 0;
  uint8_t txData = reg;

  HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
  //HAL_Delay(1);
  HAL_SPI_Transmit(&hspi1, &txData, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, &value, 1, HAL_MAX_DELAY);
  //HAL_Delay(1);
  HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);

  return value;
}

void readMotionCount(int16_t *deltaX, int16_t *deltaY)
{
  pmw3901_register_read(0x02);
  *deltaX = ((int16_t)pmw3901_register_read(0x04) << 8) | pmw3901_register_read(0x03);
  *deltaY = ((int16_t)pmw3901_register_read(0x06) << 8) | pmw3901_register_read(0x05);
}

void enableFrameBuffer()
{
  pmw3901_register_write(0x7F, 0x07);  //Magic frame readout registers
  pmw3901_register_write(0x41, 0x1D);
  pmw3901_register_write(0x4C, 0x00);
  pmw3901_register_write(0x7F, 0x08);
  pmw3901_register_write(0x6A, 0x38);
  pmw3901_register_write(0x7F, 0x00);
  pmw3901_register_write(0x55, 0x04);
  pmw3901_register_write(0x40, 0x80);
  pmw3901_register_write(0x4D, 0x11);

  pmw3901_register_write(0x70, 0x00);   //More magic?
  pmw3901_register_write(0x58, 0xFF);

  int temp, check;

  do { // keep reading
    temp = pmw3901_register_read(0x58); // the status register
    check = temp>>6; // right shift 6 bits so only top two stay
  } while(check == 0x03); // while bits aren't set denoting ready state
  HAL_Delay(1);
}

void readFrameBuffer(char *FBuffer)
{
  int count = 0;
  uint8_t a; //temp value for reading register
  uint8_t b; //temp value for second register
  uint8_t hold; //holding value for checking bits
  uint8_t mask = 0x0c; //mask to take bits 2 and 3 from b
  uint8_t pixel = 0; //temp holding value for pixel

  for (int ii = 0; ii < 1225; ii++) { //for 1 frame of 1225 pixels (35*35)
    do {
      //if data is either invalid status
      //check status bits 6 and 7
      //if 01 move upper 6 bits into temp value
      //if 00 or 11, reread
      //else lower 2 bits into temp value
      a = pmw3901_register_read(0x58); //read register
      hold = a >> 6; //right shift to leave top two bits for ease of check.
    } while((hold == 0x03) || (hold == 0x00));

    if (hold == 0x01) { //if data is upper 6 bits
      b = pmw3901_register_read(0x58); //read next set to get lower 2 bits
      pixel = a; //set pixel to a
      pixel = pixel << 2; //push left to 7:2
      pixel += (b & mask); //set lower 2 from b to 1:0
      FBuffer[count++] = pixel; //put temp value in fbuffer array
      //delayMicroseconds(100);
    }
  }
  pmw3901_register_write(0x70, 0x00);   //More magic?
  pmw3901_register_write(0x58, 0xFF);

  int temp, check;

  do { //keep reading and testing
    temp = pmw3901_register_read(0x58); //read status register
    check = temp>>6; //rightshift 6 bits so only top two stay
  } while(check == 0x03); //while bits aren't set denoting ready state
}

void initRegisters()
{
  pmw3901_register_write(0x7F, 0x00);
  pmw3901_register_write(0x61, 0xAD);
  pmw3901_register_write(0x7F, 0x03);
  pmw3901_register_write(0x40, 0x00);
  pmw3901_register_write(0x7F, 0x05);
  pmw3901_register_write(0x41, 0xB3);
  pmw3901_register_write(0x43, 0xF1);
  pmw3901_register_write(0x45, 0x14);
  pmw3901_register_write(0x5B, 0x32);
  pmw3901_register_write(0x5F, 0x34);
  pmw3901_register_write(0x7B, 0x08);
  pmw3901_register_write(0x7F, 0x06);
  pmw3901_register_write(0x44, 0x1B);
  pmw3901_register_write(0x40, 0xBF);
  pmw3901_register_write(0x4E, 0x3F);
  pmw3901_register_write(0x7F, 0x08);
  pmw3901_register_write(0x65, 0x20);
  pmw3901_register_write(0x6A, 0x18);
  pmw3901_register_write(0x7F, 0x09);
  pmw3901_register_write(0x4F, 0xAF);
  pmw3901_register_write(0x5F, 0x40);
  pmw3901_register_write(0x48, 0x80);
  pmw3901_register_write(0x49, 0x80);
  pmw3901_register_write(0x57, 0x77);
  pmw3901_register_write(0x60, 0x78);
  pmw3901_register_write(0x61, 0x78);
  pmw3901_register_write(0x62, 0x08);
  pmw3901_register_write(0x63, 0x50);
  pmw3901_register_write(0x7F, 0x0A);
  pmw3901_register_write(0x45, 0x60);
  pmw3901_register_write(0x7F, 0x00);
  pmw3901_register_write(0x4D, 0x11);
  pmw3901_register_write(0x55, 0x80);
  pmw3901_register_write(0x74, 0x1F);
  pmw3901_register_write(0x75, 0x1F);
  pmw3901_register_write(0x4A, 0x78);
  pmw3901_register_write(0x4B, 0x78);
  pmw3901_register_write(0x44, 0x08);
  pmw3901_register_write(0x45, 0x50);
  pmw3901_register_write(0x64, 0xFF);
  pmw3901_register_write(0x65, 0x1F);
  pmw3901_register_write(0x7F, 0x14);
  pmw3901_register_write(0x65, 0x60);
  pmw3901_register_write(0x66, 0x08);
  pmw3901_register_write(0x63, 0x78);
  pmw3901_register_write(0x7F, 0x15);
  pmw3901_register_write(0x48, 0x58);
  pmw3901_register_write(0x7F, 0x07);
  pmw3901_register_write(0x41, 0x0D);
  pmw3901_register_write(0x43, 0x14);
  pmw3901_register_write(0x4B, 0x0E);
  pmw3901_register_write(0x45, 0x0F);
  pmw3901_register_write(0x44, 0x42);
  pmw3901_register_write(0x4C, 0x80);
  pmw3901_register_write(0x7F, 0x10);
  pmw3901_register_write(0x5B, 0x02);
  pmw3901_register_write(0x7F, 0x07);
  pmw3901_register_write(0x40, 0x41);
  pmw3901_register_write(0x70, 0x00);

  HAL_Delay(100);
  pmw3901_register_write(0x32, 0x44);
  pmw3901_register_write(0x7F, 0x07);
  pmw3901_register_write(0x40, 0x40);
  pmw3901_register_write(0x7F, 0x06);
  pmw3901_register_write(0x62, 0xf0);
  pmw3901_register_write(0x63, 0x00);
  pmw3901_register_write(0x7F, 0x0D);
  pmw3901_register_write(0x48, 0xC0);
  pmw3901_register_write(0x6F, 0xd5);
  pmw3901_register_write(0x7F, 0x00);
  pmw3901_register_write(0x5B, 0xa0);
  pmw3901_register_write(0x4E, 0xA8);
  pmw3901_register_write(0x5A, 0x50);
  pmw3901_register_write(0x40, 0x80);
}

void setLed(bool ledOn)
{
  HAL_Delay(200);
  pmw3901_register_write(0x7f, 0x14);
  pmw3901_register_write(0x6f, ledOn ? 0x1c : 0x00);
  pmw3901_register_write(0x7f, 0x00);
}
