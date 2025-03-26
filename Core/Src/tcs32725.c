/**************************************************************************/
/*!
    @file     Adafruit_TCS34725.cpp
    @author   KTOWN (Adafruit Industries)
    @license  BSD (see license.txt)

    Driver for the TCS34725 digital color sensors.

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "tcs32725.h"
#include "my_print.h"
#include "stm32f4xx_hal.h"

bool _tcs34725Initialised;
tcs34725Gain_t _tcs34725Gain;
tcs34725IntegrationTime_t _tcs34725IntegrationTime;

/*========================================================================*/
/*                          PRIVATE FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    @brief  Implements missing powf function
*/
/**************************************************************************/
float powf(const float x, const float y) {
    return (float)(pow((double)x, (double)y));
}

/**************************************************************************/
/*!
    @brief  Writes a register and an 8 bit value over I2C
*/
/**************************************************************************/
void tcs34725_write8(uint8_t reg, uint8_t value) {
	/*
    Wire.beginTransmission(TCS34725_ADDRESS);
    #if ARDUINO >= 100
    Wire.write(TCS34725_COMMAND_BIT | reg);
    Wire.write(value & 0xFF);
    #else
    Wire.send(TCS34725_COMMAND_BIT | reg);
    Wire.send(value & 0xFF);
    #endif
    Wire.endTransmission();
    */
    uint8_t buffer[2] = {TCS34725_COMMAND_BIT | reg, value};
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, buffer, 2, HAL_MAX_DELAY);
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
uint8_t read8(uint8_t reg) {
	/*
    Wire.beginTransmission(TCS34725_ADDRESS);
    #if ARDUINO >= 100
    Wire.write(TCS34725_COMMAND_BIT | reg);
    #else
    Wire.send(TCS34725_COMMAND_BIT | reg);
    #endif
    Wire.endTransmission();

    Wire.requestFrom(TCS34725_ADDRESS, 1);
    #if ARDUINO >= 100
    return Wire.read();
    #else
    return Wire.receive();
    #endif
    */
    uint8_t cmd = TCS34725_COMMAND_BIT | reg;
    uint8_t value = 0;
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, &cmd, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, TCS34725_ADDRESS, &value, 1, HAL_MAX_DELAY);
    return value;
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
uint16_t read16(uint8_t reg) {
	/*
    uint16_t x; uint16_t t;

    Wire.beginTransmission(TCS34725_ADDRESS);
    #if ARDUINO >= 100
    Wire.write(TCS34725_COMMAND_BIT | reg);
    #else
    Wire.send(TCS34725_COMMAND_BIT | reg);
    #endif
    Wire.endTransmission();

    Wire.requestFrom(TCS34725_ADDRESS, 2);
    #if ARDUINO >= 100
    t = Wire.read();
    x = Wire.read();
    #else
    t = Wire.receive();
    x = Wire.receive();
    #endif
    x <<= 8;
    x |= t;
    return x;
    */
	uint8_t cmd = TCS34725_COMMAND_BIT | reg;
	uint8_t buffer[2];
	HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, &cmd, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, TCS34725_ADDRESS, buffer, 2, HAL_MAX_DELAY);
	return (uint16_t)(buffer[1] << 8) | buffer[0];
}

/**************************************************************************/
/*!
    Enables the device
*/
/**************************************************************************/
void enable(void) {
    tcs34725_write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
    HAL_Delay(3);
    tcs34725_write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
}

/**************************************************************************/
/*!
    Disables the device (putting it in lower power sleep mode)
*/
/**************************************************************************/
void disable(void) {
    /* Turn the device off to save power */
    uint8_t reg = 0;
    reg = read8(TCS34725_ENABLE);
    tcs34725_write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}

/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
    Constructor
*/
/**************************************************************************/
/*
Adafruit_TCS34725::Adafruit_TCS34725(tcs34725IntegrationTime_t it, tcs34725Gain_t gain) {
    _tcs34725Initialised = false;
    _tcs34725IntegrationTime = it;
    _tcs34725Gain = gain;
}
*/
/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    Initializes I2C and configures the sensor (call this function before
    doing anything else)
*/
/**************************************************************************/


bool tcs32725_begin(TCS34725_t *sensor, tcs34725IntegrationTime_t it, tcs34725Gain_t gain) {
    //Wire.begin();

    /* Make sure we're actually connected */
    uint8_t x = read8(TCS34725_ID);
    my_printf("device id: %d", x);
    if (x != 0x4D) {
        return false;
    }
    _tcs34725Initialised = true;

    /* Set default integration time and gain */
    setIntegrationTime(it);
    setGain(gain);

    /* Note: by default, the device is in power down mode on bootup */
    enable();

    return true;
}

/**************************************************************************/
/*!
    Sets the integration time for the TC34725
*/
/**************************************************************************/
void setIntegrationTime(tcs34725IntegrationTime_t it) {
    if (!_tcs34725Initialised) {
        //begin();
    	my_printf("set integration time go wrong\r\n");
    }

    /* Update the timing register */
    tcs34725_write8(TCS34725_ATIME, it);

    /* Update value placeholder */
    _tcs34725IntegrationTime = it;
}

/**************************************************************************/
/*!
    Adjusts the gain on the TCS34725 (adjusts the sensitivity to light)
*/
/**************************************************************************/
void setGain(tcs34725Gain_t gain) {
	/*
    if (!_tcs34725Initialised) {
        begin();
    }
    */

    /* Update the timing register */
    tcs34725_write8(TCS34725_CONTROL, gain);

    /* Update value placeholders */
    _tcs34725Gain = gain;
}

/**************************************************************************/
/*!
    @brief  Reads the raw red, green, blue and clear channel values
*/
/**************************************************************************/
void getRawData(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c) {
	/*
    if (!_tcs34725Initialised) {
        begin();
    }
	*/
    *c = read16(TCS34725_CDATAL);
    *r = read16(TCS34725_RDATAL);
    *g = read16(TCS34725_GDATAL);
    *b = read16(TCS34725_BDATAL);

    /* Set a delay for the integration time */
    switch (_tcs34725IntegrationTime) {
        case TCS34725_INTEGRATIONTIME_2_4MS:
            HAL_Delay(3);
            break;
        case TCS34725_INTEGRATIONTIME_24MS:
        	HAL_Delay(24);
            break;
        case TCS34725_INTEGRATIONTIME_50MS:
        	HAL_Delay(50);
            break;
        case TCS34725_INTEGRATIONTIME_101MS:
        	HAL_Delay(101);
            break;
        case TCS34725_INTEGRATIONTIME_154MS:
        	HAL_Delay(154);
            break;
        case TCS34725_INTEGRATIONTIME_700MS:
        	HAL_Delay(700);
            break;
    }
}

void getRGB(float *r, float *g, float *b) {
  uint16_t red, green, blue, clear;
  getRawData(&red, &green, &blue, &clear);
  uint32_t sum = clear;

  // Avoid divide by zero errors ... if clear = 0 return black
  if (clear == 0) {
    *r = *g = *b = 0;
    return;
  }

  *r = (float)red / sum * 255.0;
  *g = (float)green / sum * 255.0;
  *b = (float)blue / sum * 255.0;
}

/**************************************************************************/
/*!
    @brief  Converts the raw R/G/B values to color temperature in degrees
            Kelvin
*/
/**************************************************************************/
uint16_t calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b) {
    float X, Y, Z;      /* RGB to XYZ correlation      */
    float xc, yc;       /* Chromaticity co-ordinates   */
    float n;            /* McCamy's formula            */
    float cct;

    /* 1. Map RGB values to their XYZ counterparts.    */
    /* Based on 6500K fluorescent, 3000K fluorescent   */
    /* and 60W incandescent values for a wide range.   */
    /* Note: Y = Illuminance or lux                    */
    X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
    Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
    Z = (-0.68202F * r) + (0.77073F * g) + (0.56332F * b);

    /* 2. Calculate the chromaticity co-ordinates      */
    xc = (X) / (X + Y + Z);
    yc = (Y) / (X + Y + Z);

    /* 3. Use McCamy's formula to determine the CCT    */
    n = (xc - 0.3320F) / (0.1858F - yc);

    /* Calculate the final CCT */
    cct = (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

    /* Return the results in degrees Kelvin */
    return (uint16_t)cct;
}

/**************************************************************************/
/*!
    @brief  Converts the raw R/G/B values to lux
*/
/**************************************************************************/
uint16_t calculateLux(uint16_t r, uint16_t g, uint16_t b) {
    float illuminance;

    /* This only uses RGB ... how can we integrate clear or calculate lux */
    /* based exclusively on clear since this might be more reliable?      */
    illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

    return (uint16_t)illuminance;
}


void setInterrupt(bool i) {
    uint8_t r = read8(TCS34725_ENABLE);
    if (i) {
        r |= TCS34725_ENABLE_AIEN;
    } else {
        r &= ~TCS34725_ENABLE_AIEN;
    }
    tcs34725_write8(TCS34725_ENABLE, r);
}

/*
void clearInterrupt(void) {
    Wire.beginTransmission(TCS34725_ADDRESS);
    #if ARDUINO >= 100
    Wire.write(TCS34725_COMMAND_BIT | 0x66);
    #else
    Wire.send(TCS34725_COMMAND_BIT | 0x66);
    #endif
    Wire.endTransmission();
}
*/

void clearInterrupt(void) {
    uint8_t command = TCS34725_COMMAND_BIT | 0x66;
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, &command, 1, HAL_MAX_DELAY);
}


void setIntLimits(uint16_t low, uint16_t high) {
    tcs34725_write8(0x04, low & 0xFF);
    tcs34725_write8(0x05, low >> 8);
    tcs34725_write8(0x06, high & 0xFF);
    tcs34725_write8(0x07, high >> 8);
}
