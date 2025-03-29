//
// Created by lapchong on 3/1/25.
//

#ifndef MY_PRINT_H
#define MY_PRINT_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*
static inline void my_printf(const char *format, ...)
{
    char buffer[256];  // Adjust size as needed
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}
*/

#endif //MY_PRINT_H
