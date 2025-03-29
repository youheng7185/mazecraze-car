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
#include "usbd_cdc_if.h"

#define BUFFER_LEN 128
//uint8_t tx_buffer[BUFFER_LEN];
//uint16_t usbTxLength;

static inline void my_printf(const char *format, ...)
{
    char tx_buffer[BUFFER_LEN];
    va_list args;
    va_start(args, format);
    int usbTxLength = vsnprintf(tx_buffer, BUFFER_LEN, format, args);
    va_end(args);

    if (usbTxLength > 0 && usbTxLength < BUFFER_LEN)
    {
        CDC_Transmit_FS((uint8_t *)tx_buffer, usbTxLength);
    }
}
#endif //MY_PRINT_H
