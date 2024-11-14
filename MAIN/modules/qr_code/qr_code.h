#ifndef __QR_CODE_H
#define __QR_CODE_H

#include <stdint.h>
#include "main.h"
typedef struct
{
    uint8_t color1;
    uint8_t color2;
    uint8_t color3;
    uint8_t color4;
    uint8_t color5;
    uint8_t color6;
} QR_data_t;

int qr_data_verify(void);

QR_data_t *QR_Init(UART_HandleTypeDef *qr_usart_handle);

#endif