#ifndef __QR_CODE_H
#define __QR_CODE_H

#include <stdint.h>
#include "main.h"
#include "robot_def.h"
// typedef struct
// {
//     objColor_s color1;
//     objColor_s color2;
//     objColor_s color3;
//     objColor_s color4;
//     objColor_s color5;
//     objColor_s color6;
// } QR_data_t;

int qr_data_verify(void);

objColor_s *QR_Init(UART_HandleTypeDef *qr_usart_handle);
void QR_Close(void);
void QR_Open(void);

#endif