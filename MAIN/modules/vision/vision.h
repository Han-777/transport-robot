#ifndef __VISION_H
#define __VISION_H

#include <stdint.h>
#include "main.h"

typedef struct
{
    float x;
    float y;
    float heading;
    uint8_t ring; // clockwise negative
    uint8_t init_flag;
} VisionData_t;

VisionData_t *Vision_Init(UART_HandleTypeDef *vision_usart_handle);

#endif