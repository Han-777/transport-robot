#ifndef __OPS_H
#define __OPS_H

#include <stdint.h>
#include "main.h"

typedef union
{
    uint8_t data[24];
    float ActVal[6];
} OPS;

typedef struct
{
    float OPS_x;
    float OPS_y;
    float OPS_heading;
    int OPS_ring; // clockwise negative
    uint8_t OPS_Init_Flag;
} OPS_data_t;

OPS_data_t *Ops_Init(UART_HandleTypeDef *ops_usart_handle);

#endif // DEBUG