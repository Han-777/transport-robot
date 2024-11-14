#ifndef __VISION_H
#define __VISION_H

#include <stdint.h>
#include "main.h"
#include "robot_def.h"
// typedef struct
// {
//     objColor_s object_color;
//     uint16_t x;
//     uint16_t y;
//     int16_t heading;
// } VisionData_t;
typedef struct
{
    objColor_s object_color;
    double x;
    double y;
    double heading;
} VisionData_t;

typedef enum
{
    colorMode = 0x01, // 圆盘：color x, y
    angleMode = 0x02, // 角度：heading
    coordMode = 0x03, // 坐标：x, y
} VisionSendData_e;

VisionData_t *Vision_Init(UART_HandleTypeDef *vision_usart_handle);
void VisionSend(VisionSendData_e mode);
#endif