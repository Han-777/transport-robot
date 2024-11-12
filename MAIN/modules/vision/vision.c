#include "vision.h"
#include "bsp_usart.h"
#include "memory.h"

#define VISION_FRAME_SIZE 28 // 2 + 24 + 2
#define VISION_FRAME_HEAD 0x0D0A
#define VISION_FRAME_TAIL 0x0A0D
static VisionData_t *vision_data;
static USARTInstance *vision_instance;

static void VisionRxCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (Size != VISION_FRAME_SIZE)
    {
        return;
    }
    if (vision_instance->recv_buff[0] == VISION_FRAME_HEAD && vision_instance->recv_buff[27] == VISION_FRAME_TAIL)
    {
        memcpy(vision_data, vision_instance->recv_buff + 2, 24);
    }
}

VisionData_t *Vision_Init(UART_HandleTypeDef *vision_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = VisionRxCallback;
    conf.recv_buff_size = VISION_FRAME_SIZE;
    conf.usart_handle = vision_usart_handle;
    vision_instance = USARTRegister(&conf);

    memset(vision_data, 0, sizeof(*vision_data));
    return vision_data;
}
