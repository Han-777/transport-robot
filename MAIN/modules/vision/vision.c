#include "vision.h"
#include "bsp_usart.h"
#include "memory.h"

#define VISION_FRAME_SIZE 9 // 2 + 24 + 2
#define VISION_FRAME_HEAD 0x2C
#define VISION_FRAME_TAIL 0x5B
static VisionData_t vision_data;
static USARTInstance *vision_instance;

static void VisionRxCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (Size != VISION_FRAME_SIZE)
    {
        return;
    }
    if (vision_instance->recv_buff[0] == VISION_FRAME_HEAD && vision_instance->recv_buff[VISION_FRAME_SIZE - 1] == VISION_FRAME_TAIL)
    {
        // memcpy(vision_data, vision_instance->recv_buff + 1, VISION_FRAME_SIZE - 2);
        vision_data.object_color = vision_instance->recv_buff[1];
        vision_data.x = (uint16_t)(vision_instance->recv_buff[3] << 8 | vision_instance->recv_buff[2]);
        vision_data.y = (uint16_t)(vision_instance->recv_buff[5] << 8 | vision_instance->recv_buff[4]);
        vision_data.heading = (int16_t)(vision_instance->recv_buff[7] << 8 | vision_instance->recv_buff[6]);
    }
}

void VisionSend(VisionSendData_e mode)
{
    // uint8_t send_buff = (uint8_t)mode;q
    USARTSend(vision_instance, &mode, 1, USART_TRANSFER_IT);
}

VisionData_t *Vision_Init(UART_HandleTypeDef *vision_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = VisionRxCallback;
    conf.recv_buff_size = VISION_FRAME_SIZE;
    conf.usart_handle = vision_usart_handle;
    vision_instance = USARTRegister(&conf);

    memset(&vision_data, 0, sizeof(vision_data));
    return &vision_data;
}
