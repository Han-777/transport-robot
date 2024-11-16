
// #include "qr_code.h"
// #include "bsp_usart.h"
// #include "memory.h"

// #define OR_FRAME_SIZE 8 // 3 + 3 color + 1 tail
// #define QR_FRAME_TAIL 0x0D
// static QR_data_t qr_data_buff;
// static QR_data_t *qr_data;
// static USARTInstance *qr_instance;

// static void QRRxCallback(UART_HandleTypeDef *huart, uint16_t Size)
// {
//     if (qr_instance->recv_buff[OR_FRAME_SIZE - 1] == QR_FRAME_TAIL)
//     {
//         qr_data->color1 = qr_instance->recv_buff[0];
//         qr_data->color2 = qr_instance->recv_buff[1];
//         qr_data->color3 = qr_instance->recv_buff[2];
//         qr_data->color4 = qr_instance->recv_buff[4];
//         qr_data->color5 = qr_instance->recv_buff[5];
//         qr_data->color6 = qr_instance->recv_buff[6];
//     }
//     else
//     {
//         memset(qr_data, 0, sizeof(QR_data_t));
//     }
// }

// int qr_data_verify(void)
// {
//     uint8_t qr_total = 0;
//     qr_total = qr_data->color1 + qr_data->color2 + qr_data->color3 + qr_data->color4 + qr_data->color5 + qr_data->color6;
//     if (qr_total == 0x96)
//         return 1;
//     return 0;
// }

// QR_data_t *QR_Init(UART_HandleTypeDef *qr_usart_handle)
// {
//     USART_Init_Config_s conf;
//     conf.module_callback = QRRxCallback;
//     conf.recv_buff_size = OR_FRAME_SIZE;
//     conf.usart_handle = qr_usart_handle;
//     qr_instance = USARTRegister(&conf);

//     memset(qr_data, 0, sizeof(QR_data_t));
//     return qr_data;
// }

#include "qr_code.h"
#include "bsp_usart.h"
#include "memory.h"

#define OR_FRAME_SIZE 8 // 3 + 3 color + 1 tail
#define QR_FRAME_TAIL 0x0D
static objColor_s qr_data[6];
static USARTInstance *qr_instance;

void QR_Close(void)
{
    HAL_UART_AbortReceive_IT(qr_instance->usart_handle);

    memset(qr_instance->recv_buff, 0, qr_instance->recv_buff_size);
}

void QR_Open(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(qr_instance->usart_handle, qr_instance->recv_buff, qr_instance->recv_buff_size);

    __HAL_DMA_DISABLE_IT(qr_instance->usart_handle->hdmarx, DMA_IT_HT);
}

static void QRRxCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (qr_instance->recv_buff[OR_FRAME_SIZE - 1] == QR_FRAME_TAIL)
    {
        memcpy(qr_data, qr_instance->recv_buff, 3);
        memcpy(qr_data + 3, qr_instance->recv_buff + 4, 3);
        // qr_data.color1 = qr_instance->recv_buff[0];
        // qr_data.color2 = qr_instance->recv_buff[1];
        // qr_data.color3 = qr_instance->recv_buff[2];
        // qr_data.color4 = qr_instance->recv_buff[4];
        // qr_data.color5 = qr_instance->recv_buff[5];
        // qr_data.color6 = qr_instance->recv_buff[6];
    }
    else
    {
        memset(qr_data, 0, sizeof(*qr_data));
    }
}

int qr_data_verify(void)
{
    static uint8_t qr_total_1 = 0, qr_total_2 = 0;
    // qr_total_1 = qr_data.color1 + qr_data.color2 + qr_data.color3;
    // qr_total_2 = qr_data.color4 + qr_data.color5 + qr_data.color6;
    qr_total_1 = qr_data[0] + qr_data[1] + qr_data[2];
    qr_total_2 = qr_data[3] + qr_data[4] + qr_data[5];
    if (qr_total_1 == 0x96 && qr_total_2 == 0x96)
        return 1;
    return 0;
}

objColor_s *QR_Init(UART_HandleTypeDef *qr_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = QRRxCallback;
    conf.recv_buff_size = OR_FRAME_SIZE;
    conf.usart_handle = qr_usart_handle;
    qr_instance = USARTRegister(&conf);

    memset(qr_data, 0, sizeof(*qr_data));
    return qr_data;
}
