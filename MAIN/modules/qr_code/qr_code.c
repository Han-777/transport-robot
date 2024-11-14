
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
static QR_data_t qr_data_buff;
static QR_data_t qr_data;
static USARTInstance *qr_instance;

static void QRRxCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (qr_instance->recv_buff[OR_FRAME_SIZE - 1] == QR_FRAME_TAIL)
    {
        qr_data.color1 = qr_instance->recv_buff[0];
        qr_data.color2 = qr_instance->recv_buff[1];
        qr_data.color3 = qr_instance->recv_buff[2];
        qr_data.color4 = qr_instance->recv_buff[4];
        qr_data.color5 = qr_instance->recv_buff[5];
        qr_data.color6 = qr_instance->recv_buff[6];
    }
    else
    {
        memset(&qr_data, 0, sizeof(QR_data_t));
    }
}

int qr_data_verify(void)
{
    uint8_t qr_total = 0;
    qr_total = qr_data.color1 + qr_data.color2 + qr_data.color3 + qr_data.color4 + qr_data.color5 + qr_data.color6;
    if (qr_total == 0x96)
        return 1;
    return 0;
}

QR_data_t *QR_Init(UART_HandleTypeDef *qr_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = QRRxCallback;
    conf.recv_buff_size = OR_FRAME_SIZE;
    conf.usart_handle = qr_usart_handle;
    qr_instance = USARTRegister(&conf);

    memset(&qr_data, 0, sizeof(QR_data_t));
    return &qr_data;
}
