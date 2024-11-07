#include "qr_code.h"
#include "bsp_usart.h"
#include "memory.h"

#define OR_FRAME_SIZE 7 // 3 + 3 color + 1 tail
#define QR_FRAME_TAIL 0x0D
static QR_data_t *qr_data;
static USARTInstance *qr_instance;

static void QRRxCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (qr_instance->recv_buff[OR_FRAME_SIZE - 1] == QR_FRAME_TAIL)
    {
        memcmp(qr_data, qr_instance->recv_buff, OR_FRAME_SIZE - 1);
    }
}

QR_data_t *QR_Init(UART_HandleTypeDef *qr_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = QRRxCallback;
    conf.recv_buff_size = OR_FRAME_SIZE;
    conf.usart_handle = qr_usart_handle;
    qr_instance = USARTRegister(&conf);

    memset(qr_data, 0, sizeof(*qr_data));
    return qr_data;
}
