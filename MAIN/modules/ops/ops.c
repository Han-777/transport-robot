#include "ops.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "memory.h"
/***
 * @note ops的y轴(front)位0度，逆时针方向为正方向
 */
#define OPS_FRAME_SIZE 28 // 2 + 24 + 2
#define OPS_FRAME_HEAD 0x0D0A
#define OPS_FRAME_TAIL 0x0A0D
static OPS_data_t *ops_data;
static USARTInstance *ops_instance;
static DaemonInstance *ops_daemon_instance;
static OPS ops_data_buff;

static void OPS_data_process(void)
{
    // 转弯处理，顺时针为正
    if (ops_data->OPS_heading > 170 && ops_data_buff.ActVal[0] < -170)
    {
        ops_data->OPS_ring++;
    }
    else if (ops_data->OPS_heading < -170 && ops_data_buff.ActVal[0] > 170)
    {
        ops_data->OPS_ring--;
    }
    ops_data->OPS_heading = ops_data_buff.ActVal[0];
    ops_data->OPS_x = ops_data_buff.ActVal[3];
    ops_data->OPS_y = ops_data_buff.ActVal[4];
    if (ops_data->OPS_heading + ops_data->OPS_y + ops_data->OPS_heading != 0 && ops_data->OPS_Init_Flag == 0)
    {
        ops_data->OPS_Init_Flag = 1;
    }
}
static void OPSRxCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    DaemonReload(ops_daemon_instance);
    if (ops_instance->recv_buff[0] == (uint8_t)(OPS_FRAME_HEAD >> 8) && ops_instance->recv_buff[1] == (uint8_t)(OPS_FRAME_HEAD & 0xFF) && ops_instance->recv_buff[OPS_FRAME_SIZE - 2] == (uint8_t)(OPS_FRAME_TAIL >> 8) && ops_instance->recv_buff[OPS_FRAME_SIZE - 1] == (uint8_t)(OPS_FRAME_TAIL & 0xFF))
    {
        memcpy(ops_data_buff.data, &ops_instance->recv_buff[2], 24);
        OPS_data_process();
    }
}
static void OpsLostCallback(void *id)
{
    USARTServiceInit(ops_instance);
    memset(ops_instance->recv_buff, 0, ops_instance->recv_buff_size);
    memset(ops_data, 0, sizeof(*ops_data));
    ops_data->OPS_ring = 0;
    ops_data->OPS_Init_Flag = 0;
}
OPS_data_t *Ops_Init(UART_HandleTypeDef *ops_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = OPSRxCallback;
    conf.recv_buff_size = OPS_FRAME_SIZE;
    conf.usart_handle = ops_usart_handle;
    ops_instance = USARTRegister(&conf);

    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 2, // 200fps = 5ms, 1: 10ms没收到消息
        .callback = OpsLostCallback,
        .owner_id = ops_usart_handle,
    };
    ops_daemon_instance = DaemonRegister(&daemon_conf);
    memset(ops_data, 0, sizeof(*ops_data));
    ops_data->OPS_ring = 0;
    ops_data->OPS_Init_Flag = 0;
    return ops_data;
}