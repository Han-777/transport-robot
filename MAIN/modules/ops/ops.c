#include "ops.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "memory.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
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
    // 转弯处理，顺时针
    if (ops_data->OPS_heading > 170 && ops_data_buff.ActVal[0] < -170)
    {
        ops_data->OPS_ring++;
    }
    else if (ops_data->OPS_heading < -170 && ops_data_buff.ActVal[0] > 170)
    {
        ops_data->OPS_ring--;
    }
    ops_data->OPS_heading = -ops_data_buff.ActVal[0];
    ops_data->OPS_x = ops_data_buff.ActVal[3];
    ops_data->OPS_y = ops_data_buff.ActVal[4];
    if (ops_data->OPS_x + ops_data->OPS_y + ops_data->OPS_heading != 0 && ops_data->OPS_Init_Flag == 0)
    {
        ops_data->OPS_Init_Flag = 1;
    }
}

void OPS_Calibrate(float x, float y, float heading)
{
    char update_x[4] = "ACTX";
    char update_y[4] = "ACTY";
    char update_j[4] = "ACTJ";

    static union
    {
        float value;
        char data[4];
    } new_value;

    new_value.value = x;
    USARTSendBytes(ops_instance, update_x, 4, USART_TRANSFER_IT);
    USARTSendBytes(ops_instance, new_value.data, 4, USART_TRANSFER_IT);

    osDelay(20);

    new_value.value = y;
    USARTSendBytes(ops_instance, update_y, 4, USART_TRANSFER_IT);
    USARTSendBytes(ops_instance, new_value.data, 4, USART_TRANSFER_IT);

    osDelay(20);

    new_value.value = heading;
    USARTSendBytes(ops_instance, update_j, 4, USART_TRANSFER_IT);
    USARTSendBytes(ops_instance, new_value.data, 4, USART_TRANSFER_IT);
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
    return ops_data;
}