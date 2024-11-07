#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "message_center.h"
#include "bluetooth.h"
#include "general_def.h"
#include "cmsis_os.h"
#include "mp3.h"
#include "lcd_test.h"
#include "lcd_rgb.h"
// bsp
#include "bsp_dwt.h"
#include "usart.h"
#include <string.h>
#include "usart.h"
#include "qr_code.h"
/*------------------------water message----------------------------*/
// static Water_Ctrl_Cmd_s water_cmd_send;
// static Water_Upload_Data_s water_feedback_data;
/*==============data======================*/
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
static Publisher_t *arm_cmd_pub;       // 浇水控制消息发布者
static Subscriber_t *arm_feed_sub;     // 浇水反馈信息订阅者
#endif

static Chassis_Ctrl_Cmd_s chassis_cmd_send;         // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_feedback_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等
// static Vision_Recv_s *vision_recv_data;             // 视觉接收数据指针,初始化时返回
// static Vision_Send_s vision_send_data;              // 视觉发送数据

static QR_data_t *qr_data;
static int (*operation_sequence[])(void);
#define max_run_itr 2
static uint8_t cmd_run_idx = 0;

/*--------------Coordinate Input------------------*/
static void SetCoordinate(float x, float y, float heading, uint16_t speed_limit)
{
    chassis_cmd_send.x = x;
    chassis_cmd_send.y = y;
    chassis_cmd_send.heading = heading;
    chassis_cmd_send.chassis_speed_limit = speed_limit;
}
/*--------------QR Code-------------*/
/**
 * @brief 机器人到达二维码位置
 */
static int qr1_1(void)
{
    chassis_cmd_send.chassis_mode = CHASSIS_OPS_MOVE;
    SetCoordinate(-0.18, 0.6, 0, 2000);
    if (chassis_feedback_data.chassis_vague_arrive)
        return 1;
    return 0;
}

/**
 * @brief 机器人扫描成功并得到正确数据
 */
static int qr1_2(void)
{
    // qr code handle
    // 屏幕显示
    return 1;
}

/*------------Get Object--------------*/
/**
 * @brief 机器人到达转盘位置
 */
static int obj1_1(void)
{
    // SetCoordinate(-0.20, 1.25, 0);
    SetCoordinate(0.1, 0.1, 0, 5000);
    if (chassis_feedback_data.chassis_vague_arrive)
        return 1;
    return 0;
}

/**
 * @brief 获取目标物体
 */
static int obj1_2(void)
{

    return 1;
}
static int (*operation_sequence[])(void) = {
    qr1_1,
    qr1_2,
    obj1_1,
};

void RobotCMDInit()
{
    qr_data = QR_Init(&huart2);
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    /*------------------------Can communication Init---------------------
    send:  CANCommSend(CANComm_ins, comm_send_data);
    recv:  (Comm_Recv_Data_s *) comm_recv_data = CANCommGet(&CANComm_ins);
    */
    // CANComm_Init_Config_s can_comm_config =
    //     {
    //         .can_config =
    //             {
    //                 .can_handle = &hfdcan1,
    //                 .tx_id = 0x02,
    //                 .rx_id = 0x01},
    //         .send_data_len = sizeof(Comm_Send_Data_s),
    //         .recv_data_len = sizeof(Comm_Recv_Data_s)};
    // CANComm_ins = CANCommInit(&can_comm_config);
    /*-----------------------message center-------------------------*/
}

// static int drought_info_ready(void)
// {
//     // static uint8_t checkCnt = 0;
//     // if (bluetooth_data->drought_buff[17] == 0)
//     // {
//     //     osDelay(2000);
//     //     checkCnt++;
//     //     if (checkCnt > 10) // 10s没看到就默认值
//     //     {
//     //         memcpy(drought_info, test, sizeof(drought_info));
//     //         HAL_UART_Abort_IT(&huart6);
//     //         return 1;
//     //     }
//     // }
//     // else
//     // {
//     //     LCD_Display_All(bluetooth_data->drought_buff);
//     //     memcpy(drought_info, bluetooth_data->drought_buff, sizeof(bluetooth_data->drought_buff));
//     //     HAL_UART_Abort_IT(&huart6);
//     //     return 1;
//     // }
//     // return 0;
//     if (bluetooth_data->drought_buff[17] != 0)
//     {
//         HAL_UART_Abort_IT(&huart6);
//         LCD_Display_All(bluetooth_data->drought_buff);
//         memcpy(comm_send_data->drought_info, bluetooth_data->drought_buff, sizeof(bluetooth_data->drought_buff));
//         return 1;
//     }

//     if (comm_recv_data->recv_feedback_flag == 1)
//     {
//         if (bluetooth_data->drought_buff[17] == 0)
//         {
//             HAL_UART_Abort_IT(&huart6);
//             memcpy(drought_info, test, sizeof(drought_info));
//         }
//         comm_send_data->recv_feedback_flag = 1;
//         return 1;
//     }
//     return 0;
// }

// static int Display_task(void)
// {
//     static uint8_t display_Cnt = 0;

//     if (comm_recv_data->plant_Cnt > display_Cnt)
//     {
//         if (comm_recv_data->plant_Cnt == 1)
//         {
//             LCD_Clear();
//         }
//         // comm_send_data->recv_feedback_flag = 1;
//         if (comm_recv_data <= 18)
//         {
//             MP3_broadcast(drought_info[display_Cnt]);
//             LCD_Display_One(drought_info[display_Cnt], display_Cnt);
//             display_Cnt = comm_recv_data->plant_Cnt;
//         }
//         else
//         {
//             MP3_broadcast(comm_recv_data->D_Drougnt_info);
//             LCD_Display_One(comm_recv_data->D_Drougnt_info, display_Cnt);
//             display_Cnt++;
//         }
//         memset(comm_recv_data, 0, sizeof(Comm_Recv_Data_s));
//         return 1;
//     }
//     return 0;
// }

void RobotCMDTask(void)
{
    SubGetMessage(chassis_feed_sub, (void *)&chassis_feedback_data);

    if (operation_sequence[cmd_run_idx]())
        cmd_run_idx++;
    if (cmd_run_idx >= max_run_itr)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        return;
    }

    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
}
