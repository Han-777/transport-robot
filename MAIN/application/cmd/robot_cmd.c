#include "robot_def.h"
#include "robot_cmd.h"
// bsp
#include "bsp_dwt.h"
#include "usart.h"
#include <string.h>
#include "usart.h"

// module
#include "message_center.h"
#include "general_def.h"
#include "cmsis_os.h"
#include "lcd_test.h"
#include "lcd_rgb.h"
#include "ops.h"
#include "qr_code.h"
#include "vision.h"

/*------------------------water message----------------------------*/
// static Water_Ctrl_Cmd_s water_cmd_send;
// static Water_Upload_Data_s water_feedback_data;
/*==============data======================*/
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;      // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub;    // 底盘反馈信息订阅者
static Publisher_t *object_cmd_pub;       // 物体控制消息发布者
static Subscriber_t *object_feedback_sub; // 物体反馈信息订阅者
#endif

static Chassis_Ctrl_Cmd_s chassis_cmd_send;         // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_feedback_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等
static Object_Ctrl_Cmd_s object_cmd_send;           // 发送给物体应用的信息,包括控制信息和UI绘制相关
static Object_Upload_Data_s object_feedback_data;   // 从物体应用接收的反馈信息信息,物体夹取状态等

static objColor_s *qr_data;
static VisionData_t *vision_data;
static int (*operation_sequence[])(void);
#define max_run_itr 10
static uint8_t cmd_run_idx = 0;
static uint8_t default_rgb[6] = {0x31, 0x32, 0x33, 0x31, 0x32, 0x33};
/*--------------Coordinate Input------------------*/
static void SetCoordinate(float x, float y, float heading, uint16_t speed_limit)
{
    chassis_cmd_send.chassis_mode = CHASSIS_OPS_MOVE;
    chassis_cmd_send.x = x;
    chassis_cmd_send.y = y;
    chassis_cmd_send.heading = heading;
    chassis_cmd_send.chassis_speed_limit = speed_limit;
}

/*-------------------- Object Input ----------------------*/
static void SetAction(objectAction action, uint8_t color)
{
    object_cmd_send.actionNum = action;
    object_cmd_send.color = color;
}
/*--------------QR Code-------------*/
// static int robot_init(void)
// {
//     SetAction(defau, none);
//     // SetCoordinate(0, 0, 0, 10000);

//     // OPS_Calibrate(0, 0, 0);
//     return 1;
// }
// static int robot_init(void)
// {
//     SetAction(getObjFromPlate, red);

//     // osDelay(100000);
//     // OPS_Calibrate(0, 0, 0);
//     return 0;
// }
/**
 * @brief 机器人到达二维码位置
 */
static int qr1_1(void)
{
    SetAction(defau, none);
    SetCoordinate(-200, 500, 0, 10000);
    // SetCoordinate(0, 2000, 0, 20000);
    // chassis_feedback_data.chassis_vague_arrive = 1;
    if (chassis_feedback_data.chassis_vague_arrive)
    {
        VisionSend(coordMode);
        return 1;
    }
    return 0;
}

/**
 * @brief 机器人扫描成功并得到正确数据
 */
static int qr1_2(void)
{
    static uint8_t qr_pre_time = 0;
    qr_pre_time++;
    SetCoordinate(-200, 650, 0, 10000);
    if (qr_pre_time > 1000)
    {
        memcpy(qr_data, default_rgb, 6);
    }
    if (qr_data_verify())
    {
        QR_Close();
        // 屏幕显示
        LCD_Display_Number(qr_data);
        SetAction(plateScan, none);
        return 1;
    }
    return 0;
}

/*------------Get Object--------------*/
/**
 * @brief 机器人到达转盘位置
 */
static int obj1_1(void)
{
    SetCoordinate(-170, 1300, 90, 10000);
    // SetCoordinate(-0.20, 1.25, 0);
    if (chassis_feedback_data.chassis_vague_arrive)
        return 1;
    return 0;
}

/**
 * @brief 获取目标物体
 */
static int obj1_2(void)
{
    SetCoordinate(-200, 1500, 90, 10000);
    VisionSend(colorMode);

    if (vision_data->object_color == qr_data[0])
    {
        SetAction(getObjFromPlate + qr_data[0] - 0x30, qr_data[0] - 0x30);
        return 1;
    }
    return 0;
}

static int obj1_3(void)
{
    osDelay(5000);
    SetCoordinate(-210, 1450, 90, 10000);
    VisionSend(colorMode);

    if (vision_data->object_color == qr_data[1])
    {
        SetAction(getObjFromPlate + qr_data[1] - 0x30, qr_data[1] - 0x30);
        return 1;
    }
    return 0;
}
static int obj1_4(void)
{
    osDelay(5000);
    SetCoordinate(-200, 1400, 90, 10000);
    VisionSend(colorMode);

    if (vision_data->object_color == qr_data[2])
    {
        SetAction(getObjFromPlate + qr_data[2] - 0x30, qr_data[2] - 0x30);
    }
    return 1;
}

// 回中
static int obj1_5(void)
{
    SetCoordinate(-190, 700, 90, 10000);
    if (chassis_feedback_data.chassis_vague_arrive)
        return 1;
    return 0;
}

static int obj2_1(void)
{
    SetCoordinate(-1500, 700, 0, 12000);
    if (chassis_feedback_data.chassis_vague_arrive)
        return 1;
    return 0;
}

static int obj2_2(void)
{
    chassis_cmd_send.chassis_mode = CHASSIS_VISION_REFINE;
    //  这里需要将修正好的视觉坐标传入
    // chassis_cmd_send.x = vision_data->x;
    // chassis_cmd_send.y = vision_data->y;
    // SetCoordinate(-1500, 700, 0, 12000); // 视觉坐标
    return 0;
}
// static int obj1_6(void)
// {
//     return 0;
// }

static int (*operation_sequence[])(void) = {
    // robot_init,
    qr1_1,
    qr1_2,
    obj1_1,
    obj1_2,
    obj1_3,
    obj1_4,
    obj1_5,
    obj2_1,
};

void RobotCMDInit()
{
    qr_data = QR_Init(&huart2);
    vision_data = Vision_Init(&huart4);
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    object_cmd_pub = PubRegister("object_cmd", sizeof(Object_Ctrl_Cmd_s));
    object_feedback_sub = SubRegister("object_feed", sizeof(Object_Upload_Data_s));
    chassis_cmd_send.chassis_mode = CHASSIS_OPS_MOVE;
    chassis_cmd_send.chassis_speed_limit = 2000;

    // OPS_Calibrate(0, 0, 0);
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

void RobotCMDTask(void)
{
    SubGetMessage(chassis_feed_sub, (void *)&chassis_feedback_data);
    SubGetMessage(object_feedback_sub, (void *)&object_feedback_data);
    // SubGetMessage(object_feedback_sub, (void *)&object_feedback_data);

    if (operation_sequence[cmd_run_idx]())
        cmd_run_idx++;
    if (cmd_run_idx >= max_run_itr)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    }

    // object_cmd_send.actionNum = defau;
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(object_cmd_pub, (void *)&object_cmd_send);
    // PubPushMessage(object_cmd_pub, (void *)&object_cmd_send);
}
