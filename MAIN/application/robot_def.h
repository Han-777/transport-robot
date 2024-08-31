#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

// #include "master_process.h"
#include "stdint.h"
#define ROBOT_DEF_PARAM_WARNING

#define ONE_BOARD // 单板控制整车
// #define CHASSIS_BOARD //底盘板
// #define GIMBAL_BOARD  //云台板
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 350              // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 300             // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 60             // 轮子半径
#define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
    CHASSIS_VISION_REFINE,     // 视觉微调模式
} chassis_mode_e;

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 区域ENUN
 */
typedef enum
{
    A,
    B,
    C,
    D,
    C2C, // cross to cross
    home
} regionEnum;

// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float x;       // x position
    float y;       // y position
    float heading; // target angle
    // float vx;           // 前进方向速度
    // float vy;           // 横移方向速度
    // float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    chassis_mode_e chassis_mode;
    int chassis_speed_buff;
    // UI部分
    //  ...

} Chassis_Ctrl_Cmd_s;

typedef struct
{
    uint8_t water_flag;
    uint8_t set_plantCnt_flag; // 1: 6, 2: 12, 3: 18, 4: 24
} Water_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
    uint8_t chassis_arrive;
    uint8_t chassis_vague_arrive;
} Chassis_Upload_Data_s;

typedef struct
{
    uint8_t water_finish_state; // water state (完成为1)
    uint8_t plant_cnt;          // number of plant
} Water_Upload_Data_s;

/*=================双板通信================*/
typedef enum
{
    SLIGHT = 0x01,
    GENERAL = 0x02, // Corrected typo from "GENERAl" to "GENERAL"
    SERIOUS = 0x03,
} Drought_Info_e;

typedef struct
{
    Drought_Info_e drought_info[18];
    uint8_t recv_feedback_flag; // 接收到正确数据返回1
} Comm_Send_Data_s;             // 板间通信结构体

typedef struct
{
    uint8_t plant_Cnt;
    uint8_t D_Drougnt_info;
    uint8_t recv_feedback_flag; // 接收到正确数据返回1
} Comm_Recv_Data_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H