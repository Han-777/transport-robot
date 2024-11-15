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

// #define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
// #define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

#define WHEEL_BASE 350              // 纵向轴距(前进后退方向)
#define TRACK_WIDTH                 // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 113  // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 113  // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 41             // 轮子半径
#define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0, // 电流零输入, 停车
    CHASSIS_OPS_MOVE,       // OPS模式
    CHASSIS_VISION_REFINE,  // 视觉微调模式
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
    uint16_t chassis_speed_limit;
    // UI部分
    //  ...

} Chassis_Ctrl_Cmd_s;

typedef enum
{
    none,
    defau,
    putObjectFromPlate,
    getObect,
    putObject,
    Scan,
    calib
} objectAction;
typedef struct
{
    objectAction actionNum;
} Object_Ctrl_Cmd_s;
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

/**
 * @brief object状态
 */
typedef struct
{
    uint8_t obj_finish_state; // object state (完成为1)
    uint8_t obj_color;        // object color
} Object_Upload_Data_s;
/*==============舵控板==============*/
typedef enum
{
    defaut,   // 2s
    calibObj, // 3s

    putObj,      // 2 300ms
    getObj,      // 3 300ms
    rotateRed,   // 4 300ms
    rotateGreen, // 5
    rotateBlue,  // 6

    moveBack,  // 7
    moveFront, // 8

    getObjRed,
    getObjGreen,
    getObjBlue,

    putRoughRed,
    putRoughGreen,
    putRoughBlue,
    getRoughRed,
    getRoughGreen,
    getRoughBlue,

    putDepHighRed,
    putDepHighGreen,
    putDepHighBlue,

    idle
} servoAction;

typedef enum
{
    red = 0x31,
    green = 0x32,
    blue = 0x33
} objColor_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H