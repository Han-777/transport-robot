#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include "ops.h"
#include "usart.h"
/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm

#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
static PIDInstance x_pid_instance, y_pid_instance, heading_pid_instance;
/*-------------------------data--------------------------*/
static OPS_data_t *ops_data;
/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy, chassis_w, ff_vx, ff_vy; // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb;                      // 底盘速度解算后的临时输出,待进行限幅

void ChassisInit()
{
    /*----------------data-------------------*/
    ops_data = Ops_Init(&huart1);

    /*---------------x, y and heading loop--------------*/
    PID_Init_Config_s xy_pid_init_config =
        {
            .Kp = 50,
            .Ki = 12,
            .Kd = 2,
            .IntegralLimit = 15000,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .MaxOut = 15000,
        };
    PID_Init_Config_s heading_pid_init_config =
        {
            .Kp = 50,
            .Ki = 5,
            .Kd = 0,
            .IntegralLimit = 500,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .MaxOut = 2500,
        };
    PIDInit(&x_pid_instance, &xy_pid_init_config);
    PIDInit(&y_pid_instance, &xy_pid_init_config);
    PIDInit(&heading_pid_instance, &heading_pid_init_config);
    /*---------------speed and current loops--------------*/
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        // the initialization from module to bsp_
        .can_init_config.can_handle = &hfdcan1, // can通用设置，更多配置在下面按具体电机分配（方向和电机号）
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 2,  // 4.5
                .Ki = 10, // 0
                .Kd = 0,  // 0
                .IntegralLimit = 20000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, // 梯形积分（accuracy），微分先行 （防止突变）
                .MaxOut = 20000,
            },
            .current_PID = {
                .Kp = 2,  // 0.4
                .Ki = 10, // 0
                .Kd = 0,
                .IntegralLimit = 20000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 20000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, // 从电调获得反馈
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 速度环套电流环
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M2006,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = DJIMotorInit(&chassis_motor_config);

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}

/**
 * @brief calculate position pid loop -> target speed
 */
static void speedCalculate(void)
{
    PIDCalculate(&x_pid_instance, ops_data->OPS_x, chassis_cmd_recv.x);
    PIDCalculate(&y_pid_instance, ops_data->OPS_y, chassis_cmd_recv.y);
    PIDCalculate(&heading_pid_instance, ops_data->OPS_heading + ops_data->OPS_ring * 360, chassis_cmd_recv.heading);

    // calculate feedforward
    // ff_vx = (x_pid_instance.Err > 0.3 ? (0.1 * chassis_cmd_recv.x) : 0);
    // ff_vy = (y_pid_instance.Err > 0.3 ? (0.1 * chassis_cmd_recv.y) : 0);
    // ff_w = 0.01 * chassis_cmd_recv.heading;

    chassis_vx = x_pid_instance.Output; // 可能需要乘系数
    chassis_vy = y_pid_instance.Output; //
    chassis_w = heading_pid_instance.Output;
}

/**
 * @brief coordinate transform, 云台坐标系到底盘坐标系的变换
 */
static void CoordinateTransform(void)
{
    // 根据角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向; y正北，x正东
    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32(ops_data->OPS_heading * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(ops_data->OPS_heading * DEGREE_2_RAD);
    chassis_vx = chassis_vx * cos_theta - chassis_vy * sin_theta;
    chassis_vy = chassis_vx * sin_theta + chassis_vy * cos_theta;
}

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void OmniCalculate()
{
    // vt_lf = chassis_vx + chassis_vy + chassis_cmd_recv.wz * LF_CENTER;
    // vt_rf = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * RF_CENTER;
    // vt_lb = -chassis_vx + chassis_vy + chassis_cmd_recv.wz * LB_CENTER;
    // vt_rb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * RB_CENTER;
    // PIDCalculate(&w_pid_instance, chassis_heading, ops_data->OPS_heading + ops_data->OPS_ring * 360);
    vt_lf = chassis_vx + chassis_vy - chassis_w * LF_CENTER;
    vt_rf = -chassis_vx + chassis_vy + chassis_w * RF_CENTER;
    vt_lb = -chassis_vx + chassis_vy - chassis_w * LB_CENTER;
    vt_rb = chassis_vx + chassis_vy + chassis_w * RB_CENTER;
}

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
    // 功率限制待添加}
    // referee_data->PowerHeatData.chassis_power;
    // referee_data->PowerHeatData.chassis_power_buffer;
    chassis_cmd_recv.chassis_speed_limit = 10000;
    if (fabs(vt_lf) > chassis_cmd_recv.chassis_speed_limit)
    {
        vt_lf = (vt_lf > 0 ? chassis_cmd_recv.chassis_speed_limit : -chassis_cmd_recv.chassis_speed_limit);
    }
    if (fabs(vt_rf) > chassis_cmd_recv.chassis_speed_limit)
    {
        vt_rf = (vt_rf > 0 ? chassis_cmd_recv.chassis_speed_limit : -chassis_cmd_recv.chassis_speed_limit);
    }
    if (fabs(vt_lb) > chassis_cmd_recv.chassis_speed_limit)
    {
        vt_lb = (vt_lb > 0 ? chassis_cmd_recv.chassis_speed_limit : -chassis_cmd_recv.chassis_speed_limit);
    }
    if (fabs(vt_rb) > chassis_cmd_recv.chassis_speed_limit)
    {
        vt_rb = (vt_rb > 0 ? chassis_cmd_recv.chassis_speed_limit : -chassis_cmd_recv.chassis_speed_limit);
    }
    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}

/**
 * @brief 判断运动目标位置是否在范围内
 */
static void CheckStable(void)
{
    chassis_feedback_data.chassis_arrive = 0;
    chassis_feedback_data.chassis_vague_arrive = 0;
    if (fabs(x_pid_instance.Err) < 0.2 && fabs(y_pid_instance.Err) < 0.05 && fabs(heading_pid_instance.Err) < 0.05)
        chassis_feedback_data.chassis_arrive = 1;
    // else
    //     chassis_feedback_data.chassis_arrive = 0;

    if (fabs(x_pid_instance.Err) < 2 && fabs(y_pid_instance.Err) < 0.2 && fabs(heading_pid_instance.Err) < 0.2)
        chassis_feedback_data.chassis_vague_arrive = 1;
    // else
    // chassis_feedback_data.chassis_vague_arrive = 0;
}
/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD
    // chassis_cmd_recv.chassis_mode = CHASSIS_OPS_MOVE;
    // if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE || !ops_data->OPS_Init_Flag)
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    }
    else
    { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_OPS_MOVE:
        break;
    // case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
    // chassis_cmd_recv.wz = 0;
    // break;
    case CHASSIS_VISION_REFINE:
        break;
    default:
        break;
    }
    speedCalculate();
    // chassis_vx = 0;
    // chassis_vy = 15000;
    // chassis_w = 90;
    // PIDCalculate(&heading_pid_instance, ops_datsa->OPS_heading + ops_data->OPS_ring * 360, chassis_cmd_recv.heading);
    // chassis_w = heading_pid_instance.Output;
    CoordinateTransform();
    // 转换坐标系后的位置环pid计算
    // 根据控制模式进行正运动学解算,计算底盘输出
    OmniCalculate();

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();
    CheckStable();

    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}