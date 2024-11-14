#include "object.h"
#include "robot_def.h"
#include "step_motor.h"
#include "message_center.h"
#include "general_def.h"
#include "usart.h"
#include "servo_ctrl.h"
#include "cmsis_os.h"
#include "task.h"
#include "bsp_dwt.h"

static Publisher_t *object_pub;
static Subscriber_t *object_sub;

static Object_Ctrl_Cmd_s object_cmd_recv;
static Object_Upload_Data_s object_feedback_data; // 返回是否夹完
static StepMotorInstance *step_motor_rotate, *step_motor_front, *step_motor_rise;
static uint8_t task_cnt = 0;
static uint8_t sub_task_cnt = 0;
// static StepMotorInstance *step_motor2;

/**
 * @brief check if all step motor finish
 */
static uint8_t checkStepMotorFinish(void)
{
    uint8_t motor_finish = 1;
    motor_finish = (step_motor_rotate->motor_finish_flag) & (step_motor_front->motor_finish_flag) & (step_motor_rise->motor_finish_flag);
    if (motor_finish == 0)
        return 0;
    else
        return 1;
}

// 放置的时候从右向左

/*============================== getObjectFromPlate ================================*/
static uint8_t getObjectFromPlate1(void)
{
    setTargetPulse(step_motor_front, 1000, GPIO_PIN_RESET, MOTOR_DIRECTION_NORMAL); // 缩进 + 舵机下
    servoMove(calibObj);                                                            // 舵机下
    osDelay(300);
    if (checkStepMotorFinish())
    {
        sub_task_cnt++;
        return 1;
    }
    return 0;
}

static uint8_t getObjectFromPlate2(void) // 伸出去 + 夹
{
    setTargetPulse(step_motor_front, 1000, GPIO_PIN_SET, MOTOR_DIRECTION_NORMAL); // 伸出去
    if (checkStepMotorFinish())
    {
        servoMove(getObj); // 夹
        // maybe need to add a delay
        sub_task_cnt++;
        return 1;
    }
    return 0;
}

static uint8_t rotatePlane(uint8_t degree) // 提前记录好每一个角度对应的脉冲数
{
    switch (degree)
    {
    case -90:
        setTargetPulse(step_motor_rotate, 1000, GPIO_PIN_SET, MOTOR_DIRECTION_REVERSE);
        break;
    case 0:
        setTargetPulse(step_motor_rotate, 1000, GPIO_PIN_SET, MOTOR_DIRECTION_NORMAL);
        break;
    case 90:
        setTargetPulse(step_motor_rotate, 1000, GPIO_PIN_SET, MOTOR_DIRECTION_NORMAL);
        break;
    }
    if (checkStepMotorFinish())
    {
        sub_task_cnt++;
        return 1;
    }
    return 0;
}

/*============================ putObjectFromPlate =================================*/

static uint8_t putObjectFromPlate1(void) // 上 + 翻 + 放
{
    // servoMove(rotateRed +);                                                      // 翻 (同时转转盘)

    setTargetPulse(step_motor_rise, 1000, GPIO_PIN_SET, MOTOR_DIRECTION_NORMAL); // 上 (maybe in the same time)
    if (checkStepMotorFinish())
    {
        servoMove(putObj); // 放
        // maybe need to add a delay
        sub_task_cnt++;
        return 1;
    }
    return 0;
}

static uint8_t putObjectFromPlate2(void) // 翻 + 下
{
    servoMove(8);                                                                 // 翻
    setTargetPulse(step_motor_rise, 5000, GPIO_PIN_SET, MOTOR_DIRECTION_REVERSE); // 下
    if (checkStepMotorFinish())
    {
        sub_task_cnt = 0;
        return 1;
    }
    return 0;
}

static uint8_t (*getObjectFromPlate[])(void) = {
    getObjectFromPlate1, // 缩进 + 舵机下
    getObjectFromPlate2, // 伸出去 + 夹
    putObjectFromPlate1, // 上 + 翻 + 放
    putObjectFromPlate2, // 翻 + 下
};

/*================================= getObject =================================*/
static uint8_t getObjectFromCar(void)
{
}

static uint8_t putOjbectFromCar(void)
{
}
/*======================= Task Related ========================*/
void ObjectInit()
{
    Step_Motor_Init_Config_s step_motor1_config =
        {
            .step_motor_type = Emm42_V5,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .slave_htim = &htim2,
            .main_htim = &htim8,
            .direction_port = GPIOH,
            .direction_pin = GPIO_PIN_9,
            .enable_port = GPIOH,
            .enable_pin = GPIO_PIN_8,
            .channel = TIM_CHANNEL_1,
        };
    Step_Motor_Init_Config_s step_motor2_config =
        {
            .step_motor_type = Emm42_V5,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .slave_htim = &htim4,
            .main_htim = &htim8,
            .direction_port = GPIOH,
            .direction_pin = GPIO_PIN_12,
            .enable_port = GPIOH,
            .enable_pin = GPIO_PIN_11,
            .channel = TIM_CHANNEL_2,
        };
    Step_Motor_Init_Config_s step_motor3_config =
        {
            .step_motor_type = Emm42_V5,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .slave_htim = &htim5,
            .main_htim = &htim8,
            .direction_port = GPIOI,
            .direction_pin = GPIO_PIN_0,
            .enable_port = GPIOH,
            .enable_pin = GPIO_PIN_7,
            .channel = TIM_CHANNEL_3,
        };
    step_motor_rotate = StepMotorInit(&step_motor1_config);
    step_motor_front = StepMotorInit(&step_motor2_config);
    step_motor_rise = StepMotorInit(&step_motor3_config);
    // setTargetPulse(step_motor_rotate, 0, GPIO_PIN_RESET, MOTOR_DIRECTION_NORMAL); // 底盘电机控制不要放在任务里面，和底盘同时运动
    // setTargetPulse(step_motor_front, 0, GPIO_PIN_RESET, MOTOR_DIRECTION_NORMAL);  // 前后
    // setTargetPulse(step_motor_rise, 0, GPIO_PIN_RESET, MOTOR_DIRECTION_NORMAL);   // set 降 reset 升
    /*==============ServoCtrl_Init=================*/
    ServoCtrl_Init(&huart6);
    object_pub = PubRegister("object_feed", sizeof(Object_Upload_Data_s));
    object_sub = SubRegister("object_cmd", sizeof(Object_Ctrl_Cmd_s));
}

static uint8_t object_init_flag = 0;
void ObjectTask()
{
    SubGetMessage(object_sub, (void *)&object_cmd_recv);
    if (!object_init_flag)
    {
        osDelay(200);
        object_init_flag = 1;
    }
    /*------------ test code ---------------*/
    // setTargetPulse(step_motor_rotate, 5000, GPIO_PIN_RESET, MOTOR_DIRECTION_NORMAL); // 底盘电机控制不要放在任务里面，和底盘同时运动
    // setTargetPulse(step_motor_front, 5000, GPIO_PIN_RESET, MOTOR_DIRECTION_NORMAL);  // 前后
    // setTargetPulse(step_motor_rise, 5000, GPIO_PIN_RESET, MOTOR_DIRECTION_NORMAL);   // set 降 reset 升

    switch (object_cmd_recv.actionNum)
    {
    case none:
        break;
    case defau:
        servoMove(1);
        osDelay(300);
        servoMove(2);
        break;
    case putObjectFromPlate:
        if (getObjectFromPlate[sub_task_cnt]())
        {
            sub_task_cnt++;
            if (sub_task_cnt == 4)
            {
                object_cmd_recv.actionNum = none;
                sub_task_cnt = 0;
            }
        }
        break;
    case Scan:
        break;
    case getObect:
        break;
    }
    PubPushMessage(object_pub, (void *)&object_feedback_data);
}
