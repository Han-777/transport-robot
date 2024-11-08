#include "object.h"
#include "robot_def.h"
#include "step_motor.h"
#include "message_center.h"
#include "general_def.h"
#include "usart.h"

static Publisher_t *object_pub;
static Subscriber_t *object_sub;

static Object_Ctrl_Cmd_s object_cmd_send;
static Object_Upload_Data_s object_feedback_data; // 返回是否夹完
static StepMotorInstance *step_motor1, *step_motor2, *step_motor3;
// static StepMotorInstance *step_motor2;

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
            .step_motor_type = Emm35_V5,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .slave_htim = &htim5,
            .main_htim = &htim8,
            .direction_port = GPIOI,
            .direction_pin = GPIO_PIN_0,
            .enable_port = GPIOH,
            .enable_pin = GPIO_PIN_7,
            .channel = TIM_CHANNEL_3,
        };
    step_motor1 = StepMotorInit(&step_motor1_config);
    step_motor2 = StepMotorInit(&step_motor2_config);
    step_motor3 = StepMotorInit(&step_motor3_config);
    object_pub = PubRegister("object_cmd", sizeof(Object_Ctrl_Cmd_s));
    object_sub = SubRegister("object_feed", sizeof(Object_Upload_Data_s));
}

void ObjectTask()
{
    SubGetMessage(object_sub, (void *)&object_feedback_data);
    // setTargetPulse(step_motor2, 0, GPIO_PIN_SET);

    PubPushMessage(object_pub, (void *)&object_cmd_send);
}