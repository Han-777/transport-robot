#include "step_motor.h"
#include <stdlib.h>
#include <string.h>

static StepMotorInstance *step_motor_instance[STEP_MOTOR_CNT] = {NULL};
static uint16_t target_pulse_value[STEP_MOTOR_CNT] = {0};
static GPIO_PinState motor_direction_arr[STEP_MOTOR_CNT] = {DISABLE, DISABLE, DISABLE};
static uint8_t step_motor_idx = 0; // register index

/**
 * @brief step motor register function
 *
 */
StepMotorInstance *StepMotorInit(Step_Motor_Init_Config_s *Step_Init_Config)
{
    StepMotorInstance *instance = (StepMotorInstance *)malloc(sizeof(StepMotorInstance));
    memset(instance, 0, sizeof(StepMotorInstance));

    instance->Step_Motor_type = Step_Init_Config->step_motor_type;
    // instance->htim = Step_Init_Config->htim;
    instance->main_htim = Step_Init_Config->main_htim;
    instance->channel = Step_Init_Config->channel;
    instance->slave_htim = Step_Init_Config->slave_htim;
    instance->slave_htim->Instance->CNT = 0;
    instance->motor_reverse_flag = Step_Init_Config->motor_reverse_flag;
    instance->stop_flag = MOTOR_STOP;
    instance->direction_port = Step_Init_Config->direction_port;
    instance->direction_pin = Step_Init_Config->direction_pin;
    instance->enable_port = Step_Init_Config->enable_port;
    instance->enable_pin = Step_Init_Config->enable_pin;
    step_motor_instance[step_motor_idx++] = instance;
    HAL_TIM_PWM_Stop(instance->main_htim, instance->channel);
    HAL_TIM_Base_Stop(instance->slave_htim);
    instance->motor_finish_flag = motor_finish;
    return instance;
}

void setTargetPulse(StepMotorInstance *instance, uint16_t target_pulse, GPIO_PinState motor_direction, Motor_Reverse_Flag_e motor_direction_flag)
{
    for (size_t i = 0; i < step_motor_idx; ++i)
    {
        if (step_motor_instance[i] == instance)
        {
            // instance->slave_htim->Instance->CNT = 0;
            step_motor_instance[i]->slave_htim->Instance->CNT = 0;
            target_pulse_value[i] = target_pulse;
            motor_direction_arr[i] = motor_direction;
            instance->motor_reverse_flag = motor_direction_flag;
        }
    }
}
/**
 * @brief step motor control function
 *        if motor finish flag is not finish, the next step should not be implemented
 *
 */
void StepMotorControl()
{
    StepMotorInstance *Step_Motor;

    for (size_t i = 0; i < step_motor_idx; ++i)
    {
        if (step_motor_instance[i])
        {
            Step_Motor = step_motor_instance[i];
            switch (Step_Motor->Step_Motor_type)
            {
            case Emm28_V5:
            case Emm35_V5:
            case Emm42_V5:
                HAL_GPIO_WritePin(Step_Motor->enable_port, Step_Motor->enable_pin, GPIO_PIN_SET);
                if (Step_Motor->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
                {
                    HAL_GPIO_WritePin(Step_Motor->direction_port, Step_Motor->direction_pin, !motor_direction_arr[i]);
                }
                else
                {
                    HAL_GPIO_WritePin(Step_Motor->direction_port, Step_Motor->direction_pin, motor_direction_arr[i]);
                }
                if (Step_Motor->slave_htim->Instance->CNT < target_pulse_value[i])
                {
                    HAL_TIM_Base_Start(Step_Motor->slave_htim);
                    HAL_TIM_PWM_Start(Step_Motor->main_htim, Step_Motor->channel);
                    Step_Motor->motor_finish_flag = motor_running;
                }
                else
                {
                    HAL_TIM_PWM_Stop(Step_Motor->main_htim, Step_Motor->channel);
                    HAL_TIM_Base_Stop(Step_Motor->slave_htim);
                    // Step_Motor->slave_htim->Instance->CNT = 0;
                    // target_pulse_value[i] = 0;
                    Step_Motor->motor_finish_flag = motor_finish;
                }
                break;
            }
        }
    }
}
