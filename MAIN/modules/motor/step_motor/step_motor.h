/**
 * @file step_motor.h
 * @author Han-777
 * @brief 步进电机控制头文件
 * @version 0.1
 * @date 2024-9-8
 *
 */

#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include <stdint.h>
#include "main.h"
#include "tim.h"
#include "bsp_can.h"
#include "motor_def.h"

#define STEP_MOTOR_CNT 3 // numbers of step motor

typedef enum
{
    motor_finish = 0,
    motor_running = 1
} Step_Motor_Finish_flag_e;
typedef enum
{
    Emm28_V5 = 0,
    Emm42_V5 = 1,
} Step_Motor_Type_e;

// typedef enum
// {
//     begin_pos = 0,
//     end_pos = 1,
// } Step_Motor_Position_e;

/**
 * @brief step motor configuration structure
 */
typedef struct
{
    Step_Motor_Type_e step_motor_type;
    // Step_Motor_Position_e step_motor_position;
    Motor_Reverse_Flag_e motor_reverse_flag; // controller 具体用了哪些环（微观）_
    TIM_HandleTypeDef *main_htim;            // pwm输出
    TIM_HandleTypeDef *slave_htim;           // 计数
    uint32_t channel;
} Step_Motor_Init_Config_s;

/**
 * @brief step motor instance structure
 */
typedef struct
{
    Step_Motor_Type_e Step_Motor_type; // 电机类型
    // Step_Motor_Position_e Step_Motor_position; // 电机位置
    Motor_Working_Type_e stop_flag;          // 启停标志
    Motor_Reverse_Flag_e motor_reverse_flag; // controller 具体用了哪些环（微观）_
    TIM_HandleTypeDef *main_htim;            // pwm输出
    uint32_t channel;
    TIM_HandleTypeDef *slave_htim; // 计数
    GPIO_TypeDef *direction_port;
    uint16_t direction_pin;
    uint8_t motor_finish_flag;
} StepMotorInstance;

void setTargetPulse(StepMotorInstance *step_motor_instance, uint16_t target_pulse, GPIO_PinState motor_direction);

void StepMotorControl();

#endif