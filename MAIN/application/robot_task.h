/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "usart.h"
#include "cmsis_os.h"

#include "robot.h"
#include "daemon.h"
// #include "buzzer.h"
#include <memory.h>
#include "robot_queue.h"
#include "robot_def.h"
// osThreadId insTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId daemonTaskHandle;
osThreadId objectTaskHandle;
// osThreadId uiTaskHandle;
/*      中断处理任务       */
#include "ops.h"
osThreadId opsTaskHandle;
// osThreadId isrTaskHandle;

// void StartINSTASK(void const *argument);
// void RobotInitTASK(void const *argument); // 用于陀螺仪以及蓝牙数据处理,此任务完成之后可以激活其他任务
void StartMOTORTASK(void const *argument);
void StartDAEMONTASK(void const *argument);
void StartROBOTTASK(void const *argument);
void StartOBJECTTASK(void const *argument);
// void StartGyroTask(void const *argument);
// void StartISRTASK(void const *argument);
void StartOPSTask(void const *argument);

/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
void OSTaskInit()
{
    // 创建队列
    opsQueue = xQueueCreate(10, sizeof(uint8_t) * (OPS_FRAME_SIZE + 1));

    osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 512);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
    daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);

    osThreadDef(robottask, StartROBOTTASK, osPriorityAboveNormal, 0, 2048);
    robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

    osThreadDef(objecttask, StartOBJECTTASK, osPriorityNormal, 0, 1024);
    robotTaskHandle = osThreadCreate(osThread(objecttask), NULL);

    // osThreadDef(opstask, StartOPSTask, osPriorityNormal, 0, 128);
    // opsTaskHandle = osThreadCreate(osThread(opstask), NULL);
    // osThreadDef(watertask, StartWaterTASK, osPriorityNormal, 0, 1024);
    // waterTaskHandle = osThreadCreate(osThread(watertask), NULL);
}

__attribute__((noreturn)) void StartMOTORTASK(void const *argument)
{
    static float motor_dt;
    static float motor_start;
    for (;;)
    {
        motor_start = DWT_GetTimeline_ms();
        MotorControlTask();
        motor_dt = DWT_GetTimeline_ms() - motor_start;
        if (motor_dt > 1)
        {
            // LOGERROR("[freeRTOS] MOTOR Task is being DELAY! dt = [%f]", &motor_dt);
        }
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartDAEMONTASK(void const *argument)
{
    static float daemon_dt;
    static float daemon_start;
    // BuzzerInit();
    // LOGINFO("[freeRTOS] Daemon Task Start");
    for (;;)
    {
        // 100Hz
        daemon_start = DWT_GetTimeline_ms();
        DaemonTask();
        // BuzzerTask();
        daemon_dt = DWT_GetTimeline_ms() - daemon_start;
        if (daemon_dt > 10)
        {
            // LOGERROR("[freeRTOS] Daemon Task is being DELAY! dt = [%f]", &daemon_dt);
        }
        osDelay(10);
    }
}

__attribute__((noreturn)) void StartROBOTTASK(void const *argument)
{
    // static float robot_dt;
    // static float robot_start;
    // LOGINFO("[freeRTOS] ROBOT core Task Start");
    // 200Hz-500Hz,若有额外的控制任务如平衡步兵可能需要提升至1kHz
    for (;;)
    {
        // robot_start = DWT_GetTimeline_ms();
        // GYRO_buff_to_data();
        RobotTask();
        // robot_dt = DWT_GetTimeline_ms() - robot_start;
        // if (robot_dt > 5)
        // {
        // LOGERROR("[freeRTOS] ROBOT core Task is being DELAY! dt = [%f]", &robot_dt);
        // }
        osDelay(5);
    }
}

__attribute__((noreturn)) void StartOBJECTTASK(void const *argument)
{
    for (;;)
    {
        ObjectTask();
        osDelay(5);
    }
}

// __attribute__((noreturn)) void StartOPSTask(void const *argument)
// {
//     uint8_t buffer[OPS_FRAME_SIZE];
//     for (;;)
//     {
//         if (xQueueReceive(opsQueue, &buffer, portMAX_DELAY) == pdPASS)
//         {
//             // 处理接收到的数据
//             OPS_data_process(buffer);
//             osDelay(5);
//         }
//     }
// }

// __attribute__((noreturn)) void StartOPSTask(void const *argument)
// {
//     // uint8_t OPS ops_data_buffer;
//     for (;;)
//     {
//         // 处理接收到的数据
//         // OPS_data_process(ops_data_buffer);
//         // OPS_data_process();
//         osDelay(10);
//     }
// }