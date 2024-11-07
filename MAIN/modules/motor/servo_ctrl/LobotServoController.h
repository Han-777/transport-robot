/*******************************************************************************
 * 文件名: LobotServoController.h
 * 作者: 深圳乐幻索尔科技
 * 日期：20160806
 * LSC系列舵机控制板二次开发示例
 *******************************************************************************/

#ifndef LOBOTSERVOCONTROLLER_H_
#define LOBOTSERVOCONTROLLER_H_

#include <stdint.h>
#include "main.h"

void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time);
void moveServosByArray(LobotServo servos[], uint8_t Num, uint16_t Time);
void moveServos(uint8_t Num, uint16_t Time, ...);
void runActionGroup(uint8_t numOfAction, uint16_t Times);
void stopActionGroup(void);
void setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed);
void setAllActionGroupSpeed(uint16_t Speed);
void getBatteryVoltage(void);

void ServoCtrl_Init(USART_TypeDef *usart_handle);

#endif
