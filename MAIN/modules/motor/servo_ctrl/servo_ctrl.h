#ifndef SERVO_CTRL_H_
#define SERVO_CTRL_H_

#include <stdint.h>
#include "main.h"

typedef struct _lobot_servo_
{ // If LobotServo is not defined elsewhere, add it here.
    uint8_t ID;
    uint16_t Position;
} LobotServo;

void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time);
void moveServosByArray(LobotServo servos[], uint8_t Num, uint16_t Time);
void moveServos(uint8_t Num, uint16_t Time, ...);
void runActionGroup(uint8_t numOfAction, uint16_t Times);
void stopActionGroup(void);
void setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed);
void setAllActionGroupSpeed(uint16_t Speed);
void getBatteryVoltage(void);

void ServoCtrl_Init(USART_TypeDef *usart_handle);

#endif // SERVO_CTRL_H_
