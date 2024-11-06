#include "servo_ctrl.h"
#include "bsp_usart.h"
#include <stdarg.h>
#include <string.h>
// #include "bool.h"

#define FRAME_HEADER 0x55            // 帧头
#define CMD_SERVO_MOVE 0x03          // 舵机移动指令
#define CMD_ACTION_GROUP_RUN 0x06    // 运行动作组指令
#define CMD_ACTION_GROUP_STOP 0x07   // 停止动作组指令
#define CMD_ACTION_GROUP_SPEED 0x0B  // 设置动作组运行速度
#define CMD_GET_BATTERY_VOLTAGE 0x0F // 获取电池电压指令

#define GET_LOW_BYTE(A) ((uint8_t)(A))         // 实现获得A的低八位
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8)) // 实现获得A的高八位

static uint8_t LobotTxBuf[128]; // 发送缓存区
uint16_t batteryVolt;

static USARTInstance *lobotUsartInstance;
/*===================== private =====================*/
/*********************************************************************************

Function: moveServo

Description： 控制单个舞机转动

Parameters: sevoID:舞机ID，Position:目标位置,Time:转动时间
舞机ID取值:0<=舞机ID<=31,Time取值: Time > 0

Return: 无返回

Others:
**********************************************************************************/
void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time)
{
    if (servoID > 31 || !(Time > 0))
    { // 舞机ID不能大于31,可根据对应控制板修改
        return;
    }
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER; // 填充帧头
    LobotTxBuf[2] = 8;
    LobotTxBuf[3] = CMD_SERVO_MOVE;          // 填充舞机移动指令
    LobotTxBuf[4] = 1;                       // 要控制的舞机个数
    LobotTxBuf[5] = GET_LOW_BYTE(Time);      // 取得时间的低八位
    LobotTxBuf[6] = GET_HIGH_BYTE(Time);     // 取得时间的高八位
    LobotTxBuf[7] = servoID;                 // 舞机ID
    LobotTxBuf[8] = GET_LOW_BYTE(Position);  // 取得目标位置的低八位
    LobotTxBuf[9] = GET_HIGH_BYTE(Position); // 取得目标位置的高八位

    if (lobotUsartInstance != NULL)
    {
        USARTSend(lobotUsartInstance, LobotTxBuf, 10, USART_TRANSFER_BLOCKING);
    }
}

/*********************************************************************************

Function: moveServosByArray

Description： 控制多个舞机转动

Parameters: servos[]:舞机结构体数组，Num:舞机个数,Time:转动时间
0 < Num <= 32,Time > 0

Return: 无返回

Others:
**********************************************************************************/
void moveServosByArray(LobotServo servos[], uint8_t Num, uint16_t Time)
{
    uint8_t index = 7;
    uint8_t i = 0;

    if (Num < 1 || Num > 32 || !(Time > 0))
    {
        return; // 舞机数不能为零和大于32，时间不能为零
    }
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER; // 填充帧头
    LobotTxBuf[2] = Num * 3 + 5;                  // 数据长度 = 要控制舞机数*3+5
    LobotTxBuf[3] = CMD_SERVO_MOVE;               // 填充舞机移动指令
    LobotTxBuf[4] = Num;                          // 要控制的舞机个数
    LobotTxBuf[5] = GET_LOW_BYTE(Time);           // 取得时间的低八位
    LobotTxBuf[6] = GET_HIGH_BYTE(Time);          // 取得时间的高八位

    for (i = 0; i < Num; i++)
    {                                                            // 循环填充舞机ID和对应目标位置
        LobotTxBuf[index++] = servos[i].ID;                      // 填充舞机ID
        LobotTxBuf[index++] = GET_LOW_BYTE(servos[i].Position);  // 填充目标位置低八位
        LobotTxBuf[index++] = GET_HIGH_BYTE(servos[i].Position); // 填充目标位置高八位
    }

    if (lobotUsartInstance != NULL)
    {
        USARTSend(lobotUsartInstance, LobotTxBuf, LobotTxBuf[2] + 2, USART_TRANSFER_BLOCKING); // 发送
    }
}

/*********************************************************************************

Function: moveServos

Description： 控制多个舞机转动

Parameters: Num:舞机个数,Time:转动时间,...:舞机ID,转动角，舞机ID,转动角度 如此类推

Return: 无返回

Others:
**********************************************************************************/
void moveServos(uint8_t Num, uint16_t Time, ...)
{
    uint8_t index = 7;
    uint8_t i = 0;
    uint16_t temp;
    va_list arg_ptr;

    va_start(arg_ptr, Time); // 取得可变参数首地址
    if (Num < 1 || Num > 32)
    {
        return; // 舞机数不能为零和大于32，时间不能小于0
    }
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER; // 填充帧头
    LobotTxBuf[2] = Num * 3 + 5;                  // 数据长度 = 要控制舞机数 * 3 + 5
    LobotTxBuf[3] = CMD_SERVO_MOVE;               // 舞机移动指令
    LobotTxBuf[4] = Num;                          // 要控制舞机数
    LobotTxBuf[5] = GET_LOW_BYTE(Time);           // 取得时间的低八位
    LobotTxBuf[6] = GET_HIGH_BYTE(Time);          // 取得时间的高八位

    for (i = 0; i < Num; i++)
    {                                // 从可变参数中取得并循环填充舞机ID和对应目标位置
        temp = va_arg(arg_ptr, int); // 可变参数中取得舞机ID
        LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp));
        temp = va_arg(arg_ptr, int);                          // 可变参数中取得对应目标位置
        LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp)); // 填充目标位置低八位
        LobotTxBuf[index++] = GET_HIGH_BYTE(temp);            // 填充目标位置高八位
    }

    va_end(arg_ptr); // 置空arg_ptr

    if (lobotUsartInstance != NULL)
    {
        USARTSend(lobotUsartInstance, LobotTxBuf, LobotTxBuf[2] + 2, USART_TRANSFER_BLOCKING); // 发送
    }
}

/*********************************************************************************

Function: runActionGroup

Description： 运行指定动作组

Parameters: NumOfAction:动作组序号, Times:执行次数

Return: 无返回

Others: Times = 0 时无限循环
**********************************************************************************/
void runActionGroup(uint8_t numOfAction, uint16_t Times)
{
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER; // 填充帧头
    LobotTxBuf[2] = 5;                            // 数据长度，数据帧除帧头部分数据字节数，此命令固定为5
    LobotTxBuf[3] = CMD_ACTION_GROUP_RUN;         // 填充运行动作组命令
    LobotTxBuf[4] = numOfAction;                  // 填充要运行的动作组号
    LobotTxBuf[5] = GET_LOW_BYTE(Times);          // 取得要运行次数的低八位
    LobotTxBuf[6] = GET_HIGH_BYTE(Times);         // 取得要运行次数的高八位

    if (lobotUsartInstance != NULL)
    {
        USARTSend(lobotUsartInstance, LobotTxBuf, 7, USART_TRANSFER_BLOCKING); // 发送
    }
}

/*********************************************************************************

Function: stopActionGroup

Description： 停止动作组运行

Parameters: Speed: 目标速度

Return: 无返回

Others:
**********************************************************************************/
void stopActionGroup(void)
{
    LobotTxBuf[0] = FRAME_HEADER; // 填充帧头
    LobotTxBuf[1] = FRAME_HEADER;
    LobotTxBuf[2] = 2;                     // 数据长度，数据帧除帧头部分数据字节数，此命令固定为2
    LobotTxBuf[3] = CMD_ACTION_GROUP_STOP; // 填充停止运行动作组命令

    if (lobotUsartInstance != NULL)
    {
        USARTSend(lobotUsartInstance, LobotTxBuf, 4, USART_TRANSFER_BLOCKING); // 发送
    }
}

/*********************************************************************************

Function: setActionGroupSpeed

Description： 设定指定动作组的运行速度

Parameters: NumOfAction: 动作组序号 , Speed:目标速度

Return: 无返回

Others:
**********************************************************************************/
void setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed)
{
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER; // 填充帧头
    LobotTxBuf[2] = 5;                            // 数据长度，数据帧除帧头部分数据字节数，此命令固定为5
    LobotTxBuf[3] = CMD_ACTION_GROUP_SPEED;       // 填充设定动作组速度命令
    LobotTxBuf[4] = numOfAction;                  // 填充要设定的动作组号
    LobotTxBuf[5] = GET_LOW_BYTE(Speed);          // 获得目标速度的低八位
    LobotTxBuf[6] = GET_HIGH_BYTE(Speed);         // 获得目标速度的高八位

    if (lobotUsartInstance != NULL)
    {
        USARTSend(lobotUsartInstance, LobotTxBuf, 7, USART_TRANSFER_BLOCKING); // 发送
    }
}

/*********************************************************************************

Function: setAllActionGroupSpeed

Description： 设置所有动作组的运行速度

Parameters: Speed: 目标速度

Return: 无返回

Others:
**********************************************************************************/
void setAllActionGroupSpeed(uint16_t Speed)
{
    setActionGroupSpeed(0xFF, Speed); // 调用动作组速度设定，组号为0xFF时设置所有组的速度
}

/*********************************************************************************

Function: getBatteryVoltage

Description： 发送获取电池电压命令

Parameters: Timeout：重试次数

Return: 无返回

Others:
**********************************************************************************/
void getBatteryVoltage(void)
{
    LobotTxBuf[0] = FRAME_HEADER; // 填充帧头
    LobotTxBuf[1] = FRAME_HEADER;
    LobotTxBuf[2] = 2;                       // 数据长度，数据帧除帧头部分数据字节数，此命令固定为2
    LobotTxBuf[3] = CMD_GET_BATTERY_VOLTAGE; // 填充获取电池电压命令

    if (lobotUsartInstance != NULL)
    {
        USARTSend(lobotUsartInstance, LobotTxBuf, 4, USART_TRANSFER_BLOCKING); // 发送
    }
}

/**
 * @brief 舵机控制板初始化函数
 */
void ServoCtrl_Init(USART_TypeDef *usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = NULL;
    conf.recv_buff_size = NULL;
    conf.usart_handle = usart_handle;
    lobotUsartInstance = USARTRegister(&conf);
    return;
}
