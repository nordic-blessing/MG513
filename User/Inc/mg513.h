#ifndef __MG513_H__
#define __MG513_H__

#include "main.h"

typedef enum {
    LEFT = 0,
    RIGHT = 1
}Motor;

typedef enum {
    Init,
    Speed_Control,
    Position_Control,
    Speed_Follow,
    Position_Follow_L,
    Position_Follow_R,
    Speed_CurveControl,
    Position_CurveControl
}MotorMode;

//控制电机状态
#define AIN1(state) HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, (GPIO_PinState)(state));
#define AIN2(state) HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, (GPIO_PinState)(state));
#define BIN1(state) HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, (GPIO_PinState)(state));
#define BIN2(state) HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, (GPIO_PinState)(state));

void mg513_Start(void);         //打开电机
void mg513_Stop(void);          //暂停电机
void mg513_EncoderInit(void);   //编码器初始化
void mg513_InitPID(void);       //初始化电机控制环
void mg513_SetPID(MotorMode);   //设置电机控制环参数

#endif //__MG513_H__
