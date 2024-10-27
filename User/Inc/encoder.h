#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "stdint.h"
#include "tim.h"

typedef struct {
    uint8_t multiple;               //倍频
    float   reduction_ratio;        //电机减速比
    uint8_t ppr;                    //编码器线数
    float   r;                      //轮子半径
    TIM_HandleTypeDef*  tim_hander; //用于编码器读取AB相进行编码的指针
}Parameter;

typedef struct {
    float angular;                  //角速度   rpm
    float linear;                   //线速度   m/s
    float acceleration;             //角加速度
}Velocity;

typedef struct {
    float rotations;               //电机输出轴转动圈数
    float distance;                //轮子移动距离    m
    float angle;                   //轮子转动角度    °
}Position;

typedef struct {
    uint16_t count_now;               //编码器当前计数
    uint16_t count_last;              //编码器上次计数
    int32_t count_increment;         //编码器两帧增量计数
    int32_t count_total;             //编码器总计数

    int64_t count_overflow;           //计数器寄存器 TIMx_CNT溢出次数     -正数-正转上溢出   -负数-反转下溢出

    int32_t JUMP_THRESHOLD;           //计数器溢出阈值
    uint32_t TIMx_MAX_COUNT;          //16bits-TIMx = 2^16 - 1      32bits-TIMx = 2^32 - 1
}Counter;

typedef enum{
    UP=0,       //前进
    DOWN=1,     //后退
    INIT=2      //初始化状态
}Direction;

typedef struct {
    Parameter param;

    Direction direction;

    Velocity velocity;

    Position position;

    Counter counter;
}Encoder;

void initEncoder(Encoder* ecd, const Parameter param);          //初始化编码器
void restEncoder(Encoder* ecd);          //编码器计数器清零
void updateEncoderLoop(Encoder* ecd, uint8_t loop_period);      //在循环函数中更新编码器状态

#endif //__ENCODER_H__
