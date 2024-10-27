#ifndef __PID_H__
#define __PID_H__

#include "main.h"

//误差
typedef struct {
    float now;          //当前误差
    float last;         //上一次误差
    float pre;          //
    float integral;     //累积误差
}Error;

//曲线
typedef struct {
    float start;       //初始
    float current;     //当前
    float target;      //目标
    float acceleration;     //加速度
    float Max;         //速度控制时表示最大速度，位置控制时表示目标速度
    uint32_t aTimes;        //时间
    uint32_t maxTimes;      //控制总时间
    float A,B,C,D;
}Curve;

typedef struct {
    // kp ki kd
    float kp, ki, kd;

    // TIO
    float target, input, output, output_last;

    // Error
    Error error;

    //Curve
    Curve curve;

    //limit
    float MAX_OUTPUT;
    float MAX_ERROR_INTEGRAL;
}PID;

void initPID(PID* pid, const float MAX_OUTPUT, const float MAX_E_I);
void setPIDParam(PID* pid, float kp, float ki, float kd);
void setPIDTarget(PID* pid, float target);
void setCurve(Curve* curve, float start, float target, float acceleration, float Max);
void updatePID_Position(PID* pid, float input);
void updatePID_Speed(PID* pid, float input);
void VelocityCurve(Curve *curve);
void PositionCurve(Curve* curve);

#endif //__PID_H__
