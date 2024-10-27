#include "pid.h"
#include "math.h"

//限幅
//float output                  需要限幅的值
//const float MAX_OUTPUT_ABS    限幅范围
float limitOutput(float output, const float MAX_OUTPUT_ABS) {
    output = output > +MAX_OUTPUT_ABS ? +MAX_OUTPUT_ABS : output;
    output = output < -MAX_OUTPUT_ABS ? -MAX_OUTPUT_ABS : output;
    return output;
}

//初始化参数
//初始化pid曲线调速参数
//PID* pid                      需要操作的PID地址
//const float MAX_OUTPUT        最大输出值
//const float MAX_E_I           最大误差积分值
void initCurveParma(PID *pid) {
    pid->curve.Max = 0;

    pid->curve.maxTimes = 0;
    pid->curve.aTimes = 0;

    pid->curve.current = 0;
    pid->curve.start = 0;
    pid->curve.target = 0;

    pid->curve.acceleration = 0;

    pid->curve.A = 0;
    pid->curve.B = 0;
    pid->curve.C = 0;
    pid->curve.D = 0;
}
void initPID(PID* pid, const float MAX_OUTPUT, const float MAX_E_I){
    // param
    pid->kp = 0;
    pid->ki = 0;
    pid->kd = 0;

    // TIO
    pid->target = 0;
    pid->input = 0;
    pid->output = 0;

    // error
    pid->error.now = 0;
    pid->error.last = 0;
    pid->error.integral = 0;

    //curve
    initCurveParma(pid);

    //limit
    pid->MAX_OUTPUT = MAX_OUTPUT;
    pid->MAX_ERROR_INTEGRAL = MAX_E_I;
}

//设置pid参数
//PID* pid                      需要操作的PID地址
//float kp                      kp值
//float ki                      ki值
//float kd                      kd值
void setPIDParam(PID* pid, float kp, float ki, float kd){
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

//设置pid目标值
//PID* pid                      需要操作的PID地址
//float target                  目标值
void setPIDTarget(PID* pid, float target){
    pid->target = target;
}

//设置曲线参数
void setCurve(Curve* curve, float start, float target, float acceleration, float Max) {
    curve->target = target;             //目标状态
    curve->start = start;               //初始状态
    curve->acceleration = acceleration; //速度控制时表示加速度，位置控制时值设置为0
    curve->Max = Max;                   //速度控制时表示最大速度，位置控制时表示目标速度
}

//速度环-增量式
void updatePID_Speed(PID* pid, float input){
    pid->input = input;
    pid->error.now = pid->target - pid->input;

    pid->output = pid->output_last
                  + pid->kp * (pid->error.now - pid->error.last)
                  + pid->ki * pid->error.now
                  + pid->kd * (pid->error.now - 2 * pid->error.last + pid->error.pre);
    pid->output = limitOutput(pid->output,pid->MAX_OUTPUT);

    pid->error.pre = pid->error.last;
    pid->error.last = pid->error.now;
    pid->output_last = pid->output;
}

//位置环-位置式
void updatePID_Position(PID* pid, float input){
    pid->input = input;
    pid->error.now = pid->target - pid->input;
    pid->error.integral += pid->error.now;
    pid->error.integral = limitOutput(pid->error.integral,pid->MAX_ERROR_INTEGRAL);

    pid->output = pid->kp * pid->error.now
                  + pid->ki * pid->error.integral
                  + pid->kd * (pid->error.now - pid->error.last);
    pid->output = limitOutput(pid->output,pid->MAX_OUTPUT);

    pid->error.last = pid->error.now;
}

//速度曲线
void VelocityCurve(Curve* curve) {
    //限幅
    if (curve->target > curve->Max) {
        curve->target = curve->Max;
    }
    if(curve->target < -curve->Max) {
        curve->target = -curve->Max;
    }

    //初始化阶段
    if (curve->maxTimes == 0 && curve->current != curve->target) {
        //计算最大加速时间      加速时间 = （目标速度 - 初始速度）/ 加速度
        float temp = 0;
        temp = fabsf(curve->target - curve->start);
        temp = temp / curve->acceleration;
        curve->maxTimes = (uint32_t) (temp) + 1;
        //初始化加速时间
        curve->aTimes = 0;
        //初始化曲线
        curve->A = curve->target - curve->start;
        curve->C = curve->start;
        if (fabsf(curve->A) > 0.01)                                                     //原点处函数值设为0.01，近似为0
            curve->B = 2.0 / curve->maxTimes * log((fabsf(curve->A) - 0.01) / 0.01);   //计算B的值，使函数从近似原点增长
    }

    //控制阶段
    if (curve->aTimes < curve->maxTimes) {
        //计算速度曲线
        curve->current = curve->A / (float)(1 + exp((0.0 - curve->B) * (curve->aTimes - curve->maxTimes / 2.0)))
                         + curve->C;
        //更新时间
        curve->aTimes++;
    }

        //达到目标速度
    else {
        curve->current = curve->target;
        curve->maxTimes = 0;
        curve->aTimes = 0;
    }
}

//位置曲线
void PositionCurve(Curve* curve) {
    //初始化阶段
    if (curve->maxTimes == 0 && curve->current != curve->target) {
        //初始化位置曲线
        curve->A = (float) ((curve->target - curve->start) / 4);
        curve->B = (float)(4 * curve->Max / curve->A);
        curve->D = curve->start;
        if(curve->A > 0.01) {
            curve->C = logf((curve->A - 0.01) / 0.01) / curve->B;
        }
        //计算最大加速时间
        curve->maxTimes = curve->A * 3 / curve->Max + 2 * curve->C;
        //初始化时间
        curve->aTimes = 0;
    }

    //控制阶段
    if(curve->aTimes < curve->maxTimes){
        //加速阶段
        if(curve->aTimes <= curve->C) {
            curve->current = curve->A / (1 + expf((0.0 - curve->B) * (curve->aTimes - curve->C))) + curve->D;
        }
            //匀速阶段
        else if(curve->aTimes >= (curve->maxTimes - curve->C)) {
            curve->current = curve->A / (1 + expf((0.0 - curve->B) * ((curve->aTimes - curve->maxTimes + curve->C))))
                             + curve->target - curve->A / 2.0;
        }

            //减速阶段
        else {
            curve->current = (float)(curve->Max * (curve->aTimes - curve->C) + curve->A / 2.0);
        }
        curve->aTimes++;
    }
        //到达目标位置
    else{
        curve->target = curve->current;
        curve->aTimes = 0;
        curve->maxTimes = 0;
    }
}
