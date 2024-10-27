#include "mg513.h"
#include "pid.h"
#include "tim.h"
#include "encoder.h"
#include "filter.h"

MotorMode Mode;             //电机模式
Encoder ecd_l,ecd_r;        //编码器
PID vec_l,vec_r;            //速度环   pid
PID ang_l,ang_r;            //位置环   p

//编码器初始化
void mg513_EncoderInit() {
    //左编码器
    Parameter left_param = {4, 13, 28, 0.065, &htim2};
    initEncoder(&ecd_l, left_param);

    //右编码器
    Parameter right_param = {4, 13, 28, 0.065, &htim3};
    initEncoder(&ecd_r, right_param);
}

//pid参数初始化
void mg513_InitPID(){
    initPID(&vec_l, 2000, 4000);
    initPID(&vec_r, 2000, 4000);
    initPID(&ang_l, 2000, 4000);
    initPID(&ang_r, 2000, 4000);
}

//设置pid参数
void mg513_SetPID(MotorMode mode) {
    if (mode == Speed_Control) {
        //速度控制
        setPIDParam(&vec_l, 5, 0.8, 6);         //速度环    4.95, 0.8, 5
    } else if (mode == Position_Control) {
        //位置控制
        setPIDParam(&vec_l, 4.95, 0.8, 5);         //速度环
        setPIDParam(&ang_l, 0.5, 0, 0.0);          //位置环
    } else if (mode == Speed_Follow) {
        //速度跟随
        setPIDParam(&vec_l, 10, 0.5, 0);
    } else if (mode == Position_Follow_L) {
        //位置跟随  左
        setPIDParam(&ang_r, 80, 0, 50);
    } else if (mode == Position_Follow_R) {
        //位置跟随  右
        setPIDParam(&ang_l, 60, 0, 0);
    } else if (mode == Speed_CurveControl){
        //速度曲线控速
        setPIDParam(&vec_l,10,1.5,0);
    } else if (mode == Position_CurveControl){
        setPIDParam(&ang_l,10,0.2,0.1);
    }
}

//电机初始化
void mg513_Start() {
    restEncoder(&ecd_l);
    restEncoder(&ecd_r);
    //PWM（PSC 72-1    ARR 2000）
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);        //PWM_左
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);        //PWM_右
    //Encoder Mode（TI1 and TI2      ARR 65535）
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1|TIM_CHANNEL_2); //encoder_左
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1|TIM_CHANNEL_2);//encoder_右
    //TIM4 global interrupt （PSC 72-1     ARR 10000-1）
    HAL_TIM_Base_Start_IT(&htim4);
}

//电机停止
void mg513_Stop()
{
    restEncoder(&ecd_l);
    restEncoder(&ecd_r);
    //motor
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);        //PWM
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_1|TIM_CHANNEL_2);  //编码器模式
    HAL_TIM_Encoder_Stop(&htim3,TIM_CHANNEL_1|TIM_CHANNEL_2);
    HAL_TIM_Base_Stop_IT(&htim4);                          //定时器中断
}

//取绝对值
float ABS(float input) {
    return input > 0 ? input : -input;
}
//限幅
float Limit(float output, const float MAX_OUTPUT) {
    if (output >= 0)
        output = output > MAX_OUTPUT ? MAX_OUTPUT : output;
    else
        output = output < -MAX_OUTPUT ? -MAX_OUTPUT : output;
    return output;
}

//电机PWM驱动
void mg513_PWM(Motor l_or_r, int16_t pwm_val) {
    if (l_or_r == LEFT) {
        //左侧电机
        if (pwm_val > 0) {
            //正转    AIN1=3v3 AIN2=GND
            AIN1(1);
            AIN2(0);
            pwm_val = Limit(ABS(pwm_val), 2000);    //限值
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_val);      //PWM_A
        } else if (pwm_val < 0) {
            //反转    AIN1=GND AIN2=3v3
            AIN1(0);
            AIN2(1)
            pwm_val = Limit(ABS(pwm_val), 2000);    //限值
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_val);      //PWM_A
        } else {
            //停转    AIN1=GND AIN2=GND
            AIN1(0);
            AIN2(0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        }
    } else if (l_or_r == RIGHT) {
        //右侧电机
        if (pwm_val > 0) {
            //正转    BIN1=3v3 BIN2=GND
            BIN1(1);
            BIN2(0);
            pwm_val = Limit(ABS(pwm_val), 2000);    //限值
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_val);      //PWM_B
        } else if (pwm_val < 0) {
            //反转    BIN1=GND BIN2=3v3
            BIN1(0);
            BIN2(1);
            pwm_val = Limit(ABS(pwm_val), 2000);    //限值
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_val);      //PWM_B
        } else {
            //停转    BIN1=GND BIN2=GND
            BIN1(0);
            BIN2(0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
        }
    } else {
        return;
    }
}

//中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim4) {
        //速度环控制--增量式pid     (左电机)
        if (Mode == Speed_Control) {
            updateEncoderLoop(&ecd_l, 10);
            float filtered_velocity = movingAverageFilter(ecd_l.velocity.angular);      //低通滤波
            updatePID_Speed(&vec_l, filtered_velocity);
            mg513_PWM(LEFT, vec_l.output);
            printf("%.2f,%.2f\n", ecd_l.velocity.angular, vec_l.target);
        }
        //位置环控制--串级pid(外级位置环，内级速度环）     (左电机)
        else if (Mode == Position_Control) {
            updateEncoderLoop(&ecd_l, 10);
            updatePID_Position(&ang_l, ecd_l.position.angle);
            ang_l.output = Limit(ang_l.output,200);
            setPIDTarget(&vec_l, ang_l.output);
            float filtered_velocity = movingAverageFilter(ecd_l.velocity.angular);      //滑动平均滤波
            updatePID_Speed(&vec_l, ecd_l.velocity.angular);
            mg513_PWM(LEFT, vec_l.output);
            printf("%.2f,%.2f,%.2f\n", ecd_l.position.angle, ang_l.target, ecd_l.velocity.angular);
        }
        //速度跟随
        else if (Mode == Speed_Follow) {
            updateEncoderLoop(&ecd_l, 10);
            float filtered_velocity = lowPassFilter(ecd_l.velocity.angular);      //低通滤波
            updatePID_Speed(&vec_l, filtered_velocity);
            mg513_PWM(LEFT, vec_l.output);
            printf("%.2f,%.2f\n", ecd_l.velocity.angular, vec_l.target);
        }
        //位置跟随控制        （左电机为主电机）
        else if (Mode == Position_Follow_L) {
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
            updateEncoderLoop(&ecd_l, 10);
            float position_l = ecd_l.position.angle;
            setPIDTarget(&ang_r, position_l);
            updateEncoderLoop(&ecd_r, 10);
            updatePID_Position(&ang_r, ecd_r.position.angle);
            mg513_PWM(RIGHT, ang_r.output);
            printf("%.2f,%.2f\n", ecd_r.position.angle, ecd_l.position.angle);
        }
        //位置跟随控制        （右电机为主电机）
        else if (Mode == Position_Follow_R) {
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
            updateEncoderLoop(&ecd_r, 10);
            float position_r = ecd_r.position.angle;
            setPIDTarget(&ang_l, position_r);
            updateEncoderLoop(&ecd_l, 10);
            updatePID_Position(&ang_l, ecd_l.position.angle);
            mg513_PWM(LEFT, ang_l.output);
            printf("%.2f,%.2f\n", ecd_r.position.angle, ecd_l.position.angle);
        }
        //速度曲线规划
        else if (Mode == Speed_CurveControl) {
            updateEncoderLoop(&ecd_l, 10);
            VelocityCurve(&vec_l.curve);
            setPIDTarget(&vec_l, vec_l.curve.current);
            float filtered_velocity = lowPassFilter(ecd_l.velocity.angular);      //滑动平均滤波
            updatePID_Speed(&vec_l, filtered_velocity);
            mg513_PWM(LEFT, vec_l.output);
            printf("%.2f,%.2f\n", vec_l.target, ecd_l.velocity.angular);
        }
            //位置曲线控制
        else if(Mode == Position_CurveControl){
            updateEncoderLoop(&ecd_l,10);
            PositionCurve(&ang_l.curve);
            setPIDTarget(&ang_l,ang_l.curve.current);
            updatePID_Position(&ang_l,ecd_l.position.angle);
            mg513_PWM(LEFT,ang_l.output);
            printf("%.2f,%.2f,%.2f\n",ang_l.target,ecd_l.position.angle,ecd_l.velocity.angular);
        }
    }
}
