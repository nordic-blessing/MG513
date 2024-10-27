#include "menu.h"
#include "OLED.h"
#include "mg513.h"
#include "tim.h"
#include "pid.h"
#include "key.h"
#include "encoder.h"

int16_t this_y;
static uint16_t prevKey2State;
uint16_t currentKey2State;

extern int16_t encoder_num;
extern MotorMode Mode;
extern PID vec_l,vel_r;
extern PID ang_l,ang_r;

void Menu_option(void);//选项指针更新

//初始化
void Menu_MODE1_Init(void); //Speed Control
void Menu_MODE2_Init(void); //Position Control
void Menu_MODE3_Init(void); //Speed Follow
void Menu_MODE4_Init(void); //Position Follow
void Menu_MODE5_Init(void); //Cascade Control

//页面逻辑
void Menu_Mode1(void);      //Speed Control
void Menu_Mode2(void);      //Position Control
void Menu_Mode3(void);      //Speed Follow
void Menu_Mode4(void);      //Position Follow
void Menu_Mode5(void);      //Cascade Control

//选项逻辑
void Menu_Start_OK(void);   //主页
void Menu_Mode1_OK(void);   //Speed Control
void Menu_Mode2_OK(void);   //Position Control
void Menu_Mode3_OK(void);   //Speed Follow
void Menu_Mode4_OK(void);   //Position Follow
void Menu_Mode5_OK(void);   //Cascade Control

//选项指针更新
void Menu_option() {
    this_y = encoder_num;
    OLED_ClearArea(0, 0, 16, 64);
    OLED_ShowImage(0, (int16_t)(this_y * 9), 16, 9, This);
    OLED_Update();
}

//菜单主页初始化
void Menu_Init() {
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(16, 0, "Speed Control", OLED_6X8);
    OLED_ShowString(16, 9 * 1, "Position Control", OLED_6X8);
    OLED_ShowString(16, 9 * 2, "Speed Follow", OLED_6X8);
    OLED_ShowString(16, 9 * 3, "Position Follow", OLED_6X8);
    OLED_ShowString(16, 9 * 4, "Speed Curve", OLED_6X8);
    OLED_ShowString(16, 9 * 5, "Position Curve", OLED_6X8);
    OLED_ShowString(16, 9 * 6, "MODE7", OLED_6X8);
    OLED_Update();
}

//MODE1初始化
void Menu_MODE1_Init() {
    OLED_Clear();
    OLED_ShowString(16, 9 * 0, "OFF", OLED_6X8);
    OLED_ReverseArea(14, -1, 21, 9);
    OLED_ShowString(16, 9 * 1, "SetSpeed", OLED_6X8);
    OLED_ShowString(16, 9 * 2, "BACK", OLED_6X8);
    OLED_ShowString(16, 9 * 3, "Kp", OLED_6X8);
    OLED_ShowFloatNum(92, 9 * 3, vec_l.kp, 2, 2, OLED_6X8);
    OLED_ShowString(16, 9 * 4, "Ki", OLED_6X8);
    OLED_ShowFloatNum(92, 9 * 4, vec_l.ki, 2, 2, OLED_6X8);
    OLED_ShowString(16, 9 * 5, "Kd", OLED_6X8);
    OLED_ShowFloatNum(92, 9 * 5, vec_l.kd, 2, 2, OLED_6X8);
    OLED_Update();
}

//MODE2初始化
void Menu_MODE2_Init() {
    OLED_Clear();
    OLED_ShowString(16, 9 * 0, "OFF", OLED_6X8);
    OLED_ReverseArea(14, -1, 21, 9);
    OLED_ShowString(16, 9 * 1, "SetAngle", OLED_6X8);
    OLED_ShowString(16, 9 * 2, "BACK", OLED_6X8);
    OLED_Update();
}

//MODE3初始化
void Menu_MODE3_Init(){
    OLED_Clear();
    OLED_ShowString(16, 9*0, "OFF",OLED_6X8);
    OLED_ReverseArea(16,-1,21,9);
    OLED_ShowString(16,9*1,"SetSpeed",OLED_6X8);
    OLED_ShowString(16,9*2,"BACK",OLED_6X8);
    OLED_Update();
}

//MODE4初始化
void Menu_MODE4_Init() {
    OLED_Clear();
    OLED_ShowString(16, 9 * 0, "OFF", OLED_6X8);
    OLED_ReverseArea(14, -1, 21, 9);
    OLED_ShowString(16, 9 * 1, "Control Motor", OLED_6X8);
    OLED_ShowString(16, 9 * 2, "BACK", OLED_6X8);
    OLED_Update();
}

//MODE5初始化
void Menu_MODE5_Init() {
    OLED_Clear();
    OLED_ShowString(16, 9 * 0, "OFF", OLED_6X8);
    OLED_ReverseArea(14, -1, 21, 9);
    OLED_ShowString(16, 9 * 1, "Speed", OLED_6X8);
    OLED_ShowString(16, 9 * 2, "Acceleration", OLED_6X8);
    OLED_ShowString(16, 9 * 3, "BACK", OLED_6X8);
    OLED_Update();
}

//MODE6初始化
void Menu_MODE6_Init(){
    OLED_Clear();
    OLED_ShowString(16, 9 * 0, "OFF", OLED_6X8);
    OLED_ReverseArea(14, -1, 21, 9);
    OLED_ShowString(16, 9 * 1, "Angle", OLED_6X8);
    OLED_ShowString(16, 9 * 2, "Speed", OLED_6X8);
    OLED_ShowString(16, 9 * 3, "BACK", OLED_6X8);
    OLED_Update();
}

//主页逻辑
void Menu() {
    while (1) {
        if (encoder_num < 0) {
            encoder_num = 6;
            Menu_option();
        } else if (encoder_num <= 6) {
            Menu_option();
            if (ReadKeyState() == GPIO_PIN_RESET) {
                Menu_Start_OK();
            }
        } else {
            encoder_num = 0;
            Menu_option();
        }
    }
}

//主页各选项逻辑
void Menu_Start_OK() {
    switch (this_y) {
        //Speed_Control
        case 0: {
            encoder_num = 1;
            Mode = Speed_Control;
            mg513_InitPID();
            mg513_SetPID(Mode);
            Menu_MODE1_Init();
            Menu_Mode1();
        }
            break;
        //Position_Control
        case 1:{
            encoder_num = 1;
            Mode = Position_Control;
            mg513_InitPID();
            mg513_SetPID(Mode);
            Menu_MODE2_Init();
            Menu_Mode2();
        }
            break;
        //Speed_Follow
        case 2:
        {
            encoder_num = 1;
            Mode = Speed_Follow;
            mg513_InitPID();
            mg513_SetPID(Mode);
            Menu_MODE3_Init();
            Menu_Mode3();
        }
            break;
        //Position_Follow
        case 3:
        {
            encoder_num = 1;
            mg513_InitPID();
            Menu_MODE4_Init();
            Menu_Mode4();
        }
            break;
        //Speed_CurveControl
        case 4:
        {
            encoder_num = 1;
            Mode = Speed_CurveControl;
            mg513_InitPID();
            mg513_SetPID(Mode);
            Menu_MODE5_Init();
            Menu_Mode5();
        }
            break;
            //Position_CurveControl
        case 5:
        {
            encoder_num = 1;
            Mode = Position_CurveControl;
            mg513_InitPID();
            mg513_SetPID(Mode);
            Menu_MODE6_Init();
            Menu_Mode6();
        }
        case 6:
            break;
        default:
            break;
    }
}

//模式1页面逻辑
void Menu_Mode1() {
    while (1) {
        if (encoder_num < 1) {
            encoder_num = 2;
            Menu_option();
        } else if (encoder_num <= 2) {
            Menu_option();
            if (ReadKeyState() == GPIO_PIN_RESET) {
                encoder_num = 0;
                Menu_Mode1_OK();
            }
            currentKey2State = ReadKey2State();
            if (currentKey2State != prevKey2State) {
                if (currentKey2State) {
                    OLED_ShowString(16, 9 * 0, "ON.", OLED_6X8);
                    OLED_ReverseArea(14, -1, 21, 9);
                    OLED_Update();
                    HAL_Delay(100);
                    mg513_Start();
                } else {
                    OLED_ShowString(16, 9 * 0, "OFF", OLED_6X8);
                    OLED_ReverseArea(14, -1, 21, 9);
                    OLED_Update();
                    HAL_Delay(100);
                    mg513_Stop();
                }
                prevKey2State = currentKey2State;
            }
        } else {
            encoder_num = 1;
            Menu_option();
        }
    }
}

//模式1选项逻辑
void Menu_Mode1_OK() {
    switch (this_y) {
        case 1: {
            while (1) {
                static int16_t SetSpeed;
                SetSpeed+=(encoder_num * 10);
                encoder_num=0;
                OLED_ShowSignedNum(92, this_y*9, SetSpeed, 4, OLED_6X8);
                OLED_Update();
                if (ReadKeyState() == GPIO_PIN_RESET) {
                    encoder_num = 1;
                    setPIDTarget(&vec_l,SetSpeed);                   //更新目标速度
                    HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);     //关闭右电机
                    return;
                }
            }
        }
            break;
        case 2: {
            encoder_num = 0;
            Mode = Init;
            mg513_InitPID();
            Menu_Init();
            Menu();
        }
            break;
        default:
            break;
    }
}

//模式2页面逻辑
void Menu_Mode2(){
    while (1) {
        if (encoder_num < 1) {
            encoder_num = 2;
            Menu_option();
        } else if (encoder_num <= 2) {
            Menu_option();
            if (ReadKeyState() == GPIO_PIN_RESET) {
                encoder_num = 0;
                Menu_Mode2_OK();
            }
            currentKey2State = ReadKey2State();
            if (currentKey2State != prevKey2State) {
                if (currentKey2State) {
                    OLED_ShowString(16, 9 * 0, "ON.", OLED_6X8);
                    OLED_ReverseArea(14, -1, 21, 9);
                    mg513_Start();
                } else {
                    OLED_ShowString(16, 9 * 0, "OFF", OLED_6X8);
                    OLED_ReverseArea(14, -1, 21, 9);
                    mg513_Stop();
                }
                prevKey2State = currentKey2State;
            }
        } else {
            encoder_num = 1;
            Menu_option();
        }
    }
}

//模式2选项逻辑
void Menu_Mode2_OK(){
    switch (this_y) {
        case 1: {
            static int16_t SetAngle;
            while (1) {
                SetAngle+=(encoder_num * 90);
                encoder_num=0;
                OLED_ShowSignedNum(92, this_y*9, SetAngle, 4, OLED_6X8);
                OLED_Update();
                if (ReadKeyState() == GPIO_PIN_RESET) {
                    encoder_num = 1;
                    setPIDTarget(&ang_l,SetAngle);              //更新目标角度
                    HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);     //关闭右电机
                    return;
                }
            }
        }
            break;
        case 2: {
            encoder_num = 0;
            Mode = Init;
            mg513_InitPID();
            Menu_Init();
            Menu();
        }
            break;
        default:
            break;
    }
}

//模式3页面逻辑
void Menu_Mode3(){
    while (1) {
        if (encoder_num < 1) {
            encoder_num = 2;
            Menu_option();
        } else if (encoder_num <= 2) {
            Menu_option();
            if (ReadKeyState() == GPIO_PIN_RESET) {
                encoder_num = 0;
                Menu_Mode3_OK();
            }
            currentKey2State = ReadKey2State();
            if (currentKey2State != prevKey2State) {
                if (currentKey2State) {
                    OLED_ShowString(16, 9 * 0, "ON.", OLED_6X8);
                    OLED_ReverseArea(14, -1, 21, 9);
                    mg513_Start();
                } else {
                    OLED_ShowString(16, 9 * 0, "OFF", OLED_6X8);
                    OLED_ReverseArea(14, -1, 21, 9);
                    mg513_Stop();
                }
                prevKey2State = currentKey2State;
            }
        } else {
            encoder_num = 1;
            Menu_option();
        }
    }
}

//模式3选项逻辑
void Menu_Mode3_OK(){
    switch (this_y) {
        case 1: {
            static int16_t SetSpeed;
            while (1) {
                SetSpeed+=encoder_num;
                encoder_num=0;
                OLED_ShowImage(92,9*1,16,9,Selected);
                OLED_Update();
                setPIDTarget(&vec_l,SetSpeed);              //更新目标角度
                if (ReadKeyState() == GPIO_PIN_RESET) {
                    encoder_num = 1;
                    OLED_ClearArea(92,9*1,16,9);
                    return;
                }
            }
        }
            break;
        case 2: {
            encoder_num = 0;
            Mode = Init;
            mg513_InitPID();
            Menu_Init();
            Menu();
        }
            break;
        default:
            break;
    }
}

//模式4页面逻辑
void Menu_Mode4() {
    while (1) {
        mg513_SetPID(Mode);
        if (encoder_num < 1) {
            encoder_num = 2;
            Menu_option();
        } else if (encoder_num <= 2) {
            Menu_option();
            if (ReadKeyState() == GPIO_PIN_RESET) {
                encoder_num = 0;
                Menu_Mode4_OK();
            }
            currentKey2State = ReadKey2State();
            if (currentKey2State != prevKey2State) {
                if (currentKey2State) {
                    OLED_ShowString(16, 9 * 0, "ON.", OLED_6X8);
                    OLED_ReverseArea(14, -1, 21, 9);
                    mg513_Start();
                } else {
                    OLED_ShowString(16, 9 * 0, "OFF", OLED_6X8);
                    OLED_ReverseArea(14, -1, 21, 9);
                    mg513_Stop();
                }
                prevKey2State = currentKey2State;
            }
        } else {
            encoder_num = 1;
            Menu_option();
        }
    }
}

//模式4选项逻辑
void Menu_Mode4_OK(){
    switch (this_y) {
        case 1:{
            while (1){
                static uint8_t num;
                num += encoder_num;
                encoder_num = 0;
                if(num%2) {
                    OLED_ShowString(108,this_y*9,"-R-",OLED_6X8);
                    Mode = Position_Follow_R;
                }else{
                    OLED_ShowString(108,this_y*9,"-L-",OLED_6X8);
                    Mode = Position_Follow_L;
                }
                OLED_Update();
                if(ReadKeyState()==GPIO_PIN_RESET){
                    encoder_num = 1;
                    mg513_InitPID();
                    mg513_SetPID(Mode);
                    return;
                }
            }
        }
        case 2:{
            encoder_num = 0;
            Mode = Init;
            Menu_Init();
            Menu();
        }
    }
}

//模式5页面逻辑
void Menu_Mode5(){
    while (1) {
        if (encoder_num < 1) {
            encoder_num = 2;
            Menu_option();
        } else if (encoder_num <= 3) {
            Menu_option();
            if (ReadKeyState() == GPIO_PIN_RESET) {
                encoder_num = 0;
                Menu_Mode5_OK();
            }
            currentKey2State = ReadKey2State();
            if (currentKey2State != prevKey2State) {
                if (currentKey2State) {
                    OLED_ShowString(16, 9 * 0, "ON.", OLED_6X8);
                    OLED_ReverseArea(14, -1, 21, 9);
                    mg513_Start();
                } else {
                    OLED_ShowString(16, 9 * 0, "OFF", OLED_6X8);
                    OLED_ReverseArea(14, -1, 21, 9);
                    mg513_Stop();
                    vec_l.curve.aTimes = 0;
                }
                prevKey2State = currentKey2State;
            }
        } else {
            encoder_num = 1;
            Menu_option();
        }
    }
}

void Menu_Mode5_OK() {
    static int16_t SetSpeed;
    static float SetAcceleration;
    extern Encoder ecd_l;
    switch (this_y) {
        case 1: {
            while (1) {
                SetSpeed += (encoder_num * 10);
                encoder_num = 0;
                OLED_ShowSignedNum(92, this_y * 9, SetSpeed, 3, OLED_6X8);
                OLED_Update();
                if (ReadKeyState() == GPIO_PIN_RESET) {
                    encoder_num = 1;
                    setCurve(&vec_l.curve, ecd_l.velocity.angular, SetSpeed,SetAcceleration,380);
                    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);     //关闭右电机
                    return;
                }
            }
        }
            break;
        case 2:
        {
            while (1){
                SetAcceleration+=(encoder_num / 10.0);
                encoder_num = 0;
                 OLED_ShowFloatNum(104,this_y * 9,SetAcceleration,1,1,OLED_6X8);
                OLED_Update();
                if(ReadKeyState() == GPIO_PIN_RESET){
                    encoder_num = 2;
                    setCurve(&vec_l.curve, ecd_l.velocity.angular, SetSpeed,SetAcceleration,380);
                    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);     //关闭右电机
                    return;
                }
            }
        }
            break;
        case 3: {
            encoder_num = 0;
            Mode = Init;
            mg513_InitPID();
            Menu_Init();
            Menu();
        }
            break;
        default:
            break;
    }
}

//
void Menu_Mode6(){
    while (1) {
        if (encoder_num < 1) {
            encoder_num = 2;
            Menu_option();
        } else if (encoder_num <= 3) {
            Menu_option();
            if (ReadKeyState() == GPIO_PIN_RESET) {
                encoder_num = 0;
                Menu_Mode6_OK();
            }
            currentKey2State = ReadKey2State();
            if (currentKey2State != prevKey2State) {
                if (currentKey2State) {
                    OLED_ShowString(16, 9 * 0, "ON.", OLED_6X8);
                    OLED_ReverseArea(14, -1, 21, 9);
                    mg513_Start();
                } else {
                    OLED_ShowString(16, 9 * 0, "OFF", OLED_6X8);
                    OLED_ReverseArea(14, -1, 21, 9);
                    mg513_Stop();
                    ang_l.curve.aTimes = 0;
                }
                prevKey2State = currentKey2State;
            }
        } else {
            encoder_num = 1;
            Menu_option();
        }
    }
}

void Menu_Mode6_OK(){
    static int16_t SetSpeed;
    static float SetAngle;
    extern Encoder ecd_l;
    switch (this_y) {
        case 1: {
            while (1) {
                SetAngle += (encoder_num * 10);
                encoder_num = 0;
                OLED_ShowSignedNum(92, this_y * 9, SetAngle, 3, OLED_6X8);
                OLED_Update();
                if (ReadKeyState() == GPIO_PIN_RESET) {
                    encoder_num = 1;
                    setCurve(&vec_l.curve, ecd_l.velocity.angular, SetAngle,0,SetSpeed);
                    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);     //关闭右电机
                    return;
                }
            }
        }
            break;
        case 2:
        {
            while (1){
                SetSpeed+=(encoder_num * 10.0);
                encoder_num = 0;
                OLED_ShowNum(104,this_y * 9,SetSpeed,3,OLED_6X8);
                OLED_Update();
                if(ReadKeyState() == GPIO_PIN_RESET){
                    encoder_num = 2;
                    setCurve(&ang_l.curve, ecd_l.position.angle, SetSpeed,0,SetSpeed);
                    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);     //关闭右电机
                    return;
                }
            }
        }
            break;
        case 3: {
            encoder_num = 0;
            Mode = Init;
            mg513_InitPID();
            Menu_Init();
            Menu();
        }
            break;
        default:
            break;
    }
}