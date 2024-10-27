#include "encoder.h"
#include "limits.h"

#define PI 3.1415926

//初始化编码器
void initEncoder(Encoder* ecd, const Parameter param){
    ecd->param = param;

    //初始化速度
    ecd->velocity.angular = 0;
    ecd->velocity.linear = 0;
    ecd->velocity.acceleration = 0;

    //初始化计数器
    ecd->counter.count_now = 0;
    ecd->counter.count_last = 0;
    ecd->counter.count_increment = 0;
    ecd->counter.count_total = 0;
    ecd->counter.count_overflow = 0;
    ecd->counter.JUMP_THRESHOLD = 30000;
    ecd->counter.TIMx_MAX_COUNT = 0;

    //初始化位置
    ecd->position.rotations = 0;    //圈数
    ecd->position.angle = 0;        //角度
    ecd->position.distance = 0;     //距离

    //初始化方向
    ecd->direction = INIT;
}

//重置编码器
void restEncoder(Encoder* ecd) {

    __HAL_TIM_SetCounter(ecd->param.tim_hander,0);

    //初始化速度
    ecd->velocity.angular = 0;
    ecd->velocity.linear = 0;
    ecd->velocity.acceleration = 0;

    //初始化计数器
    ecd->counter.count_now = 0;
    ecd->counter.count_last = 0;
    ecd->counter.count_increment = 0;
    ecd->counter.count_total = 0;
    ecd->counter.count_overflow = 0;
    ecd->counter.JUMP_THRESHOLD = 30000;
    ecd->counter.TIMx_MAX_COUNT = 0;

    //初始化位置
    ecd->position.rotations = 0;    //圈数
    ecd->position.angle = 0;        //角度
    ecd->position.distance = 0;     //距离

    //初始化方向
    ecd->direction = INIT;
}

//获取编码器状态（循环）
void updateEncoderLoop(Encoder* ecd, uint8_t loop_period){
    //direction
    ecd->direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(ecd->param.tim_hander);

    //------counter
    //counter_now
    ecd->counter.count_now = (uint16_t) __HAL_TIM_GET_COUNTER(ecd->param.tim_hander);
    //
    ecd->counter.count_increment = (int32_t)ecd->counter.count_now - (int32_t)ecd->counter.count_last;

    //counter_increment
    //溢出检测
    if(ecd->counter.count_increment > ecd->counter.JUMP_THRESHOLD){
        //down overflow
        ecd->counter.count_overflow--;

        //16bits or 32bits
        if(ecd->counter.count_last > USHRT_MAX || ecd->counter.count_now > USHRT_MAX){
            //32bits
            ecd->counter.TIMx_MAX_COUNT = UINT_MAX;     //2^32 - 1
        }else {
            //16bits
            ecd->counter.TIMx_MAX_COUNT = USHRT_MAX;    //2^16 - 1
        }

        //compensate - (TIMx_MAX_COUNT+1)
        ecd->counter.count_increment = ecd->counter.count_increment - ecd->counter.TIMx_MAX_COUNT - 1;
    } else if(ecd->counter.count_increment < -ecd->counter.JUMP_THRESHOLD){
        //up overflow
        ecd->counter.count_overflow++;

        //16bits or 32bits
        if(ecd->counter.count_last > USHRT_MAX || ecd->counter.count_now > USHRT_MAX){
            //32bits
            ecd->counter.TIMx_MAX_COUNT = UINT_MAX;     //2^32 - 1
        }else {
            //16bits
            ecd->counter.TIMx_MAX_COUNT = USHRT_MAX;    //2^16 - 1
        }

        //compensate + (TIMx_MAX_COUNT+1)
        ecd->counter.count_increment = ecd->counter.count_increment + ecd->counter.TIMx_MAX_COUNT + 1;
    }

    //counter_total
    ecd->counter.count_total += ecd->counter.count_increment;

    //------position
    ecd->position.rotations = (float) ecd->counter.count_total /
                              ((float) ecd->param.multiple * ecd->param.reduction_ratio * ecd->param.ppr);
    ecd->position.angle = ecd->position.rotations * 360;
    ecd->position.distance = ecd->position.rotations / 60.0 * PI * 2.0 * ecd->param.r;

    //------velocity
    ecd->velocity.angular = (float) (ecd->counter.count_increment /
                                     ((float) ecd->param.multiple * ecd->param.reduction_ratio * ecd->param.ppr) *
                                     1000.0 / loop_period * 60);
    ecd->velocity.acceleration = (float) (ecd->velocity.angular / loop_period);
    ecd->velocity.linear = (float) (ecd->velocity.angular / 60.0 * PI * 2.0 * ecd->param.r);

    //更新count_last
    ecd->counter.count_last = ecd->counter.count_now;
}
