//
// Created by xhuanc on 2021/11/2.
//

#ifndef DEMO1_LAUNCHER_H
#define DEMO1_LAUNCHER_H

#include "can_receive.h"

/******************define******************/
//TODO:后期根据射速限制来修改摩擦轮的最大转速
#define FIRE_SPEED_MAX 6000
#define TRIGGER_CONTINUES_SPEED -3000
#define TRIGGER_REVERSE_SPEED 3000
#define FIRE_L  0
#define FIRE_R  1
#define TRIGGER 2
#define DEGREE_60_TO_ENCODER -49150.8f
//连发准备计时完成
#define CONTINUES_SHOOT_TIMING_COMPLETE() (HAL_GetTick()-continue_shoot_time>200)
#define CONTINUES_BLOCKED_JUDGE() (HAL_GetTick()-blocked_start_time>500)
#define TRIGGER_REVERSE_TIME_JUDGE() (HAL_GetTick()-reverse_start_time<50)
//摩擦轮转速PID
#define SHOOT_FIRE_L_PID_KP 10
#define SHOOT_FIRE_L_PID_KI 0.f
#define SHOOT_FIRE_L_PID_KD 0.f
#define SHOOT_FIRE_L_PID_MAX_OUT    16000
#define SHOOT_FIRE_L_PID_MAX_IOUT   0

#define SHOOT_FIRE_R_PID_KP 10
#define SHOOT_FIRE_R_PID_KI 0.f
#define SHOOT_FIRE_R_PID_KD 0.f
#define SHOOT_FIRE_R_PID_MAX_OUT    16000
#define SHOOT_FIRE_R_PID_MAX_IOUT   0

//拨弹电机角度环PID
#define SHOOT_TRI_ANGLE_PID_KP 30
#define SHOOT_TRI_ANGLE_PID_KI 0.f
#define SHOOT_TRI_ANGLE_PID_KD 0.1f
#define SHOOT_TRI_ANGLE_PID_MAX_OUT 1000
#define SHOOT_TRI_ANGLE_PID_MAX_IOUT 0

//拨弹电机速度环PID
#define SHOOT_TRI_SPEED_PID_KP  10
#define SHOOT_TRI_SPEED_PID_KI  0.f
#define SHOOT_TRI_SPEED_PID_KD  0.f
#define SHOOT_TRI_SPEED_PID_MAX_OUT 14000
#define SHOOT_TRI_SPEED_PID_MAX_IOUT 0

/******************struct&enum******************/

typedef enum{
    Fire_OFF=0,
    Fire_ON=1,
}fire_mode;

typedef enum{
    SHOOT_CLOSE=0,
    SHOOT_SINGLE=1,
    SHOOT_CONTINUES=2,
}trigger_cmd;

typedef struct {
    fire_mode fire_mode;//摩擦轮状态

    fire_mode fire_last_mode;//摩擦轮上一次状态

    trigger_cmd trigger_cmd;    //发射机构单发还是

    motor_2006_t fire_l;

    motor_2006_t fire_r;

    motor_2006_t trigger;

}launcher_t;

extern void launcher_init();
extern void launcher_mode_set();
extern void launcher_control();
extern void launcher_relax_handle();



#endif //DEMO1_LAUNCHER_H
