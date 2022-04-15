//
// Created by xhuanc on 2021/10/13.
//

#ifndef DEMO1_GIMBAL_H
#define DEMO1_GIMBAL_H
/*      Include     */

#include "can_receive.h"
#include "PID.h"
#include "remote.h"
#include "AHRS.h"

/*      define     */

#define GIMBAL_TASK_INIT_TIME 200
#define YAW_CHANNEL 2
#define PITCH_CHANNEL 3

#define RC_TO_YAW 0.002
#define RC_TO_PITCH 0.002
#define MAX_ABS_ANGLE 50
#define MIN_ABS_ANGLE -25
#define MAX_PITCH_RELA_ANGLE 10
#define MIN_PITCH_RELA_ANGLE -25
#define MAX_YAW_RELA_ANGLE 80
#define MIN_YAW_RELA_ANGLE -80
//云台转动速度系数
#define GIMBAL_RC_MOVE_RATIO_PIT 0.05f
#define GIMBAL_RC_MOVE_RATIO_YAW 0.1f
#define GIMBAL_YAW_PATROL_SPEED 0.05
#define GIMBAL_PITCH_PATROL_SPEED 0.08

#define GIMBAL_YAW_ANGLE_PID_KP     8.0f
#define GIMBAL_YAW_ANGLE_PID_KI     0.0f
#define GIMBAL_YAW_ANGLE_PID_KD     180.0f
#define GIMBAL_YAW_ANGLE_MAX_OUT    100.f
#define GIMBAL_YAW_ANGLE_MAX_IOUT   30.f

#define GIMBAL_YAW_SPEED_PID_KP     120.f
#define GIMBAL_YAW_SPEED_PID_KI     0.0f
#define GIMBAL_YAW_SPEED_PID_KD     0.0f
#define GIMBAL_YAW_SPEED_MAX_OUT    12000.f
#define GIMBAL_YAW_SPEED_MAX_IOUT   1000.f

//#define GIMBAL_YAW_ANGLE_PID_KP     3.5f
//#define GIMBAL_YAW_ANGLE_PID_KI     0.0f
//#define GIMBAL_YAW_ANGLE_PID_KD     120.0f
//#define GIMBAL_YAW_ANGLE_MAX_OUT    80.f
//#define GIMBAL_YAW_ANGLE_MAX_IOUT   30.f
//
//#define GIMBAL_YAW_SPEED_PID_KP     170.f
//#define GIMBAL_YAW_SPEED_PID_KI     2.0f
//#define GIMBAL_YAW_SPEED_PID_KD     0.0f
//#define GIMBAL_YAW_SPEED_MAX_OUT    12000.f
//#define GIMBAL_YAW_SPEED_MAX_IOUT   6000.f

//#define GIMBAL_YAW_ANGLE_PID_KP     23.f
//#define GIMBAL_YAW_ANGLE_PID_KI     0.0f
//#define GIMBAL_YAW_ANGLE_PID_KD     0.0f
//#define GIMBAL_YAW_ANGLE_MAX_OUT    2000.f
//#define GIMBAL_YAW_ANGLE_MAX_IOUT   30.f
//
//#define GIMBAL_YAW_SPEED_PID_KP     30.f
//#define GIMBAL_YAW_SPEED_PID_KI     0.008f
//#define GIMBAL_YAW_SPEED_PID_KD     20.0f
//#define GIMBAL_YAW_SPEED_MAX_OUT    30000.f
//#define GIMBAL_YAW_SPEED_MAX_IOUT   2000.f

//#define GIMBAL_YAW_ANGLE_PID_KP     20.f
//#define GIMBAL_YAW_ANGLE_PID_KI     0.0f
//#define GIMBAL_YAW_ANGLE_PID_KD     10.0f
//#define GIMBAL_YAW_ANGLE_MAX_OUT    260.f
//#define GIMBAL_YAW_ANGLE_MAX_IOUT   30.f
//
//#define GIMBAL_YAW_SPEED_PID_KP     45.f
//#define GIMBAL_YAW_SPEED_PID_KI     0.02f
//#define GIMBAL_YAW_SPEED_PID_KD     10.0f
//#define GIMBAL_YAW_SPEED_MAX_OUT    30000.f
//#define GIMBAL_YAW_SPEED_MAX_IOUT   2000.f


#define GIMBAL_PITCH_ANGLE_PID_KP  41.f
#define GIMBAL_PITCH_ANGLE_PID_KI   0.f
#define GIMBAL_PITCH_ANGLE_PID_KD   40.f
#define GIMBAL_PITCH_ANGLE_MAX_OUT  360.f
#define GIMBAL_PITCH_ANGLE_MAX_IOUT 10.f

#define GIMBAL_PITCH_SPEED_PID_KP   30.0f
#define GIMBAL_PITCH_SPEED_PID_KI   0.1f
#define GIMBAL_PITCH_SPEED_PID_KD   8.f
#define GIMBAL_PITCH_SPEED_MAX_OUT  20000.f
#define GIMBAL_PITCH_SPEED_MAX_IOUT 2000.f

//#define GIMBAL_PITCH_ANGLE_PID_KP   0.f
//#define GIMBAL_PITCH_ANGLE_PID_KI   0.f
//#define GIMBAL_PITCH_ANGLE_PID_KD   0.f
//#define GIMBAL_PITCH_ANGLE_MAX_OUT  1000.f
//#define GIMBAL_PITCH_ANGLE_MAX_IOUT 2000.f
//
//#define GIMBAL_PITCH_SPEED_PID_KP   10.f
//#define GIMBAL_PITCH_SPEED_PID_KI   0.f
//#define GIMBAL_PITCH_SPEED_PID_KD   0.f
//#define GIMBAL_PITCH_SPEED_MAX_OUT  20000.f
//#define GIMBAL_PITCH_SPEED_MAX_IOUT 6000.f

/*      结构体和枚举     */

typedef enum {
    GIMBAL_RELAX=0,//云台失能
    GIMBAL_BACK,
    GIMBAL_ACTIVE,
    GIMBAL_PATROL,
    GIMBAL_AUTO
}gimbal_mode_e;

typedef struct {
    motor_6020_t yaw;
    motor_6020_t pitch;
    motor_2006_t trigger;

    gimbal_mode_e mode;
    gimbal_mode_e last_mode;

//    AHRS_Eulr_t*Eulr;   //姿态角

    bool_t yaw_is_back;
    bool_t pitch_is_back;
}gimbal_t;

extern void gimbal_task(void const*pvParameters);


#endif //DEMO1_GIMBAL_H
