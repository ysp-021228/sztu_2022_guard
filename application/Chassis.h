//
// Created by xhuanc on 2021/10/10.
//

#ifndef DEMO1_CHASSIS_H
#define DEMO1_CHASSIS_H

/*include*/
#include "struct_typedef.h"
#include "can_receive.h"
#include "PID.h"
#include "remote.h"
#include "user_lib.h"

/*define*/



#define isMekNum 1
#define isSteer 0
#define isBalance 0


//����ʼ����һ��ʱ��

#define CHASSIS_TASK_INIT_TIME 357

#define CHASSIS_Y_CHANNEL 0

#define CHASSIS_X_CHANNEL 1

#define CHASSIS_Z_CHANNEL 2

#define CHASSIS_MODE_CHANNEL 0

//������ϵ�� ��������ٶȺ�ң����֪ͨ�����������

#define CHASSIS_VX_RC_SEN 0.006f

#define CHASSIS_VY_RC_SEN 0.005f

#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f

#define CHASSIS_WZ_RC_SEN 0.01f

#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

#define CHASSIS_CONTROL_TIME_MS 2

//m3508ת���ɵ����ٶ�(m/s)�ı�����Ҫ�������Ӵ�С�� //TODO:����֮����Ҫ�޸�
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//�����˶��������ǰ���ٶ� mm/s
#define NORMAL_MAX_CHASSIS_SPEED_X 2500.0f

//�����˶��������ƽ���ٶ�  mm/s
#define NORMAL_MAX_CHASSIS_SPEED_Y 1500.0f

#define CHASSIS_WZ_SET_SCALE 0.1f

//������ת����PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 2.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 100.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

#define CHASSIS_3508_PID_KP  8.0f
#define CHASSIS_3508_PID_KI     1.0f
#define CHASSIS_3508_PID_KD     0.0f
#define CHASSIS_3508_PID_MAX_OUT 10000.0f
#define CHASSIS_3508_PID_MAX_IOUT 1000.0f

/**********************  RF �ǶȻ�  *************************/
#define CHASSIS_6020_RF_PID_KP_ANGLE     2.f
#define CHASSIS_6020_RF_PID_KI_ANGLE     0.01f
#define CHASSIS_6020_RF_PID_KD_ANGLE     0.0f
#define CHASSIS_6020_RF_PID_MAX_OUT_ANGLE 300.0f
#define CHASSIS_6020_RF_PID_MAX_IOUT_ANGLE 5.0f

/**********************  RF �ٶȻ�  *************************/
#define CHASSIS_6020_RF_PID_KP_SPEED   40.f
#define CHASSIS_6020_RF_PID_KI_SPEED    5.0f
#define CHASSIS_6020_RF_PID_KD_SPEED    20.0f
#define CHASSIS_6020_RF_PID_MAX_OUT_SPEED 20000.0f
#define CHASSIS_6020_RF_PID_MAX_IOUT_SPEED 6000.0f

/**********************  LF �ǶȻ�  *************************/
#define CHASSIS_6020_LF_PID_KP_ANGLE     2.f
#define CHASSIS_6020_LF_PID_KI_ANGLE     0.0f
#define CHASSIS_6020_LF_PID_KD_ANGLE     0.0f
#define CHASSIS_6020_LF_PID_MAX_OUT_ANGLE 300.0f
#define CHASSIS_6020_LF_PID_MAX_IOUT_ANGLE 5.0f

/**********************  LF �ٶȻ�  *************************/
#define CHASSIS_6020_LF_PID_KP_SPEED   40.f
#define CHASSIS_6020_LF_PID_KI_SPEED    5.0f
#define CHASSIS_6020_LF_PID_KD_SPEED    0.0f
#define CHASSIS_6020_LF_PID_MAX_OUT_SPEED 20000.0f
#define CHASSIS_6020_LF_PID_MAX_IOUT_SPEED 6000.0f

/**********************  LB �ǶȻ�  *************************/
#define CHASSIS_6020_LB_PID_KP_ANGLE     2.f
#define CHASSIS_6020_LB_PID_KI_ANGLE     0.05f
#define CHASSIS_6020_LB_PID_KD_ANGLE     0.0f
#define CHASSIS_6020_LB_PID_MAX_OUT_ANGLE 300.0f
#define CHASSIS_6020_LB_PID_MAX_IOUT_ANGLE 00.0f

/**********************  LB �ٶȻ�  *************************/
#define CHASSIS_6020_LB_PID_KP_SPEED   40.f
#define CHASSIS_6020_LB_PID_KI_SPEED    4.6f
#define CHASSIS_6020_LB_PID_KD_SPEED    0.0f
#define CHASSIS_6020_LB_PID_MAX_OUT_SPEED 20000.0f
#define CHASSIS_6020_LB_PID_MAX_IOUT_SPEED 7000.0f

/**********************  RB �ǶȻ�  *************************/
#define CHASSIS_6020_RB_PID_KP_ANGLE     50.f
#define CHASSIS_6020_RB_PID_KI_ANGLE     0.01f
#define CHASSIS_6020_RB_PID_KD_ANGLE     0.f
#define CHASSIS_6020_RB_PID_MAX_OUT_ANGLE 300.0f
#define CHASSIS_6020_RB_PID_MAX_IOUT_ANGLE 0.0f

/**********************  RB �ٶȻ�  *************************/
#define CHASSIS_6020_RB_PID_KP_SPEED   30.f
#define CHASSIS_6020_RB_PID_KI_SPEED    3.f
#define CHASSIS_6020_RB_PID_KD_SPEED    0.f
#define CHASSIS_6020_RB_PID_MAX_OUT_SPEED 30000.0f
#define CHASSIS_6020_RB_PID_MAX_IOUT_SPEED 2000.f



#if isBalance

/**********************  LW �ٶȻ�  *************************/
#define CHASSIS_2006_LW_PID_KP_SPEED   10.f
#define CHASSIS_2006_LW_PID_KI_SPEED    0.00f
#define CHASSIS_2006_LW_PID_KD_SPEED    0.00f
#define CHASSIS_2006_LW_PID_MAX_OUT_SPEED 10000.0f
#define CHASSIS_2006_LW_PID_MAX_IOUT_SPEED 10000.f

/**********************  RW �ٶȻ�  *************************/
#define CHASSIS_2006_RW_PID_KP_SPEED   10.f
#define CHASSIS_2006_RW_PID_KI_SPEED    0.f
#define CHASSIS_2006_RW_PID_KD_SPEED    0.f
#define CHASSIS_2006_RW_PID_MAX_OUT_SPEED 10000.0f
#define CHASSIS_2006_RW_PID_MAX_IOUT_SPEED 0.f

/**********************  ֱ����  *************************/
#define CHASSIS_VERTICAL_PID_KP   200.f
#define CHASSIS_VERTICAL_PID_KI    0.f
#define CHASSIS_VERTICAL_PID_KD    0.f
#define CHASSIS_VERTICAL_PID_MAX_OUT 14000.0f
#define CHASSIS_VERTICAL_PID_MAX_IOUT 0.f


#endif

#define chassis_start_buzzer buzzer_on  (31, 19999)
#define chassis_buzzer_off   buzzer_off()            //buzzer off���رշ�����

#define MAX_CHASSIS_VX_SPEED 5000.f
#define MAX_CHASSIS_VY_SPEED 5000.f
#define MAX_CHASSIS_VW_SPEED 2.f
#define CHASSIS_6020_RB_PID_KP_SPEED   40.f
#define CHASSIS_6020_RB_PID_KI_SPEED    5.0f
#define CHASSIS_6020_RB_PID_KD_SPEED    0.0f
#define CHASSIS_6020_RB_PID_MAX_OUT_SPEED 20000.0f
#define CHASSIS_6020_RB_PID_MAX_IOUT_SPEED 6000.0f

#define isMekNum 1
#define isSteer 0
#define isBalance 0



#define RC_TO_VX  (MAX_CHASSIS_VX_SPEED/660)
#define RC_TO_VY  (MAX_CHASSIS_VY_SPEED/660)
#define RC_TO_VW 0.4545454545454545f    //MAX_CHASSIS_VR_SPEED / RC_MAX_VALUE

#define GYRO_TO_RPM 94.2477796076937f //  ��/min

//���̻�е��Ϣ mm
#define WHEELBASE 430
#define WHEELTRACK 430
#define GIMBAL_OFFSET 0
#define PERIMETER 478
#define PERIMETER_STEEL 314
#define M3508_DECELE_RATIO 1.0f/19.0f
#define M3508_MAX_RPM 8500
#define W_Cdistance my_sqrt(my_pow(WHEELBASE) + my_pow(WHEELTRACK))
//ö�� �ṹ��

typedef enum
{
    CHASSIS_RELAX,
    CHASSIS_BACK,
    CHASSIS_AUTO,
    CHASSIS_STANDBY,
    CHASSIS_STARTING_POINT_CAL,
    CHASSIS_END_POINT_CAL,
    CHASSIS_TEST
} chassis_mode_e;

typedef enum {
    RF=0,
    LF,
    LB,
    RB
}chassis_motor_index;

typedef struct
{
    chassis_mode_e mode;

    chassis_mode_e last_mode;
    motor_3508_t motor_chassis[4];
    motor_6020_t motor_steer[4];

#if isBalance
    motor_2006_t motor_balance_wheel[2];
    pid_t Vertical_pid;
#endif

    pid_t chassis_vw_pid;

    fp32 vx;
    fp32 vy;
    fp32 vw;

//    fp32 vx_max_speed;  //ǰ����������ٶ� ��λm/s
//    fp32 vx_min_speed;  //���˷�������ٶ� ��λm/s
//    fp32 vy_max_speed;  //��������ٶ� ��λm/s
//    fp32 vy_min_speed;  //�ҷ�������ٶ� ��λm/s

} chassis_t;

//����

//��������
extern void chassis_task(void const *pvParameters);

//
#endif //DEMO1_CHASSIS_H

