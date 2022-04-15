//
// Created by xhuanc on 2021/9/27.
//

#ifndef AVG_INFANTRY_CAN_RECEIVE_H
#define AVG_INFANTRY_CAN_RECEIVE_H
#include "struct_typedef.h"
#include "PID.h"
/******************** define *******************/

#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8192
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#define MOTOR_ECD_TO_ANGLE 0.0439453125     //        360  /8192
#define MOTOR_RAD_TO_ANGLE 57.29577951308238 // 360*2PI
#define MOTOR_ANGLE_TO_RAD  0.0174532925199433f
/******************** struct *******************/

//CAN_ID ��ö�ٲ�����CAN_1����CAN_2
typedef enum
{

    //������� ����ID
    CAN_MOTOR_0x200_ID = 0x200,
    //0X200��Ӧ�ĵ��ID
    CAN_CHASSIS_3508_MOTOR_1=0x201,
    CAN_CHASSIS_3508_MOTOR_2=0x202,
    CAN_CHASSIS_3508_MOTOR_3=0x203,
    CAN_CHASSIS_3508_MOTOR_4=0x204,


    //������� ����ID
    CAN_MOTOR_0x1FF_ID = 0x1FF,
    //0x1FF��Ӧ�ĵ����ID

#if 0
    CAN_CHASSIS_6020_RUDDER_1=0x205,
    CAN_CHASSIS_6020_RUDDER_2=0x206,
    CAN_CHASSIS_6020_RUDDER_3=0x207,
    CAN_CHASSIS_6020_RUDDER_4=0x208,

#endif

    CAN_LAUNCHER_3508_FIRE_L=0X203,
    CAN_LAUNCHER_3508_FIRE_R=0X204,
    CAN_LAUNCHER_2006_TRIGGER=0x205,

    //������� ����ID
    CAN_MOTOR_0x2FF_ID = 0x2FF,

    CAN_GIMBAL_6020_YAW=0x205,
    CAN_GIMBAL_6020_PITCH=0x206,

} can_msg_id_e;

typedef enum {
    CAN_1,
    CAN_2,
}CAN_TYPE;

//���������
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;

    int32_t round_cnt;   //�����ת����Ȧ��
    int32_t total_ecd;   //�����ת���ܱ�������ֵ

    uint16_t offset_ecd;//�����У׼����ֵ

    int32_t total_dis;   //����ܹ��ߵľ���

    float torque_round_cnt;//ת��ת���ĽǶȣ�-360-360��
    float real_round_cnt;//����ת���ĽǶ�
    float real_angle_deg;
} motor_measure_t;

typedef struct
{
    const motor_measure_t *motor_measure;

    fp32 speed;

    fp32 rpm_set;

    pid_t speed_p;

    int16_t give_current;


}motor_3508_t;

typedef struct
{

     motor_measure_t *motor_measure;

    pid_t angle_p;

    pid_t speed_p;


    fp32 max_relative_angle; //��

    fp32 min_relative_angle; //��

    fp32 relative_angle_get;
    fp32 relative_angle_set; //��

    fp32 absolute_angle_get;
    fp32 absolute_angle_set;//rad

    fp32 gyro_set;  //ת������
    int16_t give_current; //���յ���ֵ

}motor_6020_t;


typedef struct
{
    motor_measure_t *motor_measure;

    pid_t angle_p;//�ǶȻ�pid

    pid_t speed_p;//�ٶȻ�pid


    fp32 speed;//ת������ֵ

    int16_t give_current;

}motor_2006_t;

//typedef struct {
//    union {
//        float value;
//        uint8_t data[4];
//    }vx;
//
//    union {
//        float value;
//        uint8_t data[4];
//    }vy;
//}Vector_get_from_top;


/******************** extern *******************/



extern motor_measure_t motor_3508_measure[4];
extern motor_measure_t motor_6020_measure[4];
extern motor_measure_t motor_yaw_measure;
extern motor_measure_t motor_pitch_measure;
extern motor_measure_t motor_2006_measure[3];

extern void CAN_cmd_motor(CAN_TYPE can_type,can_msg_id_e CMD_ID,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern void CAN_cmd_chassis_rudder(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern fp32 motor_ecd_to_rad_change(uint16_t ecd, uint16_t offset_ecd);

extern fp32 motor_ecd_to_angle_change(uint16_t ecd,uint16_t offset_ecd);

extern void CAN_cmd_communication(CAN_TYPE can_type,can_msg_id_e CMD_ID,fp32 vx,fp32 vy);

#endif //AVG_INFANTRY_CAN_RECEIVE_H
