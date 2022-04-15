//
// Created by xhuanc on 2021/9/27.
//

#include "can_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "chassis.h"
#include"math.h"
#include "Detection.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/******************** define *******************/

//�������ݽ���
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

//����ܱ���ֵ�ļ���
#define get_motor_round_cnt(ptr) \
    {                            \
             if(ptr.ecd-ptr.last_ecd> 4192){ \
                ptr.round_cnt--;                    \
             }                   \
             else if(ptr.ecd-ptr.last_ecd<-4192)    \
             {                   \
                ptr.round_cnt++;            \
             }                   \
             ptr.total_ecd= ptr.round_cnt*8192+ptr.ecd-ptr.offset_ecd;\
                                 \
    }

//�����ʵ�������
#define get_motor_real_distance(ptr)\
{\
    get_motor_round_cnt(ptr);\
    ptr.torque_round_cnt=ptr.total_ecd/8192.f; \
    ptr.real_round_cnt=ptr.torque_round_cnt/19.f; \
    ptr.real_angle_deg=fmodf(ptr.real_round_cnt,1.0f);             \
    if(ptr.real_angle_deg<0.0f)ptr.real_angle_deg+=1.0f;                \
    ptr.real_angle_deg*=360.0f;     \
    ptr.total_dis=ptr.real_round_cnt*wheel_circumference;      \
}\

//3508���ٱ�
#define motor_3508_reduction_ratio 3591.0f/187.0f
//�����ܳ�
#define wheel_circumference 70*2*3.14f
/******************** variable *******************/



motor_measure_t motor_3508_measure[4];//0-3 �ֱ��Ӧ  RF,LF,LB,RB
motor_measure_t motor_6020_measure[4];//0-3 �ֱ��Ӧ  RF,LF,LB,RB
motor_measure_t motor_yaw_measure;
motor_measure_t motor_pitch_measure;
motor_measure_t motor_2006_measure[3];//0-2 �ֱ��Ӧ FIRE_L,FIRE_R,TRIGGER

#if isBalance
motor_measure_t motor_left_measure;
motor_measure_t motor_right_measure;
#endif

static CAN_TxHeaderTypeDef  tx_message;
static uint8_t              can_send_data[8];
extern chassis_t chassis;
//���ֵ���ķ��ͺ���
void CAN_cmd_motor(CAN_TYPE can_type,can_msg_id_e CMD_ID,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    tx_message.StdId = CMD_ID;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    can_send_data[0] = motor1 >> 8;
    can_send_data[1] = motor1;
    can_send_data[2] = motor2 >> 8;
    can_send_data[3] = motor2;
    can_send_data[4] = motor3 >> 8;
    can_send_data[5] = motor3;
    can_send_data[6] = motor4 >> 8;
    can_send_data[7] = motor4;

    if(can_type==CAN_1)
    {
        HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
    }
    else if(can_type==CAN_2)
    {
        HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);
    }

}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;

    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan == &hcan1) {
        switch (rx_header.StdId) {

#if isSteer
            case CAN_CHASSIS_6020_RUDDER_1:

            get_motor_measure(&motor_6020_measure[0],rx_data);

                break;
            case CAN_CHASSIS_6020_RUDDER_2:

            get_motor_measure(&motor_6020_measure[1],rx_data);

                break;

            case CAN_CHASSIS_6020_RUDDER_3:

            get_motor_measure(&motor_6020_measure[2],rx_data);

                break;

            case CAN_CHASSIS_6020_RUDDER_4:

            get_motor_measure(&motor_6020_measure[3],rx_data);

                break;
#endif
                case CAN_CHASSIS_3508_MOTOR_2:{
                    get_motor_measure(&motor_3508_measure[1], rx_data);
                    get_motor_real_distance(motor_3508_measure[LF]);
                    detect_handle(DETECT_CHASSIS_3508_LF);
                    }
                    break;
                case CAN_GIMBAL_6020_YAW: {
                    get_motor_measure(&motor_yaw_measure, rx_data);
                    detect_handle(DETECT_GIMBAL_6020_YAW);
                }break;

            default: {
                break;
            }
        }
    }
        else if (hcan == &hcan2) {
            switch (rx_header.StdId) {

            case CAN_LAUNCHER_3508_FIRE_L: {
                get_motor_measure(&motor_3508_measure[0], rx_data);
                detect_handle(DETECT_LAUNCHER_3508_FIRE_L);
            }break;

            case CAN_LAUNCHER_3508_FIRE_R: {
                get_motor_measure(&motor_3508_measure[1], rx_data);
                detect_handle(DETECT_LAUNCHER_3508_FIRE_R);
            }break;

            case CAN_LAUNCHER_2006_TRIGGER: {
                get_motor_measure(&motor_2006_measure[2], rx_data);
                get_motor_round_cnt(motor_2006_measure[2]);//��ȡת�����ֵ��ת��Ȧ�����ܱ���ֵ
                detect_handle(DETECT_LAUNCHER_2006_TRIGGER);
            }break;

            case CAN_GIMBAL_6020_PITCH: {
                get_motor_measure(&motor_pitch_measure, rx_data);
                detect_handle(DETECT_GIMBAL_6020_PITCH);
            }break;
                default:
                    break;
            }
        }
}

fp32 motor_ecd_to_rad_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}


//����������Ķ���  -180-180
fp32 motor_ecd_to_angle_change(uint16_t ecd,uint16_t offset_ecd)
{
    int16_t tmp=0;
    if(offset_ecd>=4096)
    {
        if(ecd>offset_ecd-4096)
        {
            tmp=ecd-offset_ecd;
        }
        else
        {
            tmp=ecd+8192-offset_ecd;
        }
    }
    else{
        if(ecd>offset_ecd+4096)
        {
            tmp=ecd-8192-offset_ecd;
        }
        else
        {
            tmp=ecd-offset_ecd;
        }
    }
    return (fp32)tmp/8192.f*360;
}


