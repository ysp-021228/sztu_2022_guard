//
// Created by xhuanc on 2021/10/13.
//

/*  Include */
#include "Gimbal.h"
#include "main.h"
#include "cmsis_os.h"
#include "launcher.h"
#include "can_receive.h"
#include "user_lib.h"
#include "Atti.h"
#include "Auto.h"
#include "Detection.h"
#include "key_board.h"
/*      define      */

/*      ����      */
gimbal_t gimbal;
extern RC_ctrl_t rc_ctrl;
//extern AHRS_Eulr_t Eulr;
extern launcher_t launcher;
extern Eulr_t Eulr;
extern fp32 INS_angle[3];
extern Vision_info_get Vision_info;
extern key_board_t KeyBoard;
first_order_filter_type_t pitch_first_order_set;
first_order_filter_type_t pitch_current_first_order_set;
first_order_filter_type_t pitch_speed_in;
first_order_filter_type_t yaw_first_order_set;
first_order_filter_type_t yaw_speed;
first_order_filter_type_t yaw_speed_in;
first_order_filter_type_t yaw_angle;
first_order_filter_type_t yaw_angle_in;
first_order_filter_type_t yaw_current_first_order_set;
uint8_t gimbal_up_flag;
uint8_t gimbal_down_flag;
uint8_t gimbal_left_flag;
uint8_t gimbal_right_flag;
fp32 debug=0.002;

/*      ����������   */
static void gimbal_init();
static void gimbal_mode_set();
static void gimbal_back_handle();
static void gimbal_active_handle();
static void gimbal_relax_handle();
static void gimbal_ctrl_loop_cal();
static void gimbal_angle_update();
static void gimbal_patrol_handle();
static void gimbal_auto_handle();

void gimbal_task(void const*pvParameters)
{
    //�����ʼ��ʱ��
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    //��̨��ʼ��
    gimbal_init();

    //���������ʼ��
    launcher_init();

    //TODO:�жϵ���Ƿ�����

    //��������
    while(1){

        gimbal_angle_update();//���¾��ԡ���ԽǶȽ���ֵ

        gimbal_mode_set();//����ң����������̨����ģʽ

        launcher_mode_set();//����ģʽ����

        switch (gimbal.mode) {

            case GIMBAL_RELAX://��̨ʧ��
                gimbal_relax_handle();
                break;

            case GIMBAL_BACK://��̨����
                gimbal_back_handle();
//                gimbal_ctrl_loop_cal();
                break;

            case GIMBAL_ACTIVE://��̨����
                gimbal_active_handle();  //�õ�ң��������̨����Ŀ���
                gimbal_ctrl_loop_cal();
                break;

            case GIMBAL_PATROL://��̨Ѳ��
                gimbal_patrol_handle();
                gimbal_ctrl_loop_cal();
                break;
            case GIMBAL_AUTO:
                gimbal_auto_handle();
                gimbal_ctrl_loop_cal();
        }

        launcher_control();//�����������

        CAN_cmd_motor(CAN_1,
                      CAN_MOTOR_0x1FF_ID,
                      gimbal.yaw.give_current,
                      0,
                      0,
                      0);

        CAN_cmd_motor(CAN_2,
                      CAN_MOTOR_0x1FF_ID,
                      launcher.fire_l.give_current,
                      gimbal.pitch.give_current,
                      launcher.fire_r.give_current,
                      launcher.trigger.give_current);


        vTaskDelay(1);
    }
}


static void gimbal_init(){
    debug=0.004;
    gimbal.yaw.motor_measure=&motor_yaw_measure;
    gimbal.pitch.motor_measure=&motor_pitch_measure;

    gimbal.mode=gimbal.last_mode=GIMBAL_RELAX;//��ʼ��Ĭ��״̬Ϊʧ��
//    gimbal.Eulr=&Eulr;    //ŷ����ָ���ȡ

    //yaw���� �ǶȻ����ٶȻ�PID��ʼ��
    pid_init(&gimbal.yaw.angle_p,
             GIMBAL_YAW_ANGLE_MAX_OUT,
             GIMBAL_YAW_ANGLE_MAX_IOUT,
             GIMBAL_YAW_ANGLE_PID_KP,
             GIMBAL_YAW_ANGLE_PID_KI,
             GIMBAL_YAW_ANGLE_PID_KD);

    pid_init(&gimbal.yaw.speed_p,
             GIMBAL_YAW_SPEED_MAX_OUT,
             GIMBAL_YAW_SPEED_MAX_IOUT,
             GIMBAL_YAW_SPEED_PID_KP,
             GIMBAL_YAW_SPEED_PID_KI,
             GIMBAL_YAW_SPEED_PID_KD);

    //pit���� �ǶȻ����ٶȻ�PID��ʼ��
    pid_init(&gimbal.pitch.angle_p,
              GIMBAL_PITCH_ANGLE_MAX_OUT,
              GIMBAL_PITCH_ANGLE_MAX_IOUT,
              GIMBAL_PITCH_ANGLE_PID_KP,
              GIMBAL_PITCH_ANGLE_PID_KI,
              GIMBAL_PITCH_ANGLE_PID_KD);

    pid_init(&gimbal.pitch.speed_p,
             GIMBAL_PITCH_SPEED_MAX_OUT,
             GIMBAL_PITCH_SPEED_MAX_IOUT,
             GIMBAL_PITCH_SPEED_PID_KP,
             GIMBAL_PITCH_SPEED_PID_KI,
             GIMBAL_PITCH_SPEED_PID_KD);

    //��ͨ�˲���ʼ��
    first_order_filter_init(&pitch_first_order_set, 1, 30);
    first_order_filter_init(&pitch_current_first_order_set, 2, 40);
    first_order_filter_init(&pitch_speed_in, 1, 30);
    first_order_filter_init(&yaw_first_order_set, 5, 30);
    first_order_filter_init(&yaw_speed, 1, 45);
    first_order_filter_init(&yaw_speed_in, 1, 40);
    first_order_filter_init(&yaw_angle,1,40);
    first_order_filter_init(&yaw_angle_in,1,40);
    first_order_filter_init(&yaw_current_first_order_set,1,0);

    //��ʼ��ʱ ��̨��Ϊδ����״̬
    gimbal.yaw_is_back=0;
    gimbal.pitch_is_back=0;
    //�ϵ�ʱĬ�������ó�ʧ��ģʽ�����л�����ǰң������ģʽ
    gimbal.last_mode=GIMBAL_RELAX;

    //yaw���pitch������У׼����ֵ
    gimbal.yaw.motor_measure->offset_ecd=3800;
    gimbal.pitch.motor_measure->offset_ecd=6827;

    //������̨Ѳ��ʱpitch���ʼ״̬
    gimbal_up_flag=1;
    gimbal_down_flag=0;
    gimbal_left_flag=1;
    gimbal_right_flag=0;
}

//��̨ģʽ���ã���ȡң������Ϣ���ж�ģʽ��
static void gimbal_mode_set(){

    switch (rc_ctrl.rc.s[RC_s_R]) {

        case RC_SW_DOWN:
        {
            gimbal.mode=GIMBAL_RELAX;
            gimbal.last_mode=gimbal.mode;
            break;
        }

        case RC_SW_MID:
            gimbal.last_mode=gimbal.mode;
            if(gimbal.last_mode==GIMBAL_RELAX || gimbal.last_mode==GIMBAL_PATROL)
            {
                gimbal.mode=GIMBAL_BACK;
                gimbal.yaw_is_back=0;
                gimbal.pitch_is_back=0;
            }
            else if(gimbal.mode==GIMBAL_BACK
                //&&gimbal.pitch_is_back==1
                //&&gimbal.yaw_is_back==1
                    )
            {
                gimbal.mode=GIMBAL_ACTIVE;
            }
            break;
        case RC_SW_UP:
        {
            if(gimbal.yaw_is_back==1&&gimbal.pitch_is_back==1)
            {
                gimbal.mode=GIMBAL_PATROL;
            }
            else{
                gimbal.mode=GIMBAL_BACK;
            }
        }

        default:{

            break;
        }

    }

    switch (gimbal.mode) {

        case GIMBAL_ACTIVE:
        {   //�����ж�
            if(Vision_info.frame_header.cmd==0x31
               &&(detect_list[DETECT_AUTO_AIM].status==ONLINE))
            {
                gimbal.last_mode=GIMBAL_ACTIVE;
                gimbal.mode=GIMBAL_AUTO;
            }

            Vision_send.frame_header.cmd=0x21;

        }   break;

        case GIMBAL_AUTO:
        {   //����ʧЧ�ж�                                                //0x31��ʾ����������Ч
            if ((detect_list[DETECT_AUTO_AIM].status==OFFLINE)||
                Vision_info.frame_header.cmd != 0x31)
            {
                gimbal.last_mode = GIMBAL_AUTO;
                gimbal.mode = GIMBAL_ACTIVE;
            }else{
                Vision_send.frame_header.cmd=0x21;//���͸����������֡֡ͷcmdΪ0x21
            }
        }   break;




    }

    if(detect_list[DETECT_REMOTE].status==OFFLINE)
    {
        gimbal.mode=GIMBAL_RELAX;
    }

}

//���д��������ж���̨�Ƿ���У�����ʶ����1��
static void gimbal_back_handle(){

    if(gimbal.yaw.relative_angle_get>-10&&gimbal.yaw.relative_angle_get<10)
    {
        gimbal.yaw_is_back=1;
    }
    else {
        gimbal.yaw.relative_angle_set=0;
    }

    if(gimbal.pitch.relative_angle_get>-10&&gimbal.pitch.relative_angle_get<10)
    {
        gimbal.pitch_is_back=1;
    }
    else {
        //��ͨ����Խ�0�Ȼ��У��ٽ����Խ���Ϊ0��׼�����п���
        gimbal.pitch.relative_angle_set=0;

    }

    //��û�л���ʱ��ÿ��ִ�к��������бջ����ƻ���
    gimbal.pitch.gyro_set= pid_calc(&gimbal.pitch.angle_p,
                                    gimbal.pitch.relative_angle_get,
                                    gimbal.pitch.relative_angle_set);
    gimbal.pitch.give_current= pid_calc(&gimbal.pitch.speed_p,
                                        gimbal.pitch.motor_measure->speed_rpm,
                                        gimbal.pitch.gyro_set);//�Ӹ���Ϊ�˵����ת

    first_order_filter_cali(&pitch_first_order_set,gimbal.pitch.give_current);
    gimbal.pitch.give_current=pitch_first_order_set.out;

    gimbal.yaw.gyro_set= pid_loop_calc(&gimbal.yaw.angle_p,

                                       gimbal.yaw.relative_angle_get,
                                       gimbal.yaw.relative_angle_set,
                                       180,
                                       -180);

    gimbal.yaw.give_current= pid_calc(&gimbal.yaw.speed_p,
                                      gimbal.yaw.motor_measure->speed_rpm,
                                      gimbal.yaw.gyro_set);

}
//ʹ��ģʽ
static void gimbal_active_handle(){
    //��yaw����ֵ�ϰ�ң������������
    if(Vision_info.isActive!=0)
    {
        gimbal.yaw.relative_angle_set+=Vision_info.yaw.value*debug;
        gimbal.pitch.relative_angle_set-=Vision_info.pitch.value*debug;
    }
    else {
        gimbal.yaw.relative_angle_set+=rc_ctrl.rc.ch[YAW_CHANNEL]*RC_TO_YAW*GIMBAL_RC_MOVE_RATIO_YAW;
        gimbal.pitch.relative_angle_set+=rc_ctrl.rc.ch[PITCH_CHANNEL]*RC_TO_PITCH*GIMBAL_RC_MOVE_RATIO_PIT;
    }

//    if(Vision_info.isActive!=0)
//        gimbal.yaw.relative_angle_set+=Vision_info.yaw.value*debug;


    //��̨��Ȧʱ���о��Խ�ѭ������
//    if(gimbal.yaw.relative_angle_set>=180){
//        gimbal.yaw.relative_angle_set-=360;
//    }
//    else if(gimbal.yaw.relative_angle_set<=-180){
//        gimbal.yaw.relative_angle_set+=360;
//    }
//    if(gimbal.mode==GIMBAL_ACTIVE)
    //��pit����ֵ�ϰ�ң������������
//    else if(gimbal.mode==GIMBAL_AUTO)
//        gimbal.pitch.relative_angle_set-=Vision_info.pitch.value*debug;

    //��pit����ֵ���ж�̬�޷�����λ�������䣩
    gimbal.pitch.relative_angle_set=fp32_constrain(gimbal.pitch.relative_angle_set, MIN_PITCH_RELA_ANGLE, MAX_PITCH_RELA_ANGLE);
}

fp32 radio_y=0.009f;
fp32 radio_p=0.01f;
static void gimbal_auto_handle(){
//TODO:��ֵ�˲�

    gimbal.yaw.absolute_angle_set+=Vision_info.yaw.value*radio_y;
    gimbal.pitch.absolute_angle_set+=Vision_info.pitch.value*radio_p;

    //��̨��Ȧʱ���о��Խ�ѭ������
//    if(gimbal.yaw.absolute_angle_set>=180){
//        gimbal.yaw.absolute_angle_set-=360;
//    }
//    else if(gimbal.yaw.absolute_angle_set<=-180){
//        gimbal.yaw.absolute_angle_set+=360;
//    }

    //��pit����ֵ���ж�̬�޷���ͨ�������Ǻͱ������õ���̬����λ��
    gimbal.pitch.absolute_angle_set=fp32_constrain(gimbal.pitch.absolute_angle_set,
                                                   MIN_ABS_ANGLE+
                                                   gimbal.pitch.absolute_angle_get-gimbal.pitch.relative_angle_get,
                                                   MAX_ABS_ANGLE+
                                                   gimbal.pitch.absolute_angle_get-gimbal.pitch.relative_angle_get);
}
//ʧ��ģʽ�����������Ϊ0��
static void gimbal_relax_handle(){
    gimbal.yaw.give_current=0;
    gimbal.pitch.give_current=0;
    //TODO: ����������� gimbal.trigger.give_current=0;
}

static void gimbal_patrol_handle(){
    //yawѲ��
    if(gimbal_left_flag==1) {
        gimbal.yaw.relative_angle_set += GIMBAL_YAW_PATROL_SPEED;
        if (gimbal.yaw.relative_angle_get >= MAX_YAW_RELA_ANGLE)
        {
            gimbal_left_flag = 0;
            gimbal_right_flag = 1;
        }
    }
    else if(gimbal_right_flag==1){
        gimbal.yaw.relative_angle_set -= GIMBAL_YAW_PATROL_SPEED;
        if(gimbal.yaw.relative_angle_get <= MIN_YAW_RELA_ANGLE)
        {
            gimbal_left_flag = 1;
            gimbal_right_flag = 0;
        }
    }

    //pitѲ��
    if(gimbal_up_flag==1){
        gimbal.pitch.relative_angle_set+=GIMBAL_PITCH_PATROL_SPEED;
        if(gimbal.pitch.relative_angle_get >= MAX_PITCH_RELA_ANGLE){
            gimbal_up_flag=0;
            gimbal_down_flag=1;
        }
    }
    else if(gimbal_down_flag==1){
        gimbal.pitch.relative_angle_set-=GIMBAL_PITCH_PATROL_SPEED;
        if(gimbal.pitch.relative_angle_get <= MIN_PITCH_RELA_ANGLE){
            gimbal_up_flag=1;
            gimbal_down_flag=0;
        }
    }
    //�������ж�
}

//��̨����ջ����ƺ���
static void gimbal_ctrl_loop_cal(){

    first_order_filter_cali(&yaw_angle_in,gimbal.yaw.relative_angle_set);
    first_order_filter_cali(&yaw_angle,gimbal.yaw.relative_angle_get);
    gimbal.yaw.gyro_set= pid_loop_calc(&gimbal.yaw.angle_p,
                                       yaw_angle.out,
                                  yaw_angle_in.out,
                                  180,
                                  -180);

    first_order_filter_cali(&yaw_speed,gimbal.yaw.motor_measure->speed_rpm);
    first_order_filter_cali(&yaw_speed_in,gimbal.yaw.gyro_set);

    gimbal.yaw.give_current= -pid_calc(&gimbal.yaw.speed_p,
                                       yaw_speed.out,
                                       yaw_speed_in.out);

//    first_order_filter_cali(&yaw_current_first_order_set,gimbal.yaw.give_current);
//    gimbal.yaw.give_current=yaw_current_first_order_set.out;
    //�����ٶȻ���
//    first_order_filter_cali(&yaw_speed_in,rc_ctrl.rc.ch[YAW_CHANNEL]*0.2f);
//
//    gimbal.yaw.give_current= pid_calc(&gimbal.yaw.speed_p,
//                                      yaw_speed.out,
//                                      yaw_speed_in.out);

    gimbal.pitch.gyro_set= pid_calc(&gimbal.pitch.angle_p,
                                    gimbal.pitch.relative_angle_get,
                                  gimbal.pitch.relative_angle_set);

    //first_order_filter_cali(&pitch_first_order_set,gimbal.pitch.gyro_set);
    first_order_filter_cali(&pitch_speed_in,gimbal.pitch.gyro_set);
    first_order_filter_cali(&pitch_current_first_order_set,gimbal.pitch.motor_measure->speed_rpm);

    gimbal.pitch.give_current= pid_calc(&gimbal.pitch.speed_p,
                                         pitch_current_first_order_set.out,
                                         pitch_speed_in.out);//�Ӹ���Ϊ�˵����ת

//    first_order_filter_cali(&pitch_current_first_order_set,gimbal.pitch.give_current);
//    gimbal.pitch.give_current=pitch_current_first_order_set.out;
//�����ٶȻ���
//    first_order_filter_cali(&pitch_speed_in,-rc_ctrl.rc.ch[PITCH_CHANNEL]*0.2f);

//    gimbal.pitch.give_current= pid_calc(&gimbal.pitch.speed_p,
//                                        pitch_current_first_order_set.out,
//                                        pitch_speed_in.out);

    first_order_filter_cali(&pitch_first_order_set,gimbal.pitch.give_current);
    gimbal.pitch.give_current=pitch_first_order_set.out;
    //TODO: trigger pid
}



//��̨�Ƕȸ���
static void gimbal_angle_update(){
//    fp32 pitch_absolute_angle_get=INS_angle[2]*MOTOR_RAD_TO_ANGLE;
    gimbal.pitch.absolute_angle_get=INS_angle[2]*MOTOR_RAD_TO_ANGLE;
    gimbal.pitch.relative_angle_get= motor_ecd_to_angle_change(gimbal.pitch.motor_measure->ecd,
                                                                gimbal.pitch.motor_measure->offset_ecd);


//    fp32 yaw_absolute_angle_get=INS_angle[0]*MOTOR_RAD_TO_ANGLE;
    gimbal.yaw.absolute_angle_get=INS_angle[0]*MOTOR_RAD_TO_ANGLE;
    gimbal.yaw.relative_angle_get= -motor_ecd_to_angle_change(gimbal.yaw.motor_measure->ecd,
                                                              gimbal.yaw.motor_measure->offset_ecd);

}