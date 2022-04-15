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

/*      变量      */
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

/*      函数及声明   */
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
    //任务初始化时间
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    //云台初始化
    gimbal_init();

    //发射机构初始化
    launcher_init();

    //TODO:判断电机是否上线

    //主任务体
    while(1){

        gimbal_angle_update();//更新绝对、相对角度接收值

        gimbal_mode_set();//根据遥控器设置云台控制模式

        launcher_mode_set();//发射模式设置

        switch (gimbal.mode) {

            case GIMBAL_RELAX://云台失能
                gimbal_relax_handle();
                break;

            case GIMBAL_BACK://云台回中
                gimbal_back_handle();
//                gimbal_ctrl_loop_cal();
                break;

            case GIMBAL_ACTIVE://云台控制
                gimbal_active_handle();  //得到遥控器对云台电机的控制
                gimbal_ctrl_loop_cal();
                break;

            case GIMBAL_PATROL://云台巡逻
                gimbal_patrol_handle();
                gimbal_ctrl_loop_cal();
                break;
            case GIMBAL_AUTO:
                gimbal_auto_handle();
                gimbal_ctrl_loop_cal();
        }

        launcher_control();//发射机构控制

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

    gimbal.mode=gimbal.last_mode=GIMBAL_RELAX;//初始化默认状态为失能
//    gimbal.Eulr=&Eulr;    //欧拉角指针获取

    //yaw轴电机 角度环和速度环PID初始化
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

    //pit轴电机 角度环和速度环PID初始化
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

    //低通滤波初始化
    first_order_filter_init(&pitch_first_order_set, 1, 30);
    first_order_filter_init(&pitch_current_first_order_set, 2, 40);
    first_order_filter_init(&pitch_speed_in, 1, 30);
    first_order_filter_init(&yaw_first_order_set, 5, 30);
    first_order_filter_init(&yaw_speed, 1, 45);
    first_order_filter_init(&yaw_speed_in, 1, 40);
    first_order_filter_init(&yaw_angle,1,40);
    first_order_filter_init(&yaw_angle_in,1,40);
    first_order_filter_init(&yaw_current_first_order_set,1,0);

    //初始化时 云台设为未回中状态
    gimbal.yaw_is_back=0;
    gimbal.pitch_is_back=0;
    //上电时默认先设置成失能模式，再切换到当前遥控设置模式
    gimbal.last_mode=GIMBAL_RELAX;

    //yaw轴和pitch轴电机的校准编码值
    gimbal.yaw.motor_measure->offset_ecd=3800;
    gimbal.pitch.motor_measure->offset_ecd=6827;

    //设置云台巡逻时pitch轴初始状态
    gimbal_up_flag=1;
    gimbal_down_flag=0;
    gimbal_left_flag=1;
    gimbal_right_flag=0;
}

//云台模式设置（获取遥控器信息，判断模式）
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
        {   //自瞄判定
            if(Vision_info.frame_header.cmd==0x31
               &&(detect_list[DETECT_AUTO_AIM].status==ONLINE))
            {
                gimbal.last_mode=GIMBAL_ACTIVE;
                gimbal.mode=GIMBAL_AUTO;
            }

            Vision_send.frame_header.cmd=0x21;

        }   break;

        case GIMBAL_AUTO:
        {   //自瞄失效判定                                                //0x31表示自瞄数据有效
            if ((detect_list[DETECT_AUTO_AIM].status==OFFLINE)||
                Vision_info.frame_header.cmd != 0x31)
            {
                gimbal.last_mode = GIMBAL_AUTO;
                gimbal.mode = GIMBAL_ACTIVE;
            }else{
                Vision_send.frame_header.cmd=0x21;//发送给自瞄的数据帧帧头cmd为0x21
            }
        }   break;




    }

    if(detect_list[DETECT_REMOTE].status==OFFLINE)
    {
        gimbal.mode=GIMBAL_RELAX;
    }

}

//回中处理函数（判断云台是否回中，将标识符置1）
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
        //先通过相对角0度回中，再将绝对角设为0度准备进行控制
        gimbal.pitch.relative_angle_set=0;

    }

    //在没有回中时，每次执行函数都进行闭环控制回中
    gimbal.pitch.gyro_set= pid_calc(&gimbal.pitch.angle_p,
                                    gimbal.pitch.relative_angle_get,
                                    gimbal.pitch.relative_angle_set);
    gimbal.pitch.give_current= pid_calc(&gimbal.pitch.speed_p,
                                        gimbal.pitch.motor_measure->speed_rpm,
                                        gimbal.pitch.gyro_set);//加负号为了电机反转

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
//使能模式
static void gimbal_active_handle(){
    //在yaw期望值上按遥控器进行增减
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


    //云台绕圈时进行绝对角循环设置
//    if(gimbal.yaw.relative_angle_set>=180){
//        gimbal.yaw.relative_angle_set-=360;
//    }
//    else if(gimbal.yaw.relative_angle_set<=-180){
//        gimbal.yaw.relative_angle_set+=360;
//    }
//    if(gimbal.mode==GIMBAL_ACTIVE)
    //在pit期望值上按遥控器进行增减
//    else if(gimbal.mode==GIMBAL_AUTO)
//        gimbal.pitch.relative_angle_set-=Vision_info.pitch.value*debug;

    //对pit期望值进行动态限幅（限位，待补充）
    gimbal.pitch.relative_angle_set=fp32_constrain(gimbal.pitch.relative_angle_set, MIN_PITCH_RELA_ANGLE, MAX_PITCH_RELA_ANGLE);
}

fp32 radio_y=0.009f;
fp32 radio_p=0.01f;
static void gimbal_auto_handle(){
//TODO:均值滤波

    gimbal.yaw.absolute_angle_set+=Vision_info.yaw.value*radio_y;
    gimbal.pitch.absolute_angle_set+=Vision_info.pitch.value*radio_p;

    //云台绕圈时进行绝对角循环设置
//    if(gimbal.yaw.absolute_angle_set>=180){
//        gimbal.yaw.absolute_angle_set-=360;
//    }
//    else if(gimbal.yaw.absolute_angle_set<=-180){
//        gimbal.yaw.absolute_angle_set+=360;
//    }

    //对pit期望值进行动态限幅（通过陀螺仪和编码器得到动态的限位）
    gimbal.pitch.absolute_angle_set=fp32_constrain(gimbal.pitch.absolute_angle_set,
                                                   MIN_ABS_ANGLE+
                                                   gimbal.pitch.absolute_angle_get-gimbal.pitch.relative_angle_get,
                                                   MAX_ABS_ANGLE+
                                                   gimbal.pitch.absolute_angle_get-gimbal.pitch.relative_angle_get);
}
//失能模式处理（两轴电流为0）
static void gimbal_relax_handle(){
    gimbal.yaw.give_current=0;
    gimbal.pitch.give_current=0;
    //TODO: 拨弹电机电流 gimbal.trigger.give_current=0;
}

static void gimbal_patrol_handle(){
    //yaw巡逻
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

    //pit巡逻
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
    //加自瞄判断
}

//云台电机闭环控制函数
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
    //调试速度环用
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
                                         pitch_speed_in.out);//加负号为了电机反转

//    first_order_filter_cali(&pitch_current_first_order_set,gimbal.pitch.give_current);
//    gimbal.pitch.give_current=pitch_current_first_order_set.out;
//调试速度环用
//    first_order_filter_cali(&pitch_speed_in,-rc_ctrl.rc.ch[PITCH_CHANNEL]*0.2f);

//    gimbal.pitch.give_current= pid_calc(&gimbal.pitch.speed_p,
//                                        pitch_current_first_order_set.out,
//                                        pitch_speed_in.out);

    first_order_filter_cali(&pitch_first_order_set,gimbal.pitch.give_current);
    gimbal.pitch.give_current=pitch_first_order_set.out;
    //TODO: trigger pid
}



//云台角度更新
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