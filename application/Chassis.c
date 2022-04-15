//
// Created by xhuanc on 2021/10/10.
//

/*include*/
#include <stdlib.h>
#include "Chassis.h"
#include "cmsis_os.h"
#include "remote.h"
#include "can_receive.h"
#include "user_lib.h"
#include "steel_ctrl.h"
#include "ramp.h"
#include "key_board.h"
#include "balance_ctrl.h"
#include "Gimbal.h"
#include"bsp_buzzer.h"
#include "math.h"
#include "bsp_led.h"
#include "Referee.h"
#include "bsp_servo_pwm.h"
#include "Detection.h"
/*define*/
     /*轮子电机id：                                    舵：                                    解算坐标：      x(前)
                 ****      前      ****                     ****               ****                              |
                *  2 *            * 1  *                   * LF *     前      * RF *                             |
                 ****              ****                     ****               ****                              |
                                                                                                                 |
                左                   右                                                             --------------z-----------y(右)
                                                                                                                 |
                 ****              ****                     ****               ****                              |
                *  3 *            * 4  *                   * LB *             * RB *                             |
                 ****      后      ****                     ****               ****                              |

     */

/*变量*/
extern RC_ctrl_t rc_ctrl;
extern motor_measure_t motor_left_measure;
extern motor_measure_t motor_right_measure;
extern Referee_info_t Referee;
ramp_function_source_t chassis_vx_ramp;
ramp_function_source_t chassis_vy_ramp;
ramp_function_source_t chassis_3508_ramp[4];
chassis_t chassis;
extern gimbal_t gimbal;
static fp32 rotate_ratio_f= ((WHEELBASE+WHEELTRACK)/2.0f-GIMBAL_OFFSET)/RADIAN_COEF;
static fp32 rotate_ratio_b= ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_OFFSET) / RADIAN_COEF;
static fp32 wheel_rpm_ratio = 60.0f / (PERIMETER * M3508_DECELE_RATIO);
int32_t starting_point;
int32_t end_point;
uint8_t chassis_go_left_flag;
uint8_t chassis_go_right_flag;
//#ifdef DEBUG_PID
//pid_t steer_angle;
//pid_t steer_speed;
//#endif

/*函数 & 声明*/

static void chassis_init(chassis_t *chassis);

static void chassis_set_mode(chassis_t *chassis);

static void chassis_ctrl_info_get();

static void chassis_relax_handle();

static void chassis_auto_handle();

static void chassis_wheel_cal(fp32 vy);

static void chassis_wheel_loop_cal();

static void chassis_steer_loop_cal();

//static void motor_angle_get(motor_6020_t* motor);

static void chassis_motor_relative_angle_cal();

static float chassis_auto_speed_cla();

static void turn_round_detect();

static void chassis_test();

void chassis_device_offline_handle();

void HP_check();

void chassis_task(void const *pvParameters){

    vTaskDelay(CHASSIS_TASK_INIT_TIME);

    chassis_init(&chassis);//底盘初始化

    //TODO:判断底盘电机是否都在线

    //主任务循环
    while (1){

        //遥控器获取底盘方向矢量
        chassis_ctrl_info_get();

        //更新PC的控制信息
        update_pc_info();

        chassis_set_mode(&chassis);

#if isSteer
        //计算当前底盘6020的相对角度
        chassis_motor_relative_angle_cal();
#endif

        //判断底盘模式选择 决定是否覆盖底盘转速vw;
        switch (chassis.mode) {

            case CHASSIS_AUTO://自动巡逻
                chassis_auto_handle();
                break;
            case CHASSIS_STANDBY:
                //有新想法可以补充
                break;
            case CHASSIS_STARTING_POINT_CAL:
            {
                starting_point=chassis.motor_chassis[LF].motor_measure->total_dis;
                chassis_start_buzzer;
                break;
            }
            case CHASSIS_END_POINT_CAL:
            {
                end_point=chassis.motor_chassis[LF].motor_measure->total_dis;
                chassis_buzzer_off;
                break;
            }
            case CHASSIS_TEST:
            {
                chassis_test();

                break;
            }

        }

        chassis_device_offline_handle();

        if(chassis.mode!=CHASSIS_RELAX)
        {


            //底盘解算
            chassis_wheel_cal(chassis.vy);

//            驱电机闭环
            chassis_wheel_loop_cal();


#if isSteer
            CAN_cmd_motor( CAN_1,
                           CAN_MOTOR_0x1FF_ID,
                           chassis.motor_steer[RF].give_current,
                           chassis.motor_steer[LF].give_current,
                           chassis.motor_steer[LB].give_current,
                           chassis.motor_steer[RB].give_current);

            CAN_cmd_motor(CAN_2,
                          CAN_MOTOR_0x200_ID,
                          chassis.motor_chassis[RF].give_current,
                          chassis.motor_chassis[LF].give_current,
                          chassis.motor_chassis[LB].give_current,
                          chassis.motor_chassis[RB].give_current);
#endif

#if isMekNum
            CAN_cmd_motor(CAN_1,
                          CAN_MOTOR_0x200_ID,
                          0,
                          chassis.motor_chassis[LF].give_current,
                          0,
                          0);
#endif

        }
        else if(chassis.mode==CHASSIS_RELAX)
        {
            chassis_relax_handle();
        }

        vTaskDelay(2);
    }

}

static void chassis_ctrl_info_get(){
    //TODO:未加上键鼠
    //vx是前进后退方向
    chassis.vx=(rc_ctrl.rc.ch[CHASSIS_X_CHANNEL])*RC_TO_VX;
//    ramp_calc(&chassis_vx_ramp,chassis.vx);
//    chassis.vx=chassis_vx_ramp.out;

    chassis.vy=(rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL])*RC_TO_VY;
//    ramp_calc(&chassis_vy_ramp,chassis.vy);
//    chassis.vy=chassis_vy_ramp.out;

    chassis.vw=(rc_ctrl.rc.ch[CHASSIS_Z_CHANNEL])*RC_TO_VW;

    //TODO:斜坡

//    rc_deadband_limit(rc_ctrl.rc.ch[CHASSIS_X_CHANNEL],chassis.vx,10);
//    rc_deadband_limit(rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL],chassis.vy,10);
//    rc_deadband_limit(rc_ctrl.rc.ch[CHASSIS_Z_CHANNEL],chassis.vw,10);

}

static void chassis_init(chassis_t *chassis) {


    if (chassis == NULL)
        return;

    uint8_t i = 0;

    //底盘跟随云台初始化
    pid_init(&chassis->chassis_vw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT,
             CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD);

    //底盘四个驱动电机速度环初始化和电机数据结构体获取
    //底盘四个舵向电机角度环和速度环 电机数据结构体获取
    for (i = 0; i < 4; i++)
    {

        chassis->motor_chassis[i].motor_measure=motor_3508_measure+i;
#if isSteer
        chassis->motor_steer[i].motor_measure=motor_6020_measure+i;
#endif

        pid_init(&chassis->motor_chassis[i].speed_p,
                 CHASSIS_3508_PID_MAX_OUT,
                 CHASSIS_3508_PID_MAX_IOUT,
                 CHASSIS_3508_PID_KP,
                 CHASSIS_3508_PID_KI,
                 CHASSIS_3508_PID_KD);
    }

#if isSteer
//舵轮PID 初始化 RF->LF->LB->RB    角度环->速度环
    pid_init(&(chassis->motor_steer[RF].angle_p),
             CHASSIS_6020_RF_PID_MAX_OUT_ANGLE,
             CHASSIS_6020_RF_PID_MAX_IOUT_ANGLE,
             CHASSIS_6020_RF_PID_KP_ANGLE,
             CHASSIS_6020_RF_PID_KI_ANGLE,
             CHASSIS_6020_RF_PID_KD_ANGLE);

    pid_init(&(chassis->motor_steer[RF].speed_p),
             CHASSIS_6020_RF_PID_MAX_OUT_SPEED,
             CHASSIS_6020_RF_PID_MAX_IOUT_SPEED,
             CHASSIS_6020_RF_PID_KP_SPEED,
             CHASSIS_6020_RF_PID_KI_SPEED,
             CHASSIS_6020_RF_PID_KD_SPEED);

    pid_init(&(chassis->motor_steer[LF].angle_p),
             CHASSIS_6020_LF_PID_MAX_OUT_ANGLE,
             CHASSIS_6020_LF_PID_MAX_IOUT_ANGLE,
             CHASSIS_6020_LF_PID_KP_ANGLE,
             CHASSIS_6020_LF_PID_KI_ANGLE,
             CHASSIS_6020_LF_PID_KD_ANGLE);

    pid_init(&(chassis->motor_steer[LF].speed_p),
             CHASSIS_6020_LF_PID_MAX_OUT_SPEED,
             CHASSIS_6020_LF_PID_MAX_IOUT_SPEED,
             CHASSIS_6020_LF_PID_KP_SPEED,
             CHASSIS_6020_LF_PID_KI_SPEED,
             CHASSIS_6020_LF_PID_KD_SPEED);

    pid_init(&(chassis->motor_steer[LB].angle_p),
             CHASSIS_6020_LB_PID_MAX_OUT_ANGLE,
             CHASSIS_6020_LB_PID_MAX_IOUT_ANGLE,
             CHASSIS_6020_LB_PID_KP_ANGLE,
             CHASSIS_6020_LB_PID_KI_ANGLE,
             CHASSIS_6020_LB_PID_KD_ANGLE);

    pid_init(&(chassis->motor_steer[LB].speed_p),
             CHASSIS_6020_LB_PID_MAX_OUT_SPEED,
             CHASSIS_6020_LB_PID_MAX_IOUT_SPEED,
             CHASSIS_6020_LB_PID_KP_SPEED,
             CHASSIS_6020_LB_PID_KI_SPEED,
             CHASSIS_6020_LB_PID_KD_SPEED);

    pid_init(&(chassis->motor_steer[RB].angle_p),
             CHASSIS_6020_RB_PID_MAX_OUT_ANGLE,
             CHASSIS_6020_RB_PID_MAX_IOUT_ANGLE,
             CHASSIS_6020_RB_PID_KP_ANGLE,
             CHASSIS_6020_RB_PID_KI_ANGLE,
             CHASSIS_6020_RB_PID_KD_ANGLE);

    pid_init(&(chassis->motor_steer[RB].speed_p),
             CHASSIS_6020_RB_PID_MAX_OUT_SPEED,
             CHASSIS_6020_RB_PID_MAX_IOUT_SPEED,
             CHASSIS_6020_RB_PID_KP_SPEED,
             CHASSIS_6020_RB_PID_KI_SPEED,
             CHASSIS_6020_RB_PID_KD_SPEED);
#endif


#if isBalance
    chassis->motor_balance_wheel[0].motor_measure=&motor_left_measure;
    chassis->motor_balance_wheel[1].motor_measure=&motor_right_measure;

    pid_init(&chassis->motor_balance_wheel[0].speed_p,
             CHASSIS_2006_LW_PID_MAX_OUT_SPEED,
             CHASSIS_2006_LW_PID_MAX_IOUT_SPEED,
             CHASSIS_2006_LW_PID_KP_SPEED,
             CHASSIS_2006_LW_PID_KI_SPEED,
             CHASSIS_2006_LW_PID_KD_SPEED);

    pid_init(&chassis->motor_balance_wheel[1].speed_p,
             CHASSIS_2006_RW_PID_MAX_OUT_SPEED,
             CHASSIS_2006_RW_PID_MAX_IOUT_SPEED,
             CHASSIS_2006_RW_PID_KP_SPEED,
             CHASSIS_2006_RW_PID_KI_SPEED,
             CHASSIS_2006_RW_PID_KD_SPEED);

    pid_init(&chassis->Vertical_pid,
             CHASSIS_VERTICAL_PID_MAX_OUT,
             CHASSIS_VERTICAL_PID_MAX_IOUT,
             CHASSIS_VERTICAL_PID_KP,
             CHASSIS_VERTICAL_PID_KI,
             CHASSIS_VERTICAL_PID_KD);
#endif

    //初始时底盘模式为失能
    chassis->mode=chassis->last_mode=CHASSIS_RELAX;




    ramp_init(&chassis_vx_ramp,0.0001,MAX_CHASSIS_VX_SPEED,-MAX_CHASSIS_VX_SPEED);
    ramp_init(&chassis_vy_ramp,0.0001,MAX_CHASSIS_VX_SPEED,-MAX_CHASSIS_VX_SPEED);
    ramp_init(&chassis_3508_ramp[LF],0.0001,M3508_MAX_RPM,-M3508_MAX_RPM);
    ramp_init(&chassis_3508_ramp[RF],0.0001,M3508_MAX_RPM,-M3508_MAX_RPM);
    ramp_init(&chassis_3508_ramp[RB],0.0001,M3508_MAX_RPM,-M3508_MAX_RPM);
    ramp_init(&chassis_3508_ramp[LB],0.0001,M3508_MAX_RPM,-M3508_MAX_RPM);

    chassis_go_left_flag=0;
    chassis_go_right_flag=1;
}

static void chassis_set_mode(chassis_t* chassis){

    if(chassis==NULL)
        return;

    if(switch_is_down(rc_ctrl.rc.s[RC_s_L])&&switch_is_down(rc_ctrl.rc.s[RC_s_R]))
    {
        chassis->last_mode=chassis->mode;
        chassis->mode=CHASSIS_RELAX;
    }
//    else if(switch_is_down(rc_ctrl.rc.s[RC_s_R])&&switch_is_mid(rc_ctrl.rc.s[RC_s_L]))
//    {
//        chassis->last_mode=chassis->mode;
//        chassis->mode=CHASSIS_STARTING_POINT_CAL;
//    }
//    else if(switch_is_down(rc_ctrl.rc.s[RC_s_R])&&switch_is_up(rc_ctrl.rc.s[RC_s_L]))
//    {
//        chassis->last_mode=chassis->mode;
//        chassis->mode=CHASSIS_END_POINT_CAL;
//    }
//    else if(switch_is_down(rc_ctrl.rc.s[RC_s_R])&&switch_is_mid(rc_ctrl.rc.s[RC_s_L]))
//    {
//        chassis->last_mode=chassis->mode;
//        chassis->mode=CHASSIS_TEST;
//    }
    else if(switch_is_up(rc_ctrl.rc.s[RC_s_R]))
    {
        chassis->last_mode=chassis->mode;
        chassis->mode=CHASSIS_AUTO;
    }
//测试时加
    //    TODO:当自瞄时 最好保持底盘静止

}

static void chassis_relax_handle()
{
    CAN_cmd_motor(CAN_1,CAN_MOTOR_0x200_ID,0,0,0,0);

#if isSteer
    CAN_cmd_motor(CAN_2,CAN_MOTOR_0x1FF_ID,0,0,0,0);
#endif

}

static void chassis_test()
{
//
//    chassis.motor_chassis[LF].give_current= pid_calc(&chassis.motor_chassis[LF].speed_p,
//                                                     chassis.motor_chassis[LF].motor_measure->speed_rpm,
//                                                     chassis.vy);
}

void chassis_device_offline_handle(){
//    if(
//       detect_list[DETECT_CHASSIS_3508_LF].status==OFFLINE||
//       detect_list[DETECT_REMOTE].status==OFFLINE||
//       Referee.GameResult.game_over==true)//比赛结束自动停下
//    {
//        chassis.mode=CHASSIS_RELAX;//防止出现底盘疯转
//    }

}

static void chassis_auto_handle(){
    turn_round_detect();
    if(chassis_go_right_flag==1){
        chassis.vy=chassis_auto_speed_cla();
        //chassis.vy=0;
//        if(motor_3508_measure[LF].total_dis>=end_point){
//            chassis_go_right_flag=0;
//            chassis_go_left_flag=1;
//        }
    }
    else if(chassis_go_left_flag==1){
        chassis.vy=-chassis_auto_speed_cla();
        //chassis.vy=0;
//        if(motor_3508_measure[LF].total_dis<=starting_point){
//            chassis_go_left_flag=0;
//            chassis_go_right_flag=1;
//        }
    }
}

#if isMekNum
static void chassis_wheel_cal(fp32 vy){
    fp32 max=0;
    int16_t wheel_rpm[4];

    VAL_LIMIT(vy, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  // mm/s

    wheel_rpm[0] =  vy  * wheel_rpm_ratio;

    // find max item
    for (uint8_t i = 0; i < 4; i++) {
        if (abs(wheel_rpm[i]) > max) max = abs(wheel_rpm[i]);
    }
    // equal proportion
    if (max > M3508_MAX_RPM) {
        float rate = M3508_MAX_RPM / max;
        for (uint8_t i = 0; i < 4; i++) wheel_rpm[i] *= rate;
    }

    chassis.motor_chassis[LF].rpm_set=wheel_rpm[0];
}
#endif

static void chassis_wheel_loop_cal(){


    ramp_calc(&chassis_3508_ramp[LF],chassis.motor_chassis[LF].rpm_set);

    chassis.motor_chassis[LF].give_current= pid_calc(&chassis.motor_chassis[LF].speed_p,
                                                     chassis.motor_chassis[LF].motor_measure->speed_rpm,
                                                     chassis.motor_chassis[LF].rpm_set);


}

static bool hurt_flag=0,hero_hurt=0;
void HP_check()
{
     static uint16_t last_hp=600,
             current_hp=0;
     static int16_t delta_hp;
    static uint32_t hp_check_time=1;

    hp_check_time++;

    if(hp_check_time%100 == 0)
    {
        current_hp = Referee.GameRobotStat.remain_HP;
        delta_hp = current_hp - last_hp;
        last_hp = current_hp;
        if(delta_hp>60)
            hero_hurt = 1;//检测到大弹丸伤害直接全游戏保持高速润跑
        if(delta_hp>=10)
            hurt_flag = 1;//检测到伤害

    }

}
static float chassis_auto_speed_cla(){
    float speed;//调试用
    static uint32_t hurt_times,safe_time;

    turn_round_detect();
    HP_check();
    if(Referee.RobotHurt.being_hurt == true && Referee.RobotHurt.hurt_type == 0x00)
    {
        hurt_flag = 1;
    }
    else
    {
        hurt_flag = 0;
    }
    if(hurt_flag==1)
    {
        safe_time=0;
        hurt_times++;
    }
    else
    {
        safe_time++;
        if(safe_time>6000)
        {
            hurt_times=0;
        }
    }
        speed = 2000;
    if(hurt_times>2||hero_hurt==1)
        speed= 3500*sin(0.006*HAL_GetTick())+4000;

    return speed;
}


//tof加微动开关通过电平变化实现转向检测
static void turn_round_detect()
{
    uint8_t left_end_flag,right_end_flag,error=0;
    left_end_flag = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0);
    right_end_flag = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1);
    if(right_end_flag == 1 && left_end_flag == 1)
    {
        error = 1;
        right_end_flag = 0;
        left_end_flag = 0;
    }
    if(error==0)
    {
        if (left_end_flag == 1)
        {
            chassis_go_right_flag = 1;
            chassis_go_left_flag = 0;
        }

        if (right_end_flag == 1)
        {
            chassis_go_right_flag = 0;
            chassis_go_left_flag = 1;
        }

    }
    else if(error == 1)
    {
        chassis.mode = CHASSIS_RELAX;
    }

}
#if isSteer
static void chassis_steer_loop_cal(){


    chassis.motor_steer[RF].gyro_set= pid_calc(&chassis.motor_steer[RF].angle_p,
    /*角度环*/                                    chassis.motor_steer[RF].relative_angle_get,
                                                 chassis.motor_steer[RF].relative_angle_set);

    chassis.motor_steer[RF].give_current= (int16_t)pid_calc(&chassis.motor_steer[RF].speed_p,
    /*速度环*/                                                chassis.motor_steer[RF].motor_measure->speed_rpm,
                                                             chassis.motor_steer[RF].gyro_set);


    chassis.motor_steer[LF].gyro_set= pid_calc(&chassis.motor_steer[LF].angle_p,
                                                chassis.motor_steer[LF].relative_angle_get,
                                                chassis.motor_steer[LF].relative_angle_set);

    chassis.motor_steer[LF].give_current= (int16_t)pid_calc(&chassis.motor_steer[LF].speed_p,
                                                             chassis.motor_steer[LF].motor_measure->speed_rpm,
                                                             chassis.motor_steer[LF].gyro_set);


    chassis.motor_steer[LB].gyro_set= pid_calc(&chassis.motor_steer[LB].angle_p,
                                                chassis.motor_steer[LB].relative_angle_get,
                                                chassis.motor_steer[LB].relative_angle_set);

    chassis.motor_steer[LB].give_current= (int16_t)pid_calc(&chassis.motor_steer[LB].speed_p,
                                                             chassis.motor_steer[LB].motor_measure->speed_rpm,
                                                             chassis.motor_steer[LB].gyro_set);


    chassis.motor_steer[RB].gyro_set = pid_calc(&chassis.motor_steer[RB].angle_p,
                                                chassis.motor_steer[RB].relative_angle_get,
                                                chassis.motor_steer[RB].relative_angle_set);

    chassis.motor_steer[RB].give_current = (int16_t)pid_calc(&chassis.motor_steer[RB].speed_p,
                                                             chassis.motor_steer[RB].motor_measure->speed_rpm,
                                                             chassis.motor_steer[RB].gyro_set);
}

static void chassis_motor_relative_angle_cal(){

    chassis.motor_steer[RF].relative_angle_get= motor_ecd_to_angle_change(chassis.motor_steer[RF].motor_measure->ecd,chassis.motor_steer[RF].motor_measure->offset_ecd);
    chassis.motor_steer[LF].relative_angle_get= motor_ecd_to_angle_change(chassis.motor_steer[LF].motor_measure->ecd,chassis.motor_steer[LF].motor_measure->offset_ecd);
    chassis.motor_steer[LB].relative_angle_get= motor_ecd_to_angle_change(chassis.motor_steer[LB].motor_measure->ecd,chassis.motor_steer[LB].motor_measure->offset_ecd);
    chassis.motor_steer[RB].relative_angle_get= motor_ecd_to_angle_change(chassis.motor_steer[RB].motor_measure->ecd,chassis.motor_steer[RB].motor_measure->offset_ecd);
}
#endif