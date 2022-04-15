//
// Created by xhuanc on 2021/10/12.
//

#include <math.h>
#include "steel_ctrl.h"
#include "remote.h"
extern RC_ctrl_t rc_ctrl;
extern chassis_t chassis;
void chassis_steering_wheel_cal(){


    if(chassis.vx==0&&chassis.vy==0&&chassis.vw==0)
    {
        chassis.motor_steer[RF].relative_angle_set=0;
        chassis.motor_steer[LF].relative_angle_set=0;
        chassis.motor_steer[LB].relative_angle_set=0;
        chassis.motor_steer[RB].relative_angle_set=0;
        chassis.motor_chassis[RF].rpm_set=
        chassis.motor_chassis[LF].rpm_set=
        chassis.motor_chassis[LB].rpm_set=
        chassis.motor_chassis[RB].rpm_set=0;
    }
    else {
        fp32 alpha,sinAlpha,cosAlpha,speedV;
        VAL_LIMIT(chassis.vy ,-MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  // mm/s
        VAL_LIMIT(chassis.vw, -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED);  // deg/s
        VAL_LIMIT(chassis.vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  // mm/s

        //通过6020电机的ecd 和 ecdoffset 求出当前相对角度


        if(chassis.vy!=0||chassis.vx!=0)
        {
            alpha=atan2(chassis.vy,chassis.vx);
        }
        sinAlpha= sinf(alpha);
        cosAlpha= cosf(alpha);
        speedV=my_sqrt(my_pow(chassis.vy) + my_pow(chassis.vx));

        //LF:
        chassis.motor_chassis[LF].speed= my_sqrt(my_pow(W_Cdistance) * my_pow(chassis.vw) / 2 + my_pow(speedV) + (WHEELBASE* sinAlpha + WHEELTRACK* cosAlpha) * chassis.vw * speedV);
        chassis.motor_steer[LF].relative_angle_set=(PI / 2) - atan2((2 * speedV * cosAlpha + WHEELTRACK * chassis.vw), (2 * speedV * sinAlpha + WHEELBASE * chassis.vw));
        if(chassis.motor_steer[LF].relative_angle_set<0)
        {
            chassis.motor_steer[LF].relative_angle_set+=2*PI;
        }
        chassis.motor_steer[LF].relative_angle_set*=RADIAN_COEF;//0-360

        //LB:   2
        chassis.motor_chassis[LB].speed= my_sqrt(my_pow(W_Cdistance) * my_pow(chassis.vw) / 2 + my_pow(speedV) + (-WHEELBASE* sinAlpha + WHEELTRACK* cosAlpha) * chassis.vw * speedV);
        chassis.motor_steer[LB].relative_angle_set=(PI / 2) - atan2((2 * speedV * cosAlpha + WHEELTRACK * chassis.vw), (2 * speedV * sinAlpha - WHEELBASE * chassis.vw));
        if(chassis.motor_steer[LB].relative_angle_set<0)
        {
            chassis.motor_steer[LB].relative_angle_set+=2*PI;
        }
        chassis.motor_steer[LB].relative_angle_set*=RADIAN_COEF;//0-360

        //RF:   0
        chassis.motor_chassis[RF].speed= my_sqrt(my_pow(W_Cdistance) * my_pow(chassis.vw) / 2 + my_pow(speedV) + (WHEELBASE* sinAlpha - WHEELTRACK* cosAlpha) * chassis.vw * speedV);
        chassis.motor_steer[RF].relative_angle_set= PI/2-atan2((2 * speedV * cosAlpha -WHEELTRACK * chassis.vw), (2 * speedV * sinAlpha + WHEELBASE * chassis.vw));
        if(chassis.motor_steer[RF].relative_angle_set<0)
        {
            chassis.motor_steer[RF].relative_angle_set+=2*PI;
        }
        chassis.motor_steer[RF].relative_angle_set*=RADIAN_COEF;//0-360

        //RB:   3
        chassis.motor_chassis[RB].speed= my_sqrt(my_pow(W_Cdistance) * my_pow(chassis.vw) / 2 + my_pow(speedV) + (-WHEELBASE* sinAlpha - WHEELTRACK* cosAlpha) * chassis.vw * speedV);
        chassis.motor_steer[RB].relative_angle_set=(PI / 2) - atan2((2 * speedV * cosAlpha - WHEELTRACK * chassis.vw), (2 * speedV * sinAlpha - WHEELBASE * chassis.vw));
        if(chassis.motor_steer[RB].relative_angle_set<0)
        {
            chassis.motor_steer[RB].relative_angle_set+=2*PI;
        }
        chassis.motor_steer[RB].relative_angle_set*=RADIAN_COEF;//0-360


        //fdb 和 set 都是 0-360
        if(ABS(chassis.motor_steer[LF].relative_angle_set-chassis.motor_steer[LF].relative_angle_get)>180.f)
        {
            if(chassis.motor_steer[LF].relative_angle_set>chassis.motor_steer[LF].relative_angle_get)
            {
                chassis.motor_steer[LF].relative_angle_set-=360.f;
            }
            else if(chassis.motor_steer[LF].relative_angle_set<chassis.motor_steer[LF].relative_angle_get)
            {
                chassis.motor_steer[LF].relative_angle_set+=360.f;
            }
        }


        if(ABS(chassis.motor_steer[LB].relative_angle_set-chassis.motor_steer[LB].relative_angle_get)>180.f)
        {
            if(chassis.motor_steer[LB].relative_angle_set>chassis.motor_steer[LB].relative_angle_get)
            {
                chassis.motor_steer[LB].relative_angle_set-=360.f;
            }
            else if(chassis.motor_steer[LB].relative_angle_set<chassis.motor_steer[LB].relative_angle_get)
            {
                chassis.motor_steer[LB].relative_angle_set+=360.f;
            }
        }

        if(ABS(chassis.motor_steer[RB].relative_angle_set-chassis.motor_steer[RB].relative_angle_get)>180.f)
        {
            if(chassis.motor_steer[RB].relative_angle_set>chassis.motor_steer[RB].relative_angle_get)
            {
                chassis.motor_steer[RB].relative_angle_set-=360.f;
            }
            else if(chassis.motor_steer[RB].relative_angle_set<chassis.motor_steer[RB].relative_angle_get)
            {
                chassis.motor_steer[RB].relative_angle_set+=360.f;
            }
        }

        if(ABS(chassis.motor_steer[RF].relative_angle_set-chassis.motor_steer[RF].relative_angle_get)>180.f)
        {
            if(chassis.motor_steer[RF].relative_angle_set>chassis.motor_steer[RF].relative_angle_get)
            {
                chassis.motor_steer[RF].relative_angle_set-=360.f;
            }
            else if(chassis.motor_steer[RF].relative_angle_set<chassis.motor_steer[RF].relative_angle_get)
            {
                chassis.motor_steer[RF].relative_angle_set+=360.f;
            }
        }

        //线速度转换为转速
        chassis.motor_chassis[RF].rpm_set=chassis.motor_chassis[RF].speed/PERIMETER_STEEL*60*19.f;
        chassis.motor_chassis[LF].rpm_set=-chassis.motor_chassis[LF].speed/PERIMETER_STEEL*60*19.f;
        chassis.motor_chassis[LB].rpm_set=chassis.motor_chassis[LB].speed/PERIMETER_STEEL*60*19.f;
        chassis.motor_chassis[RB].rpm_set=-chassis.motor_chassis[RB].speed/PERIMETER_STEEL*60*19.f;


    }

}