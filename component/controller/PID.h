/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of?
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.? See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

/**
  *********************** (C) COPYRIGHT 2018 DJI **********************
  * @update
  * @history
  * Version     Date              Author           Modification
  * V1.0.0      January-15-2018   ric.luo
  * @verbatim
  *********************** (C) COPYRIGHT 2018 DJI **********************
  */




#ifndef __pid_H__
#define __pid_H__

//#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
enum
{
    LAST  = 0,
    NOW   = 1,
};

/**
  * @brief     PID �ṹ��
  */
typedef struct
{
    /* p��i��d���� */
    float p;
    float i;
    float d;

    /* Ŀ��ֵ������ֵ�����ֵ */
    float set;
    float get;
    float err[2];

    /* p��i��d������������� */
    float pout;
    float iout;
    float dout;

    /* pid��ʽ������������ */
    float out;

    /* pid����������  */
    uint32_t max_output;

    /* pid����������޷� */
    uint32_t integral_limit;

} pid_t;

/**
  * @brief     PID ��ʼ������
  * @param[in] pid: PID �ṹ��
  * @param[in] max_out: ������
  * @param[in] intergral_limit: �����޷�
  * @param[in] kp/ki/kd: ���� PID ����
  */
extern void pid_init(pid_t *pid, uint32_t max_out, uint32_t intergral_limit, \
              float kp, float ki, float kd);

/**
  * @brief     PID ������λ����
  * @param[in] pid: PID �ṹ��
  * @param[in] kp/ki/kd: ���� PID ����
  */
extern void pid_reset(pid_t *pid, float kp, float ki, float kd);

/**
  * @brief     PID ���㺯����ʹ��λ��ʽ PID ����
  * @param[in] pid: PID �ṹ��
  * @param[in] get: ��������
  * @param[in] set: Ŀ������
  * @retval    PID �������
  */
extern float pid_calc(pid_t *pid, float get, float set);
extern float pid_calc_balance(pid_t *pid, float get, float set,float gyro_y);
extern float pid_calc_KI_Separation(pid_t* pid,float get,float set,float err_threshold);

extern float pid_loop_calc(pid_t *pid,float get,float set,float max_value,float min_value);

#endif
