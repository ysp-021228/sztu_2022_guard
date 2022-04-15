//
// Created by xhuanc on 2021/11/24.
//

#ifndef DEMO1_RAMP_H
#define DEMO1_RAMP_H
#include "struct_typedef.h"

typedef  struct
{
    fp32 input;        //��������
    fp32 out;          //�������
    fp32 min_value;    //�޷���Сֵ
    fp32 max_value;    //�޷����ֵ
    fp32 frame_period; //ʱ����
}__packed ramp_function_source_t;

void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);

#endif //DEMO1_RAMP_H
