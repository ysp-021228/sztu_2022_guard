//
// Created by xhuanc on 2021/11/17.
//

#ifndef DEMO1_FILTER_H
#define DEMO1_FILTER_H
#include "arm_math.h"


/************ һ�� �������˲� *************/

typedef struct {
    float X_last; //��һʱ�̵����Ž��  X(k-|k-1)
    float X_mid;  //��ǰʱ�̵�Ԥ����  X(k|k-1)
    float X_now;  //��ǰʱ�̵����Ž��  X(k|k)
    float P_mid;  //��ǰʱ��Ԥ������Э����  P(k|k-1)
    float P_now;  //��ǰʱ�����Ž����Э����  P(k|k)
    float P_last; //��һʱ�����Ž����Э����  P(k-1|k-1)
    float kg;     //kalman����
    float A;      //ϵͳ����
    float B;
    float Q;
    float R;
    float H;
}first_kalman_filter_t;

//�����˲���
void  first_Kalman_Create(first_kalman_filter_t *p, float T_Q, float T_R);

//���˲���������
float first_Kalman_Filter(first_kalman_filter_t* p, float dat);


//��տ������˲���
//void KalmanClear(first_kalman_filter_t *p);

/************ һ�� �������˲� *************/


/************ ���� �������˲� *************/

#define mat         arm_matrix_instance_f32 //float
#define mat_64      arm_matrix_instance_f64 //double
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32//ת�þ���
#define mat_inv     arm_mat_inverse_f32
#define mat_inv_f64 arm_mat_inverse_f64

typedef struct
{
    float raw_value;
    float filtered_value[2];
    mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} second_kalman_filter_t;

typedef struct
{
    float raw_value;
    float filtered_value[2];
    float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
    float P_data[4];
    float AT_data[4], HT_data[4];//A�0�6�0�1H�0�6�0�2���0�5�0�0�0�9�0�6�0�9�0�1��
    float A_data[4];//
    float H_data[4];
    float Q_data[4];
    float R_data[4];
} second_kalman_filter_init_t;

void second_kalman_filter_init(second_kalman_filter_t *F, second_kalman_filter_init_t *I);

float* second_kalman_filter_calc(second_kalman_filter_t *F, float signal1, float signal2);

#endif

