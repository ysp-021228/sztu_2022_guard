
#ifndef DEMO1_KEY_BOARD_H
#define DEMO1_KEY_BOARD_H
#include "remote.h"

#define DEFAULT_CNT 100/24 // 100/��������=ʵ�ʾ���ʱ��
#define MOUSE_CLICK_R_CNT 10/4 // 100/��������=ʵ�ʾ���ʱ��
#define MOUSE_CLICK_L_CNT 200/4 // 100/��������=ʵ�ʾ���ʱ��
#define MOUSE_X_RADIO 0.0036f
#define MOUSE_Y_RADIO 0.0036f

typedef enum{
    KEY_RELAX,//����û������
    KEY_DOWN,
    KEY_CLICK,//PRESS һ��֮���RELAX ����CLICK
    KEY_PRESS,//PRESS ����һ��ʱ�� DOWN
}Key_Status;

typedef struct {
    Key_Status status;
    Key_Status last_status;
    uint32_t press_cnt;
    uint8_t click_flag;
    uint32_t click_cnt;
}Key;

//���̽ṹ��
typedef struct {
    Key Mouse_l;
    Key Mouse_r;
    Key W;
    Key S;
    Key A;
    Key D;
    Key Q;
    Key E;
    Key R;
    Key F;
    Key G;
    Key Z;
    Key X;
    Key C;
    Key B;
    Key V;
    Key SHIFT;
    Key CTRL;
}key_board_t;
extern void update_pc_info();

#endif //DEMO1_KEY_BOARD_H
