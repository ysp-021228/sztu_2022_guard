//
// Created by xhuanc on 2022/1/19.
//

#ifndef DEMO1_AUTO_H
#define DEMO1_AUTO_H

#include "struct_typedef.h"

#define AUTO_TASK_INIT_TIME 7200
#define HEAD_LEN 3 //֡ͷ����
#define FRAME_LEN 22 //һ֡����
#define VISION_BUFFER_SIZE 200 //�ֽڻ���������
#define VISION_BUFFER_SEND 22
typedef struct {
    uint8_t head;
    uint8_t cmd;
//    uint8_t crc8;
}vision_frame;
typedef struct {

    vision_frame frame_header;
    union {
        uint8_t data[4];
        fp32 value;
    }pitch;

    union {
        uint8_t data[4];
        fp32 value;
    }yaw;

    union {
        uint8_t data[4];
        fp32 value;
    }distance;

    uint8_t isActive;
    uint8_t reverse_data[5];

}Vision_info_get;
typedef struct {

    vision_frame frame_header;
    union{
        uint8_t data[4];
        fp32 value;
    }pitch;

    union {
        uint8_t data[4];
        fp32 value;
    }yaw;

    uint8_t reverse_data[9];
    uint16_t crc16;

}Vision_info_send;
extern uint8_t usart1_buf[VISION_BUFFER_SIZE];
extern Vision_info_send Vision_send;
extern Vision_info_get Vision_info;
extern void auto_task(void const *pvParameters);
#endif //DEMO1_AUTO_H
