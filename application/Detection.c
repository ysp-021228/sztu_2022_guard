//
// Created by xhuanc on 2022/3/10.
//

#include "Detection.h"
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "bsp_led.h"

#define DETECT_TASK_INIT_TIME 8000
#define buzzer_remind() buzzer_on(1,18888);
//������ó����Թص���������������
#define BUZZER_REMIND_ENABLE 0

//������ߵ��豸
detect_device_t detect_list[DETECT_DEVICE_LIST_LEN];

static void detect_init(uint16_t index,uint32_t threshold_time,uint8_t warning_level){
//    detect_list[index].enable=1;
    detect_list[index].status=OFFLINE;//Ĭ��������
    detect_list[index].last_online_time=HAL_GetTick();
    detect_list[index].offline_threshold=threshold_time;//�����ж���ֵ
    detect_list[index].warning_level=warning_level;//����ȼ�

}
void detect_handle(uint8_t index){
    detect_list[index].last_online_time=HAL_GetTick();
}
uint16_t buzzer_remind_count=0;
void offline_remind(uint8_t offline_num,uint8_t max_level,uint8_t max_level_count)
{
    uint32_t rgb=0xFF00FF00;//��ɫ

    if(offline_num==0)//�����豸����
    {
        rgb=0xFF00FF00;//ȫ�豸���� �����Ӿ�
    }
    else{
        switch (max_level) {

            case 1: {
                if(detect_list[DETECT_AUTO_AIM].status==OFFLINE)
                                     //��ɫ��ʾ����ɫ �Ӿ����߲���ϵͳ ����ϵͳҲ����
                if(detect_list[DETECT_REFEREE].status==OFFLINE)
                {
                    rgb=0xFFFFFF00;
                }
            }break;

            case 2:{
                rgb=0xFF0000FF;//��ɫΪ ���̵�� level2   �������Ľ������Զ������ȼ������з� һ��ʱ��м��� ��֪���ж���ߵȼ��£�
//                if(BUZZER_REMIND_ENABLE&&detect_list[DETECT_CHASSIS_3508_RF].status==OFFLINE)
//                {
//                    if(buzzer_remind_count==5)//һ��
//                    {
//                        buzzer_remind();
//                    }else{
//                        buzzer_off();
//                    }
//                }
                if(BUZZER_REMIND_ENABLE&&detect_list[DETECT_CHASSIS_3508_LF].status==OFFLINE)
                {
                    if(buzzer_remind_count==5||buzzer_remind_count==10)//����
                    {
                        buzzer_remind();
                    }else{
                        buzzer_off();
                    }
                }
//                if(BUZZER_REMIND_ENABLE&&detect_list[DETECT_CHASSIS_3508_LB].status==OFFLINE)
//                {
//                    if(buzzer_remind_count==5||buzzer_remind_count==10||buzzer_remind_count==15)//����
//                    {
//                        buzzer_remind();
//                    }else{
//                        buzzer_off();
//                    }
//                }
//                if(BUZZER_REMIND_ENABLE&&detect_list[DETECT_CHASSIS_3508_RB].status==OFFLINE)
//                {
//                    if(buzzer_remind_count==5||buzzer_remind_count==10||buzzer_remind_count==15||buzzer_remind_count==20)//����
//                    {
//                        buzzer_remind();
//                    }else{
//                        buzzer_off();
//                    }
//                }
            }break;

            case 3:{
                rgb=0xFFFF00FF;//�ۺ�ɫΪĦ���ֵ�� level3
                if(BUZZER_REMIND_ENABLE&&detect_list[DETECT_LAUNCHER_3508_FIRE_L].status==OFFLINE)
                {
                    if(buzzer_remind_count==5)
                    {
                        buzzer_remind();
                    }else{
                        buzzer_off();
                    }
                }
                if(BUZZER_REMIND_ENABLE&&detect_list[DETECT_LAUNCHER_3508_FIRE_R].status==OFFLINE)
                {
                    if(buzzer_remind_count==5||buzzer_remind_count==10)
                    {
                        buzzer_remind();
                    }else{
                        buzzer_off();
                    }
                }
                if(BUZZER_REMIND_ENABLE&&detect_list[DETECT_LAUNCHER_2006_TRIGGER].status==OFFLINE)
                {
                    if(buzzer_remind_count==5||buzzer_remind_count==10||buzzer_remind_count==15)
                    {
                        buzzer_remind();
                    }else{
                        buzzer_off();
                    }
                }

            }break;

            case 4:{
                rgb=0xFF00FFFF;//��ɫΪ��̨��� level4
                if(BUZZER_REMIND_ENABLE&&detect_list[DETECT_GIMBAL_6020_YAW].status==OFFLINE)
                {
                    if(buzzer_remind_count==5)
                    {
                        buzzer_remind();
                    }else{
                        buzzer_off();
                    }
                }
                if(BUZZER_REMIND_ENABLE&&detect_list[DETECT_GIMBAL_6020_PITCH].status==OFFLINE)
                {
                    if(buzzer_remind_count==5||buzzer_remind_count==10)
                    {
                        buzzer_remind();
                    }else{
                        buzzer_off();
                    }
                }
            }break;

            case 5:{
                rgb=0xFFFF0000;//��ɫΪң���� level5 ң�����Ͳ�����̫����
            }break;

            default:{
                rgb=0xFFFFFFFF;
                buzzer_off();
            }break;
        }
    }
    //�ۼ�
    if(buzzer_remind_count>80)
        buzzer_remind_count=0;
    buzzer_remind_count++;
    //���
    aRGB_led_show(rgb);
}
uint8_t same_level_count[6]={0};

void detect_task(void const*pvParameters){
    vTaskDelay(DETECT_TASK_INIT_TIME);

    //1���Ļ�δ��Ҫ�ȼ�,1��������Ҫ���Ƴ���Ҫ���
    detect_init(DETECT_AUTO_AIM,200,1);
    detect_init(DETECT_REFEREE,200,1);

    detect_init(1,200,2);//200ms�����̵�����

    detect_init(DETECT_LAUNCHER_3508_FIRE_L,200,3);
    detect_init(DETECT_LAUNCHER_3508_FIRE_R,200,3);
    detect_init(DETECT_LAUNCHER_2006_TRIGGER,200,3);

    detect_init(DETECT_GIMBAL_6020_PITCH,200,4);
    detect_init(DETECT_GIMBAL_6020_YAW,200,4);

    detect_init(DETECT_REMOTE,100,5);

    uint8_t offline_num=0;//�����豸����
    uint32_t max_level=0;//0��ʾ��;���ȼ� Ҳ���ǲ�����

    while(1){
        offline_num=0;
        max_level=0;

        for(uint8_t i =0;i<6;i++)
            same_level_count[i]=0;

        for(uint8_t i=0;i<DETECT_DEVICE_LIST_LEN;i++)
        {
            if((HAL_GetTick()-detect_list[i].last_online_time)>
                detect_list[i].offline_threshold)
            {
                detect_list[i].status=OFFLINE;
                offline_num++;//�����豸����
                //��¼�����ʾ�ȼ�
                max_level=detect_list[i].warning_level>=max_level?
                            detect_list[i].warning_level:max_level;
                same_level_count[detect_list[i].warning_level]++;
            }
            else{
                detect_list[i].status=ONLINE;
            }
        }

        //TODO:ģ��������ʾ
        offline_remind(offline_num,max_level,same_level_count[max_level]);

        vTaskDelay(40);
    }
}