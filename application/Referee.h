//
// Created by xhuanc on 2021/10/23.
//

#ifndef DEMO1_REFEREE_H
#define DEMO1_REFEREE_H
#include "struct_typedef.h"
#include "stdbool.h"

#define REFREE_HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))
#define REFEREE_BUFFER_SIZE     200

#define LEN_HEADER 5

//ͨ��Э���ʽ λ����
typedef enum
{
    FRAME_HEADER= 0,
    CMD_ID               = 5,
    DATA                 = 7,
}RefereeFrameOffset;

// frame_header ��ʽ
typedef enum
{
    SOF          = 0,//��ʼλ
    DATA_LENGTH  = 1,//֡�����ݳ���,�����������ȡ���ݳ���
    SEQ          = 3,//�����
    CRC8         = 4 //CRC8
}FrameHeaderOffset;

//����ϵͳ����ID
/***************������ID********************

	ID: 0x0001  Byte:  3    ����״̬����       			����Ƶ�� 1Hz
	ID: 0x0002  Byte:  1    �����������         		������������
	ID: 0x0003  Byte:  32   ����������Ѫ������   		1Hz����       **
	ID: 0x0004  Byte:  3   	���ڷ���״̬  		?		���ڷ���ʱ����**
	ID: 0x0005  Byte:  3   	�˹�������ս���ӳ���ͷ�����   **

	ID: 0x0101  Byte:  4    �����¼�����   				�¼��ı����
	ID: 0x0102  Byte:  3    ���ز���վ������ʶ����    	�����ı����
	ID: 0X0104  Byte:  2    ���о�������
	ID: 0x0105  Byte:  1    ���ڷ���ڵ���ʱ

	ID: 0X0201  Byte: 15    ������״̬����        		10Hz
	ID: 0X0202  Byte: 14    ʵʱ������������   			50Hz
	ID: 0x0203  Byte: 16    ������λ������           	10Hz
	ID: 0x0204  Byte:  1    ��������������           	����״̬�ı����
	ID: 0x0205  Byte:  3    ���л���������״̬����      10Hz
	ID: 0x0206  Byte:  1    �˺�״̬����           		�˺���������
	ID: 0x0207  Byte:  6    ʵʱ�������           		�ӵ��������
	ID: 0x0208  Byte:  2    ����ʣ������  �����л����� �ڱ�
	ID: 0x0209  Byte:  4    ������RFID״̬

	ID: 0x0301  Byte:  n    �����˼佻������           	���ͷ���������,10Hz

*/

typedef enum
{
    Referee_ID_game_state                   = 0x0001,
    Referee_ID_game_result                  = 0x0002,
    Referee_ID_game_robot_survivors       	= 0x0003,//���������˴������
    Referee_ID_game_dart_state              = 0x0004, //���ڷ���״̬
    Referee_ID_game_buff                    = 0x0005,//buff
    Referee_ID_event_data  					= 0x0101,//�����¼�����
    Referee_ID_supply_projectile_action   	= 0x0102,//���ز���վ������ʶ����
    Referee_ID_supply_warm 	                = 0x0104,//����ϵͳ��������
    Referee_ID_dart_shoot_time              = 0x0105, //���ڷ���ڵ���ʱ
    Referee_ID_game_robot_state    			= 0x0201,//������״̬����
    Referee_ID_power_heat_data    			= 0x0202,//ʵʱ������������
    Referee_ID_game_robot_pos        		= 0x0203,//������λ������
    Referee_ID_buff_musk					= 0x0204,//��������������
    Referee_ID_aerial_robot_energy			= 0x0205,//���л���������״̬����
    Referee_ID_robot_hurt					= 0x0206,//�˺�״̬����
    Referee_ID_shoot_data					= 0x0207,//ʵʱ�������
    Referee_ID_bullet_remaining             = 0x0208,//ʣ�෢����
    Referee_ID_rfid_status					= 0x0209,//������RFID״̬��1Hz
    Referee_ID_dart_client_directive        = 0x020A,//���ڻ����˿ͻ���ָ����, 10Hz
    Referee_ID_robot_interactive_header_data	  = 0x0301,//�����˽������ݣ��������ͷ������������� 10Hz
    Referee_ID_controller_interactive_header_data = 0x0302,//�Զ���������������ݽӿڣ�ͨ�������ͻ��˴����������� 30Hz
    Referee_ID_map_interactive_header_data        = 0x0303,//�ͻ���С��ͼ�������ݣ������������͡���
    Referee_ID_keyboard_information               = 0x0304,//���̡������Ϣ��ͨ������ͼ�����ڡ�������
//    IDCustomData,
}referee_cmd_id_t;



//����ϵͳ����������ݳ���
typedef enum
{
    /* Std */
    Referee_LEN_FRAME_HEAD 	                 = 5,	// ֡ͷ����
    Referee_LEN_CMD_ID 		                   = 2,	// �����볤��
    Referee_LEN_FRAME_TAIL 	                 = 2,	// ֡βCRC16
    /* Ext */

    Referee_LEN_game_state       				=  11,	//0x0001
    Referee_LEN_game_result       				=  1,	//0x0002
    Referee_LEN_game_robot_survivors       		=  32,	//0x0003  ����������Ѫ������
    Referee_LED_game_missile_state      =3  , //0X0004���ڷ���״̬
    Referee_LED_game_buff               =11 , //0X0005

    Referee_LEN_event_data  					=  4,	//0x0101  �����¼�����
    Referee_LEN_supply_projectile_action        =  4,	//0x0102���ز���վ������ʶ����
    Referee_LEN_supply_warm        =2, //����ϵͳ���� 0x0104
    Referee_LEN_missile_shoot_time =1  , //���ڷ���ڵ���ʱ

    Referee_LEN_game_robot_state    			= 27,	//0x0201������״̬����
    Referee_LEN_power_heat_data   				= 16,	//0x0202ʵʱ������������
    Referee_LEN_game_robot_pos        			= 16,	//0x0203������λ������
    Referee_LEN_buff_musk        				=  1,	//0x0204��������������
    Referee_LEN_aerial_robot_energy        		=  1,	//0x0205���л���������״̬����
    Referee_LEN_robot_hurt        				=  1,	//0x0206�˺�״̬����
    Referee_LEN_shoot_data       				=  7,	//0x0207	ʵʱ�������
    Referee_LEN_bullet_remaining          = 6,//ʣ�෢����
    Referee_LEN_rfid_status					         = 4,
    Referee_LEN_dart_client_directive        = 12,//0x020A

}RefereeDataLength;


typedef enum{
     Referee_hero_red       = 1,
     Referee_engineer_red   = 2,
     Referee_infantry3_red  = 3,
     Referee_infantry4_red  = 4,
     Referee_infantry5_red  = 5,
     Referee_plane_red      = 6,

     Referee_hero_blue      = 101,
     Referee_engineer_blue  = 102,
     Referee_infantry3_blue = 103,
     Referee_infantry4_blue = 104,
     Referee_infantry5_blue = 105,
     Referee_plane_blue     = 106,
}Referee_robot_ID;


typedef struct {

    bool static_update;//��̬Ԫ���Ƿ�Ҫˢ��
    uint8_t gimbal_mode;//��̨ģʽ ���˿� �����Կأ��Կ��Ǵ����������)
    uint8_t chassis_mode;//TODO:�Ӳ�������
    uint8_t block_warning;//TODO:�µ�����
    uint8_t shoot_heat_limit;//TODO:��ǰ��������
    fp32 super_cap_value;//TODO:��������ֵ

}ui_robot_status_t;//������״̬�ṹ�� ��������Ƿ��,�����Ƿ�򿪣������Ƿ�򿪵���Ϣ
//����ϵͳ֡ͷ�ṹ��
typedef  struct
{
    uint8_t SOF;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
}__packed frame_header_struct_t;


/* ID: 0x0001  Byte:  11    ����״̬���� */
typedef  struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} __packed ext_game_state_t;


/* ID: 0x0002  Byte:  1    ����������� */
typedef  struct
{
    uint8_t winner;
    bool game_over;
}__packed  ext_game_result_t;


/* ID: 0x0003  Byte:  32    ����������Ѫ������ */
typedef  struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_6_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;

    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_6_robot_HP;
    uint16_t blue_7_robot_HP;

    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} __packed ext_game_robot_HP_t;


/* ID: 0x0004  Byte:  3    ���ڷ���״̬ */
typedef  struct
{
    uint8_t dart_belong;
    uint16_t stage_remaining_time;
} __packed ext_dart_status_t;


/* ID: 0x0005  Byte:  11    buff */
typedef  struct
{
    uint8_t F1_zone_status:1;
    uint8_t F1_zone_buff_debuff_status:3;

    uint8_t F2_zone_status:1;
    uint8_t F2_zone_buff_debuff_status:3;

    uint8_t F3_zone_status:1;
    uint8_t F3_zone_buff_debuff_status:3;

    uint8_t F4_zone_status:1;
    uint8_t F4_zone_buff_debuff_status:3;

    uint8_t F5_zone_status:1;
    uint8_t F5_zone_buff_debuff_status:3;

    uint8_t F6_zone_status:1;
    uint8_t F6_zone_buff_debuff_status:3;

    uint16_t red1_bullet_left;

    uint16_t red2_bullet_left;

    uint16_t blue1_bullet_left;

    uint16_t blue2_bullet_left;

}__packed  ext_ICRA_buff_debuff_zone_status_t;


/* ID: 0x0101  Byte:  4    �����¼����� */
typedef  struct
{
    uint32_t event_type;
} __packed ext_event_data_t;


/* ID: 0x0102  Byte:  4    ���ز���վ������ʶ���� */
typedef  struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} __packed ext_supply_projectile_action_t;


/* ID: 0x0104  Byte: 2   ����ϵͳ������Ϣ */
typedef  struct
{
    uint8_t level;
    uint8_t foul_robot_id;
} __packed  ext_referee_warning_t;


/* ID: 0x0105  Byte:1  ���ڷ���ڵ���ʱ */
typedef  struct
{
    uint8_t dart_remaining_time;
} __packed ext_dart_remaining_time_t;


/* ID: 0X0201  Byte: 27    ������״̬���� */
typedef  struct
{
    uint8_t robot_id;   //������ID��������У�鷢��
    uint8_t robot_level;  //1һ����2������3����
    uint16_t remain_HP;  //������ʣ��Ѫ��
    uint16_t max_HP; //��������Ѫ��

    uint16_t shooter1_17mm_cooling_rate;  //������ 17mm �ӵ�������ȴ�ٶ� ��λ /s
    uint16_t shooter1_17mm_cooling_limit;   // ������ 17mm �ӵ���������
    uint16_t shooter1_17mm_speed_limit;


    uint16_t shooter2_17mm_cooling_rate;
    uint16_t shooter2_17mm_cooling_limit;
    uint16_t shooter2_17mm_speed_limit;


    uint16_t shooter_42mm_cooling_rate;
    uint16_t shooter_42mm_cooling_limit;
    uint16_t shooter_42mm_speed_limit;


    uint16_t max_chassis_power;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} __packed ext_game_robot_state_t;


/* ID: 0X0202  Byte: 16    ʵʱ������������ */
typedef  struct
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;   //˲ʱ����
    uint16_t chassis_power_buffer;//60������������
    uint16_t shooter_heat0;//17mm
    uint16_t shooter_heat1;
    uint16_t mobile_shooter_heat2;
} __packed ext_power_heat_data_t;


/* ID: 0x0203  Byte: 16    ������λ������ */
typedef  struct
{
    float x;
    float y;
    float z;
    float yaw;
} __packed ext_game_robot_pos_t;


/* ID: 0x0204  Byte:  1    �������������� */
typedef  struct
{
    uint8_t power_rune_buff;
} __packed ext_buff_musk_t;


/* ID: 0x0205  Byte:  1    ���л���������״̬���� */
typedef  struct
{
    uint8_t attack_time;
} __packed aerial_robot_energy_t;


/* ID: 0x0206  Byte:  1    �˺�״̬���� */
typedef  struct
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
    bool being_hurt;
} __packed ext_robot_hurt_t;


/* ID: 0x0207  Byte:  7    ʵʱ������� */
typedef  struct
{
    uint8_t bullet_type;
    uint8_t shooter_id;  //���������17����42
    uint8_t bullet_freq;
    float bullet_speed;
}__packed  ext_shoot_data_t;


/* ID: 0x0208  Byte:  6    �ӵ�ʣ������ */
typedef  struct
{
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;//���ʣ��
} __packed ext_bullet_remaining_t;

/* ID: 0x0209  Byte:  4 	������RFID״̬ */
typedef  struct
{
    uint32_t rfid_status;
} __packed ext_rfid_status_t;



typedef  struct{
    uint8_t dart_launch_opening_status;//��ǰ���ڷ���ڵ�״̬
    uint8_t dart_attack_target;        //���ڵĴ��Ŀ�꣬Ĭ��Ϊǰ��վ��1��ǰ��վ��2�����أ�
    uint16_t target_change_time;       //�л����Ŀ��ʱ�ı���ʣ��ʱ��
    uint8_t first_dart_speed;          //��⵽�ĵ�һö�����ٶȣ���λ 0.1m/s/LSB
    uint8_t second_dart_speed;         //��⵽�ĵڶ�ö�����ٶȣ���λ 0.1m/s/LSB
    uint8_t third_dart_speed;          //��⵽�ĵ���ö�����ٶȣ���λ 0.1m/s/LSB
    uint8_t fourth_dart_speed;         //��⵽�ĵ���ö�����ٶȣ���λ 0.1m/s/LSB
    uint16_t last_dart_launch_time;    //���һ�εķ�����ڵı���ʣ��ʱ�䣬��λ��
    uint16_t operate_launch_cmd_time;  //���һ�β�����ȷ������ָ��ʱ�ı���ʣ��ʱ�䣬��λ��
}__packed ext_dart_client_cmd_t; //LEN_DART_CLIENT_DIRECTIVE  ��3-19


/*
	�������ݣ�����һ��ͳһ�����ݶ�ͷ�ṹ��
	���������� ID���������Լ������ߵ� ID ���������ݶΣ�
	�����������ݵİ��ܹ������Ϊ 128 ���ֽڣ�
	��ȥ frame_header,cmd_id,frame_tail �Լ����ݶ�ͷ�ṹ�� 6 ���ֽڣ�
	�ʶ����͵��������ݶ����Ϊ 113��
	������������ 0x0301 �İ�����Ƶ��Ϊ 10Hz��

	������ ID��
	1��Ӣ��(��)��
	2������(��)��
	3/4/5������(��)��
	6������(��)��
	7���ڱ�(��)��
	11��Ӣ��(��)��
	12������(��)��
	13/14/15������(��)��
	16������(��)��
	17���ڱ�(��)��
	�ͻ��� ID��
	0x0101 ΪӢ�۲����ֿͻ���( ��) ��
	0x0102 �����̲����ֿͻ��� ((�� )��
	0x0103/0x0104/0x0105�����������ֿͻ���(��)��
	0x0106�����в����ֿͻ���((��)��
	0x0111��Ӣ�۲����ֿͻ���(��)��
	0x0112�����̲����ֿͻ���(��)��
	0x0113/0x0114/0x0115�������ֿͻ��˲���(��)��
	0x0116�����в����ֿͻ���(��)��
*/


/* �������ݽ�����Ϣ��0x0301  */
typedef  struct
{
    uint16_t data_cmd_id;
    uint16_t send_ID;
    uint16_t receiver_ID;
}__packed ext_student_interactive_header_data_t;

typedef struct{
    uint16_t teammate_hero;
    uint16_t teammate_engineer;
    uint16_t teammate_infantry3;
    uint16_t teammate_infantry4;
    uint16_t teammate_infantry5;
    uint16_t teammate_plane;
    uint16_t teammate_sentry;

    uint16_t client_hero;
    uint16_t client_engineer;
    uint16_t client_infantry3;
    uint16_t client_infantry4;
    uint16_t client_infantry5;
    uint16_t client_plane;
} ext_interact_id_t;

/*
	ѧ�������˼�ͨ�� cmd_id 0x0301������ ID:0x0200~0x02FF
	�������� �����˼�ͨ�ţ�0x0301��
	����Ƶ�ʣ����� 10Hz

	�ֽ�ƫ���� 	��С 	˵�� 			��ע
	0 			2 		���ݵ����� ID 	0x0200~0x02FF
										���������� ID ��ѡȡ������ ID �����ɲ������Զ���

	2 			2 		�����ߵ� ID 	��ҪУ�鷢���ߵ� ID ��ȷ�ԣ�

	4 			2 		�����ߵ� ID 	��ҪУ������ߵ� ID ��ȷ�ԣ�
										���粻�ܷ��͵��жԻ����˵�ID

	6 			n 		���ݶ� 			n ��ҪС�� 113

*/
typedef  struct
{
    uint8_t data[113]; //���ݶ�,n��ҪС��113
} __packed robot_interactive_data_t;

typedef struct judge_info_struct {
    frame_header_struct_t 							FrameHeader;				// ֡ͷ��Ϣ

    ext_game_state_t 							    GameState;				// 0x0001           ����״̬����
    ext_game_result_t 							    GameResult;				// 0x0002         �����������
    ext_game_robot_HP_t 						    GameRobotHP;			// 0x0003         ������Ѫ������
    ext_dart_status_t								GameDartStatus;				// 0x0004         ���ڷ���״̬
    ext_ICRA_buff_debuff_zone_status_t	            GameICRABuff;      //                �˹�������ս���ӳ���ͷ���״̬

    ext_event_data_t								EventData;					// 0x0101         �����¼�����
    ext_supply_projectile_action_t	                SupplyProjectileAction;		// 0x0102 ����վ������ʶ
    ext_referee_warning_t						    RefereeWarning;		// 0x0104         ���о�����Ϣ
    ext_dart_remaining_time_t				        DartRemainingTime;// 0x0105         ���ڷ���ڵ���ʱ

    ext_game_robot_state_t					        GameRobotStat;	// 0x0201         ����������״̬
    ext_power_heat_data_t						    PowerHeatData;		// 0x0202         ʵʱ������������
    ext_game_robot_pos_t						    GameRobotPos;			// 0x0203         ������λ��
    ext_buff_musk_t									Buff;								// 0x0204     ����������
    aerial_robot_energy_t				            AerialRobotEnergy;// 0x0205             ���л���������״̬
    ext_robot_hurt_t								RobotHurt;					// 0x0206         �˺�״̬
    ext_shoot_data_t								ShootData;					// 0x0207         ʵʱ�����Ϣ(��Ƶ  ����  �ӵ���Ϣ)
    ext_bullet_remaining_t					        BulletRemaining;		// 0x0208	        �ӵ�ʣ�෢����
    ext_rfid_status_t								RfidStatus;				// 0x0209	        RFID��Ϣ
    ext_dart_client_cmd_t                           DartClient;        // 0x020A         ���ڿͻ���

    ext_interact_id_t								ids;								//�뱾�������Ļ�����id
    uint16_t                                        SelfClient;        //�����ͻ���

} Referee_info_t;


/*
	ѧ�������˼�ͨ�� cmd_id 0x0301������ data_ID:0x0200~0x02FF
	�������� �����˼�ͨ�ţ�0x0301��
	����Ƶ�ʣ����������кϼƴ������� 5000 Byte�� �����з���Ƶ�ʷֱ𲻳���30Hz��
 * +------+------+-------------+------------------------------------+
 * | byte | size |    breif    |            note                    |
 * |offset|      |             |                                    |
 * +------+------+-------------+------------------------------------+
 * |  0   |  2   | 	 data_ID   | 0x0200~0x02FF,��������Щ ID ��ѡȡ |
 * |      |      |             | ����ID�����ɲ������Զ���           |
 * +------|------|-------------|------------------------------------|
 * |  2   |  2   | 	sender_ID  | ��ҪУ�鷢���ߵ� ID ��ȷ��					|
 * +------|------|-------------|------------------------------------|
 * |  4   |  2   | receiver_ID | ��ҪУ������ߵ� ID ��ȷ��					|
 * |      |      |             | ���粻�ܷ��͵��жԻ����˵�ID				|
 * +------|------|-------------|------------------------------------|
 * |  6   |  n   | 		Data     | n ��ҪС�� 113 										|
 * +------+------+-------------+------------------------------------+
*/

typedef enum
{
    //0x200-0x02ff 	�����Զ������� ��ʽ  INTERACT_ID_XXXX
     UI_INTERACT_ID_delete_graphic 			= 0x0100,	/*�ͻ���ɾ��ͼ��*/
     UI_INTERACT_ID_draw_one_graphic 		= 0x0101,	/*�ͻ��˻���һ��ͼ��*/
     UI_INTERACT_ID_draw_two_graphic 		= 0x0102,	/*�ͻ��˻���2��ͼ��*/
     UI_INTERACT_ID_draw_five_graphic 	= 0x0103,	/*�ͻ��˻���5��ͼ��*/
     UI_INTERACT_ID_draw_seven_graphic 	= 0x0104,	/*�ͻ��˻���7��ͼ��*/
     UI_INTERACT_ID_draw_char_graphic 	= 0x0110,	/*�ͻ��˻����ַ�ͼ��*/
     UI_INTERACT_ID_bigbome_num					= 0x02ff
}Interact_ID;

typedef enum
{
   UI_LEN_INTERACT_delete_graphic     = 8,  //ɾ��ͼ�� 2(��������ID)+2(������ID)+2��������ID��+2���������ݣ�
   UI_LEN_INTERACT_draw_one_graphic   = 21, // ����2+2+2+15
   UI_LEN_INTERACT_draw_two_graphic   = 36, //6+15*2
   UI_LEN_INTERACT_draw_five_graphic  = 81, //6+15*5
   UI_LEN_INTERACT_draw_seven_graphic = 111,//6+15*7
   UI_LEN_INTERACT_draw_char_graphic  = 51, //6+15+30���ַ������ݣ�
}Interact_ID_len;

//TODO:���ղ��Բ��� ���ٽṹ��������� ���������ͷ�Ϊ{�ַ���}��{���֡�ͼ��}���� ��Ϊ�ַ����޷�һ֡�໭ �����߿��� ���Է�Ϊ������
//TODO:������֡�Ľṹ�����ͷ�Ϊ string ���one,two,five,seven�� ���忴����ϵͳ����Э��
typedef  struct//ͼ��
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye:3;
    uint32_t graphic_tpye:3;          //ֱ��  ����  ��Բ  ��Բ  Բ��  ����  ����  �ַ�
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;           //��    ��    ��    ��    �Ƕ�  ��С  ��С  ��С
    uint32_t end_angle:9;             //��    ��    ��    ��          λ��  ��    ����
    uint32_t width:10;
    uint32_t start_x:11;              //���  ���  Բ��  Բ��  Բ��  ���  ���  ���
    uint32_t start_y:11;              //
    union {
        struct {
            uint32_t radius:10;      //��    ��    �뾶  ��    ��    ��    ��    ��
            uint32_t end_x:11;       //�յ�  �Զ�  ��    ����  ����  ��    ��    ��
            uint32_t end_y:11;       //                              ��    ��    ��                  ��    ��    ��
        };
        int32_t number;
    };

} __packed ui_graphic_data_struct_t;//ui��ͷ �����κ͸�����Ҳ��Ϊͼ�� ������graphic����

typedef  struct
{
    frame_header_struct_t txFrameHeader;			//֡ͷ
    uint16_t  CmdID;										//������
    ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
    ui_graphic_data_struct_t clientData;		//���ݶ�
    uint16_t	FrameTail;								//֡β
}__packed ext_graphic_one_data_t;

typedef  struct
{
    frame_header_struct_t txFrameHeader;			//֡ͷ
    uint16_t  CmdID;										//������
    ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
    ui_graphic_data_struct_t clientData[2];		//���ݶ�
    uint16_t	FrameTail;								//֡β
}__packed ext_graphic_two_data_t;

typedef  struct
{
    frame_header_struct_t txFrameHeader;			//֡ͷ
    uint16_t  CmdID;										//������
    ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
    ui_graphic_data_struct_t clientData[5];		//���ݶ�
    uint16_t	FrameTail;								//֡β
}__packed ext_graphic_five_data_t;

typedef  struct
{
    frame_header_struct_t txFrameHeader;			//֡ͷ
    uint16_t  CmdID;										//������
    ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
    ui_graphic_data_struct_t clientData[7];		//���ݶ�
    uint16_t	FrameTail;								//֡β
}__packed ext_graphic_seven_data_t;

//���ַ���
//�ַ�������ui_graphic_data_struct_t����ṹ�������
// ����30���ֽڵ��ַ����ݴ洢�ַ���
typedef  struct
{
    ui_graphic_data_struct_t graphic_data_struct;
    uint8_t data[30];
}__packed ui_string_t;

//�̶����ݶγ������ݰ�
typedef  struct
{
    frame_header_struct_t txFrameHeader;			//֡ͷ
    uint16_t  CmdID;										//������
    ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
    ui_string_t clientData;//���ݶ�
    uint16_t	FrameTail;								//֡β
}__packed ext_string_data_t;

//****************************��ͼ�����ݶ�����****************************/


//typedef  struct//ͼ��
//{
//    uint8_t graphic_name[3];
//    uint32_t operate_tpye:3;
//    uint32_t graphic_tpye:3; //ֱ��  ����  ��Բ  ��Բ  Բ��  ����  ����  �ַ�
//    uint32_t layer:4;
//    uint32_t color:4;
//    uint32_t start_angle:9;  //��    ��    ��    ��    �Ƕ�  ��С  ��С  ��С
//    uint32_t end_angle:9;    //��    ��    ��    ��          λ��  ��    ����
//    uint32_t width:10;
//    uint32_t start_x:11;     //���  ���  Բ��  Բ��  Բ��  ���  ���  ���
//    uint32_t start_y:11;     //
//} __packed graphic_data_struct_t;
//
//typedef  struct//������
//{
//    uint8_t graphic_name[3];
//    uint32_t operate_tpye:3;
//    uint32_t graphic_tpye:3;
//    uint32_t layer:4;
//    uint32_t color:4;
//    uint32_t start_angle:9;
//    uint32_t end_angle:9;
//    uint32_t width:10;
//    uint32_t start_x:11;
//    uint32_t start_y:11;
//    uint32_t number;
//}__packed  float_data_struct_t;
//
//typedef  struct//������
//{
//    uint8_t graphic_name[3];
//    uint32_t operate_tpye:3;
//    uint32_t graphic_tpye:3;
//    uint32_t layer:4;
//    uint32_t color:4;
//    uint32_t start_angle:9;
//    uint32_t end_angle:9;
//    uint32_t width:10;
//    uint32_t start_x:11;
//    uint32_t start_y:11;
//    uint32_t number;
//} __packed int_data_struct_t;

/* data_ID: 0X0100  Byte:  2	    �ͻ���ɾ��ͼ��*/
typedef  struct
{
    uint8_t operate_type;
    uint8_t layer;//ͼ������0~9
}__packed ext_client_custom_graphic_delete_t;

/* ͼ��ɾ��������ö�� */
typedef enum
{
    UI_NONE_delete    = 0,
    UI_GRAPHIC_delete = 1,
    UI_ALL_delete     = 2
}Delete_Graphic_Operate;//ext_client_custom_graphic_delete_t��uint8_t operate_type

//bit 0-2
typedef enum
{
    UI_NONE   = 0,/*�ղ���*/
    UI_ADD    = 1,/*����ͼ��*/
    UI_MODIFY = 2,/*�޸�ͼ��*/
    UI_DELETE = 3,/*ɾ��ͼ��*/
}Graphic_Operate;//graphic_data_struct_t��uint32_t operate_tpye
/*ͼ�����*/

//bit3-5
typedef enum
{
    UI_LINE      = 0,//ֱ��
    UI_RECTANGLE = 1,//����
    UI_CIRCLE    = 2,//��Բ
    UI_OVAL      = 3,//��Բ
    UI_ARC       = 4,//Բ��
    UI_FLOAT     = 5,//������
    UI_INT       = 6,//������
    UI_CHAR      = 7 //�ַ�
}Graphic_Type;
/*ͼ������*/

//bit 6-9ͼ���� ���Ϊ9����С0


//bit 10-13��ɫ
typedef enum
{
    UI_RED_BLUE  = 0,//������ɫ
    UI_YELLOW    = 1,
    UI_GREEN     = 2,
    UI_ORANGE    = 3,
    UI_FUCHSIA   = 4,	/*�Ϻ�ɫ*/
    UI_PINK      = 5,
    UI_CYAN_BLUE = 6,	/*��ɫ*/
    UI_BLACK     = 7,
    UI_WHITE     = 8
}Graphic_Color;

typedef enum {

    UI_ZERO_LAYER=0,
    UI_ONE_LAYER,
    UI_TWO_LAYER,
    UI_THREE_LAYER,
    UI_FOUR_LAYER,
    UI_FIVE_LAYER,
    UI_SIX_LAYER,
    UI_SEVEN_LAYER,
    UI_EIGHT_LAYER,

}Graphic_layer;
/*ͼ����ɫ����*/
//bit 14-31 �Ƕ� [0,360]

/*
 * ���ݽṹ��
 */

//ɾ��ͼ��
typedef  struct
{
    frame_header_struct_t txFrameHeader;
    uint16_t  CmdID;
    ext_student_interactive_header_data_t   dataFrameHeader;
    ext_client_custom_graphic_delete_t clientData;
    uint16_t	FrameTail;
} __packed ext_deleteLayer_data_t;



////������ͼ
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;			//֡ͷ
//    uint16_t  CmdID;										//������
//    ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
//    graphic_data_struct_t clientData;		//���ݶ�
//    uint16_t	FrameTail;								//֡β
//}__packed ext_graphic_one_data_t;
//
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    graphic_data_struct_t clientData[2];
//    uint16_t	FrameTail;
//}__packed ext_graphic_two_data_t;
//
//
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    graphic_data_struct_t clientData[5];
//    uint16_t	FrameTail;
//}__packed ext_graphic_five_data_t;
//
//
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    graphic_data_struct_t clientData[7];
//    uint16_t	FrameTail;
//}__packed ext_graphic_seven_data_t;
//
//
////���Ƹ�����
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    float_data_struct_t clientData[2];
//    uint16_t	FrameTail;
//}__packed ext_float_two_data_t;
//
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    float_data_struct_t clientData;
//    uint16_t	FrameTail;
//}__packed ext_float_one_data_t;
//
//
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    float_data_struct_t clientData[7];
//    uint16_t	FrameTail;
//}__packed ext_float_seven_data_t;
//
////��������
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    int_data_struct_t clientData[2];
//    uint16_t	FrameTail;
//}__packed ext_int_two_data_t;
//
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    int_data_struct_t clientData;
//    uint16_t	FrameTail;
//}__packed ext_int_one_data_t;
//
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    int_data_struct_t clientData[7];
//    uint16_t	FrameTail;
//}__packed ext_int_seven_data_t;

extern ui_robot_status_t ui_robot_status;
extern Referee_info_t Referee;
extern void referee_task(void const*argument);
extern void UI_paint_task(void const*argument);
extern uint8_t usart6_buf[REFEREE_BUFFER_SIZE];


#endif //DEMO1_REFEREE_H
