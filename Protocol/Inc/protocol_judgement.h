#ifndef __PROTOCOL_JUDGEMENT_H
#define __PROTOCOL_JUDGEMENT_H

#include "stm32f4xx.h"
//#define SEND_BUF_SIZE 1000	//�������ݳ���,��õ���sizeof(TEXT_TO_SEND)+2��������.

//u8 SendBuff[SEND_BUF_SIZE];
#define FrameHeader_Len         5 
#define JudgeFrameHeader        0xA5        //֡ͷ

typedef __packed struct
{
	uint8_t 	SOF;//����֡��ʼ�ֽڣ��̶�ֵΪ0xA5;
	uint16_t 	DataLength; //����֡��Data����;
	uint8_t	Seq;//�����;
	uint8_t	CRC8;//֡ͷCRC8;
}tMsg_head;//֡ͷ5λ����

#define CMD_LEN      2    //cmdid bytes
#define CRC_LEN      2    //crc16 bytes
#define HEADER_LEN   sizeof(tMsg_head) 

typedef enum
{
	CmdID_1 = 0x0001,//����״̬���ݣ�1HzƵ�����ڷ���;
	CmdID_2 = 0x0002,//����������ݣ�������������;
	CmdID_3 = 0x0003,//����������Ѫ����1Hz���ڷ���;
	CmdID_4 = 0x0101,//�����¼����ݣ��¼��ı����;
	CmdID_5 = 0x0102,//���ز���վ������ʶ���ݣ������ı����;
	CmdID_6 = 0x0103,//���󲹸�վ�������ݣ���MA�����ַ���;
	CmdID_7 = 0x0104,//���о������ݣ����淢�ͺ���;
	CmdID_8 = 0x0201,//������״̬���ݣ�10Hz���ͣ�
	CmdID_9 = 0x0202, //ʵʱ�����������ݣ�50Hz���ڷ��ͣ�
	CmdID_10 = 0x0203,//������λ�����ݣ�10Hz���ͣ�
	CmdID_11 = 0x0204,//�������������ݣ�����״̬�ı���ͣ�
	CmdID_12 = 0x0205,//ֻ�����˻������л���������״̬���ݣ�10Hz���ڷ��ͣ�
	CmdID_13 = 0x0206,//�˺�״̬���ݣ��˺����ͷ������ͣ�
	CmdID_14 = 0x0207,//ʵʱ������ݣ����跢����ͣ�
	CmdID_15 = 0x0208,//ʣ�൯��������������л��������ڱ���
	CmdID_16 = 0x0301,//�����˼佻�����ݣ����Ƶ��10Hz�����ͷ���������

}Msg_CmdID;//��λ����


//����״̬cmd_id=0x0001
typedef __packed struct
{
	uint16_t rxCmdId;		//��������
	uint8_t game_type : 4;             //��������
	uint8_t game_progress : 4; 	//������ǰ״̬
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
}ext_game_status_t;


//�������
typedef __packed struct
{

	uint8_t winner;                   //�������
}ext_game_result_t;

//cmid=0x0003
typedef __packed struct
{

	uint16_t red_1_robot_HP;     //��1HPӢ��
	uint16_t red_2_robot_HP;     //��2HP����
	uint16_t red_3_robot_HP;     //��3HP����
	uint16_t red_4_robot_HP;     //��4HP����
	uint16_t red_5_robot_HP;     //��5HP����
	uint16_t red_7_robot_HP;     //��7�ڱ�HP
	uint16_t red_outpost_HP;     //�췽ǰ��
	uint16_t red_base_HP;        //����HP
	uint16_t blue_1_robot_HP;    //��1HPӢ��
	uint16_t blue_2_robot_HP;    //��2HP����
	uint16_t blue_3_robot_HP;    //��3HP����
	uint16_t blue_4_robot_HP;    //��4HP����
	uint16_t blue_5_robot_HP;    //��5HP����
	uint16_t blue_7_robot_HP;    //��7�ڱ�HP
	uint16_t blue_outpost_HP;    //����ǰ��
	uint16_t blue_base_HP;	   //����HP
}ext_game_robot_HP_t;

//���ڷ���״̬��0x0004
typedef __packed struct
{
	uint8_t dart_belong;
	uint16_t stage_remaining_time;
} ext_dart_status_t;



//���о�����Ϣcmd_id=0x0104
typedef __packed struct
{

	uint8_t level;               //����ȼ�
	uint8_t foul_robot_id;       //���������ID
}ext_referee_warning_t;
//������״̬cmd_id=0x0201
typedef __packed struct
{

	uint8_t  robot_id;                      //������ID
	uint8_t  robot_level;                   //�����˵ȼ�
	uint16_t remain_HP;                    //������ʣ��Ѫ��
	uint16_t max_HP;                       //����������Ѫ��
	uint16_t shooter_id1_17mm_cooling_rate;   //������17mmǹ��ÿ����ȴֵ
	uint16_t shooter_id1_17mm_cooling_limit;  //������42mmǹ����������
	uint16_t shooter_id1_17mm_speed_limit;   //������17mmǹ��ÿ����ȴֵ

	uint16_t shooter_id2_17mm_cooling_rate; //������42mmǹ����������
	uint16_t shooter_id2_17mm_cooling_limit;
	uint16_t shooter_id2_17mm_speed_limit;

	uint16_t shooter_id1_42mm_cooling_rate;
	uint16_t shooter_id1_42mm_cooling_limit;
	uint16_t shooter_id1_42mm_speed_limit;

	uint16_t chassis_power_limit;
	uint8_t  mains_power_gimbal_output : 1; //0 bit��gimbal ������� 1 Ϊ�� 24V �����0 Ϊ�� 24v �����
	uint8_t  mains_power_chassis_output : 1;//1 bit��chassis �������1 Ϊ�� 24V �����0 Ϊ�� 24v �����
	uint8_t  mains_power_shooter_output : 1;//2 bit��shooter �������1 Ϊ�� 24V �����0 Ϊ�� 24v �����
}ext_game_robot_status_t;

//ʵʱ�����Ϣ0x207
typedef __packed struct
{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float bullet_speed;
}ext_shoot_data_t;


typedef enum
{
	Robot_Status_ID = 0x0201,//������״̬  �ȼ�
	power_heat_data_ID = 0x0202,//ǹ������ ���̹���
	robot_hurt_ID = 0x0206,//�˺�����
	shoot_data_ID = 0x0207,//��Ƶ����
	student_interactive_header_ID = 0x0301,
} Judege_Cmd_ID;


typedef enum
{
	GameInfo = 0x0001,
	RealBloodChangedData,
	RealShootData,
	SelfDefinedData = 0x0100,
	Wrong = 0x1301
}tCmdID;

typedef __packed struct
{
	float data1;
	float data2;
	float data3;
	uint8_t mask;
}tSelfDefineInfo;

typedef  __packed struct
{
	float data1;
	float data2;
	float data3;
	uint8_t mask;
}SelfDefineInfo_t;   //ѧ���ϴ��Զ������� (0x0005) 

typedef __packed struct
{
	uint16_t rxCmdId;
	uint16_t data_id;
	uint16_t send_id;
	uint16_t receive_id;
} id_data_t;


typedef __packed struct
{
	tMsg_head Header;
	id_data_t    id;
	float data1;
	float data2;
	float data3;
	uint8_t masks;
	uint16_t crc_16;
} client_custom_data_t;

//ʵʱ��������cmd_id=0x0202
typedef __packed struct
{

	uint16_t chassis_volt;                //���������ѹ mv
	uint16_t chassis_current;             //����������� ma
	float chassis_power;                  //����������� w
	uint16_t chassis_power_buffer;        //���̹��ʻ��� j��������Ҫ250j
	uint16_t shooter_id1_17mm_cooling_heat;            //17mmǹ������
	uint16_t shooter_id2_17mm_cooling_heat;              //42mmǹ������
	uint16_t shooter_id1_42mm_cooling_heat;       //����17mmǹ������
}ext_power_heat_data_t;



typedef __packed struct
{
	float x;
	float y;
	float z;
	float yaw;
}ext_game_robot_pos_t;

//����������
typedef __packed struct
{
	uint8_t power_rune_buff;
}ext_buff_t;


typedef struct
{
	tMsg_head                     judgedatahead;
	uint16_t                      rxCmdId;
	ext_game_status_t            game_status;           //����״̬
	ext_game_robot_status_t      game_robot_state_t;    //������״̬
	ext_game_robot_HP_t          game_robot_HP_t;       //������Ѫ��
	ext_power_heat_data_t        power_heat_data_t;     //�����˹���������
	ext_game_robot_pos_t         game_robot_position;   //������λ��
	ext_shoot_data_t	         shoot_data_t;
	ext_referee_warning_t        referee_warning_t;     //���о�����Ϣ	
	client_custom_data_t         robot_data_t;
	client_custom_data_t          userinfo;

}judge_type;

typedef __packed struct
{
	tMsg_head       FrameHeader;
	tCmdID          CmdID;
	tSelfDefineInfo SelfDefineInfo;
	uint16_t        CRC16;
}tFrame;

// �������� �����˼�ͨ�ţ� 0x0301������Ƶ�ʣ����� 10Hz
typedef __packed struct
{
	uint8_t data[30];
} robot_interactive_data_t;

typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
}ext_student_interactive_header_data_t;


///////////////////////////////////ͼ�νṹ��
typedef __packed struct
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye : 3;
	uint32_t graphic_tpye : 3;
	uint32_t layer : 4;
	uint32_t color : 4;
	uint32_t start_angle : 9;
	uint32_t end_angle : 9;
	uint32_t width : 10;
	uint32_t start_x : 11;
	uint32_t start_y : 11;
	uint32_t radius : 10;
	uint32_t end_x : 11;
	uint32_t end_y : 11;
}graphic_data_struct_t;
typedef __packed struct
{
	tMsg_head       											 			UIMsg_head;
	uint16_t         													CmdID;
	ext_student_interactive_header_data_t  		UIdraw_header_id;
	uint8_t graphic_name[3];
	uint32_t operate_tpye : 3;
	uint32_t graphic_tpye : 3;
	uint32_t layer : 4;
	uint32_t color : 4;
	uint32_t start_angle : 9;
	uint32_t end_angle : 9;
	uint32_t width : 10;
	uint32_t start_x : 11;
	uint32_t start_y : 11;
	int32_t 	graph_num;
	uint16_t        													CRC16;
}Num_data_struct_t;
typedef __packed struct
{
	tMsg_head       											 		UIMsg_head;
	uint16_t         													CmdID;
	ext_student_interactive_header_data_t  		UIdraw_header_id;
	graphic_data_struct_t                     graphic_data[7];
	uint16_t        													CRC16;
}draw_data_struct_t;
typedef __packed struct
{
	ext_student_interactive_header_data_t  UIdraw_header_id;
	graphic_data_struct_t grapic_data_struct;
}ext_client_custom_graphic_single_t;

typedef  struct
{
	uint16_t              rxCmdId;
	__packed union
	{
		ext_game_status_t            game_status;           //����״̬
		ext_game_robot_status_t      game_robot_state_t;    //������״̬
		ext_game_robot_HP_t          game_robot_HP_t;       //������Ѫ��
		ext_power_heat_data_t        power_heat_data_t;     //�����˹���������
		ext_game_robot_pos_t         game_robot_position;
		ext_shoot_data_t		 				 shoot_data_t;
		ext_referee_warning_t        referee_warning_t; 		//���о�����Ϣ
		robot_interactive_data_t     robot_data_t;
		client_custom_data_t 	userinfo;
		graphic_data_struct_t	graphic_data;
	}Data;
	uint16_t        CRC16;
	uint16_t        CRC16_2;
}FRAME;

/*--------------------------------�ӿڽṹ��----------------------------------------------*/
//����ϵͳ���ݽṹ
typedef struct
{
	uint8_t  robo_level;//�����˵�ǰ�ȼ�
	uint8_t  robo_ID;		//�����˵�ǰID
	uint16_t robo_HP;
	float robo_17_Speed;
	//.....................
}robo_data_t;

typedef __packed struct
{
	tMsg_head       											 		UIMsg_head;
	uint16_t         													CmdID;
	ext_student_interactive_header_data_t  		UIdraw_header_id;
	graphic_data_struct_t                     graphic_data;
	uint16_t        													CRC16;
}graph_data_struct_t;
typedef __packed struct
{
	tMsg_head       											 UIMsg_head;
	uint16_t         											 CmdID;
	ext_student_interactive_header_data_t  CharUI_header_id;
	graphic_data_struct_t                  char_data;
	uint8_t data[30];
	uint16_t        											 CRC16;
}char_data_struct_t;
typedef __packed struct
{
	tMsg_head       											 			Del_head;
	uint16_t         													CmdID;
	ext_student_interactive_header_data_t  		UIdraw_header_id;
	uint8_t Delete_Operate;
	uint8_t Layer;
	uint16_t        											 			CRC16;
} ext_client_custom_graphic_delete_t;
typedef __packed struct
{
	graphic_data_struct_t Char_data_struct;
	uint8_t data[30];
}ext_client_custom_character_t;
typedef __packed struct
{
	tMsg_head       											 Map_head;
	uint16_t         											 CmdID;
	uint16_t         											 Target_Robot_ID;
	float 																 Target_Position_x;
	float 																 Target_Position_y;
	float																	 Reserverd;
	uint16_t        											 CRC16;
}map_data_struct_t;

#define RX_BUF_NUM   1000u
#define TX_BUF_NUM   512u

extern  ext_game_robot_status_t      game_robot_state_t;
extern  ext_game_status_t            game_status;
extern  ext_game_robot_HP_t          game_robot_HP_t;
extern  ext_power_heat_data_t        power_heat_data_t;
extern  ext_referee_warning_t        referee_warning_t;
extern	tMsg_head                    judgedatahead;
extern  SelfDefineInfo_t             SelfDefineInfo;
extern  tFrame						tframe;
extern  judge_type                   judgetype;
extern  ext_game_robot_pos_t         game_robot_position;
extern  ext_shoot_data_t			shoot_data_t;
extern	uint8_t						rx7_buf[RX_BUF_NUM];
extern  uint8_t						TX7_buf[TX_BUF_NUM];
extern	ext_game_robot_pos_t	game_robot_pos_t;
extern	draw_data_struct_t		draw_data_struct;
extern  char_data_struct_t    char_data_struct;
extern  graph_data_struct_t   graph_data_struct;
extern  ext_client_custom_character_t  ext_client_custom_characte;
extern  ext_client_custom_graphic_delete_t graphic_delete_struct;
// extern  num_data_struct_t  						num_data_struct;

extern	ext_client_custom_graphic_single_t   client_custom_graphic_single_t;
void usart7_send_char(uint8_t c);
void Usart_SendBuff(uint8_t *buf, uint16_t len);
void chassis_to_judgeui(uint16_t txlen);
const robo_data_t *get_robo_data_Point();
extern void usart7_DMA_init(void);
void uart7_dma_get(void);

#endif /* __PROTOCOL_JUDGEMENT_H */
