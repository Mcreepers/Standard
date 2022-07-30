#ifndef __PROTOCOL_JUDGEMENT_H
#define __PROTOCOL_JUDGEMENT_H

#include "stm32f4xx.h"
//#define SEND_BUF_SIZE 1000	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.

//u8 SendBuff[SEND_BUF_SIZE];
#define FrameHeader_Len         5 
#define JudgeFrameHeader        0xA5        //帧头

typedef __packed struct
{
	uint8_t 	SOF;//数据帧起始字节，固定值为0xA5;
	uint16_t 	DataLength; //数据帧内Data长度;
	uint8_t	Seq;//包序号;
	uint8_t	CRC8;//帧头CRC8;
}tMsg_head;//帧头5位数据

#define CMD_LEN      2    //cmdid bytes
#define CRC_LEN      2    //crc16 bytes
#define HEADER_LEN   sizeof(tMsg_head) 

typedef enum
{
	CmdID_1 = 0x0001,//比赛状态数据，1Hz频率周期发送;
	CmdID_2 = 0x0002,//比赛结果数据，比赛结束发送;
	CmdID_3 = 0x0003,//比赛机器人血量，1Hz周期发送;
	CmdID_4 = 0x0101,//场地事件数据，事件改变后发送;
	CmdID_5 = 0x0102,//场地补给站动作标识数据，动作改变后发送;
	CmdID_6 = 0x0103,//请求补给站补弹数据，由MA操作手发送;
	CmdID_7 = 0x0104,//裁判警告数据，警告发送后发送;
	CmdID_8 = 0x0201,//机器人状态数据，10Hz发送；
	CmdID_9 = 0x0202, //实时功率热量数据，50Hz周期发送；
	CmdID_10 = 0x0203,//机器人位置数据，10Hz发送；
	CmdID_11 = 0x0204,//机器人增益数据，增益状态改变后发送；
	CmdID_12 = 0x0205,//只有无人机，空中机器人能量状态数据，10Hz周期发送，
	CmdID_13 = 0x0206,//伤害状态数据，伤害发送发生后发送；
	CmdID_14 = 0x0207,//实时射击数据，弹丸发射后发送；
	CmdID_15 = 0x0208,//剩余弹丸数，限用与空中机器人与哨兵；
	CmdID_16 = 0x0301,//机器人间交互数据，最大频率10Hz，发送方触发发送

}Msg_CmdID;//两位数据


//比赛状态cmd_id=0x0001
typedef __packed struct
{
	uint16_t rxCmdId;		//比赛类型
	uint8_t game_type : 4;             //比赛类型
	uint8_t game_progress : 4; 	//比赛当前状态
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
}ext_game_status_t;


//比赛结果
typedef __packed struct
{

	uint8_t winner;                   //比赛结果
}ext_game_result_t;

//cmid=0x0003
typedef __packed struct
{

	uint16_t red_1_robot_HP;     //红1HP英雄
	uint16_t red_2_robot_HP;     //红2HP工程
	uint16_t red_3_robot_HP;     //红3HP步兵
	uint16_t red_4_robot_HP;     //红4HP步兵
	uint16_t red_5_robot_HP;     //红5HP步兵
	uint16_t red_7_robot_HP;     //红7哨兵HP
	uint16_t red_outpost_HP;     //红方前哨
	uint16_t red_base_HP;        //基地HP
	uint16_t blue_1_robot_HP;    //蓝1HP英雄
	uint16_t blue_2_robot_HP;    //蓝2HP工程
	uint16_t blue_3_robot_HP;    //蓝3HP步兵
	uint16_t blue_4_robot_HP;    //蓝4HP步兵
	uint16_t blue_5_robot_HP;    //蓝5HP步兵
	uint16_t blue_7_robot_HP;    //蓝7哨兵HP
	uint16_t blue_outpost_HP;    //蓝方前哨
	uint16_t blue_base_HP;	   //基地HP
}ext_game_robot_HP_t;

//飞镖发射状态：0x0004
typedef __packed struct
{
	uint8_t dart_belong;
	uint16_t stage_remaining_time;
} ext_dart_status_t;



//裁判警告信息cmd_id=0x0104
typedef __packed struct
{

	uint8_t level;               //警告等级
	uint8_t foul_robot_id;       //犯规机器人ID
}ext_referee_warning_t;
//机器人状态cmd_id=0x0201
typedef __packed struct
{

	uint8_t  robot_id;                      //机器人ID
	uint8_t  robot_level;                   //机器人等级
	uint16_t remain_HP;                    //机器人剩余血量
	uint16_t max_HP;                       //机器人上限血量
	uint16_t shooter_id1_17mm_cooling_rate;   //机器人17mm枪口每秒冷却值
	uint16_t shooter_id1_17mm_cooling_limit;  //机器人42mm枪口热量上限
	uint16_t shooter_id1_17mm_speed_limit;   //机器人17mm枪口每秒冷却值

	uint16_t shooter_id2_17mm_cooling_rate; //机器人42mm枪口热量上限
	uint16_t shooter_id2_17mm_cooling_limit;
	uint16_t shooter_id2_17mm_speed_limit;

	uint16_t shooter_id1_42mm_cooling_rate;
	uint16_t shooter_id1_42mm_cooling_limit;
	uint16_t shooter_id1_42mm_speed_limit;

	uint16_t chassis_power_limit;
	uint8_t  mains_power_gimbal_output : 1; //0 bit：gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出；
	uint8_t  mains_power_chassis_output : 1;//1 bit：chassis 口输出：1 为有 24V 输出，0 为无 24v 输出；
	uint8_t  mains_power_shooter_output : 1;//2 bit：shooter 口输出：1 为有 24V 输出，0 为无 24v 输出；
}ext_game_robot_status_t;

//实时射击信息0x207
typedef __packed struct
{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float bullet_speed;
}ext_shoot_data_t;


typedef enum
{
	Robot_Status_ID = 0x0201,//机器人状态  等级
	power_heat_data_ID = 0x0202,//枪口热量 底盘功率
	robot_hurt_ID = 0x0206,//伤害类型
	shoot_data_ID = 0x0207,//射频射速
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
}SelfDefineInfo_t;   //学生上传自定义数据 (0x0005) 

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

//实时功率数据cmd_id=0x0202
typedef __packed struct
{

	uint16_t chassis_volt;                //底盘输出电压 mv
	uint16_t chassis_current;             //底盘输出电流 ma
	float chassis_power;                  //底盘输出功率 w
	uint16_t chassis_power_buffer;        //底盘功率缓冲 j，飞坡需要250j
	uint16_t shooter_id1_17mm_cooling_heat;            //17mm枪口热量
	uint16_t shooter_id2_17mm_cooling_heat;              //42mm枪口热量
	uint16_t shooter_id1_42mm_cooling_heat;       //机动17mm枪口热量
}ext_power_heat_data_t;



typedef __packed struct
{
	float x;
	float y;
	float z;
	float yaw;
}ext_game_robot_pos_t;

//机器人增益
typedef __packed struct
{
	uint8_t power_rune_buff;
}ext_buff_t;


typedef struct
{
	tMsg_head                     judgedatahead;
	uint16_t                      rxCmdId;
	ext_game_status_t            game_status;           //比赛状态
	ext_game_robot_status_t      game_robot_state_t;    //机器人状态
	ext_game_robot_HP_t          game_robot_HP_t;       //机器人血量
	ext_power_heat_data_t        power_heat_data_t;     //机器人功率与热量
	ext_game_robot_pos_t         game_robot_position;   //机器人位置
	ext_shoot_data_t	         shoot_data_t;
	ext_referee_warning_t        referee_warning_t;     //裁判警告信息	
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

// 交互数据 机器人间通信： 0x0301。发送频率：上限 10Hz
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


///////////////////////////////////图形结构体
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
		ext_game_status_t            game_status;           //比赛状态
		ext_game_robot_status_t      game_robot_state_t;    //机器人状态
		ext_game_robot_HP_t          game_robot_HP_t;       //机器人血量
		ext_power_heat_data_t        power_heat_data_t;     //机器人功率与热量
		ext_game_robot_pos_t         game_robot_position;
		ext_shoot_data_t		 				 shoot_data_t;
		ext_referee_warning_t        referee_warning_t; 		//裁判警告信息
		robot_interactive_data_t     robot_data_t;
		client_custom_data_t 	userinfo;
		graphic_data_struct_t	graphic_data;
	}Data;
	uint16_t        CRC16;
	uint16_t        CRC16_2;
}FRAME;

/*--------------------------------接口结构体----------------------------------------------*/
//裁判系统数据结构
typedef struct
{
	uint8_t  robo_level;//机器人当前等级
	uint8_t  robo_ID;		//机器人当前ID
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
