#ifndef Message_TASK_H
#define Message_TASK_H

#include "app_serial.h"

#include "queue.h"
#include "protocol_dbus.h"
#include "protocol_judgement.h"

#ifdef __cplusplus
extern "C" {
#endif

	void Message_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

//无效，信息任务为最高优先级任务，运行条件为消息队列中有消息
#define Message_CONTROL_TIME_MS 1

#define Message_Q_NUM    40  		    //消息队列的数量
extern QueueHandle_t Message_Queue;   		//消息队列句柄

#define RX_BUF_NUM   1000u

#define CAP_CHECK 0x00
#define CAP_CLOSE 0x0f
#define CAP_OPEN 0xf0
#define CAP_ERROR 0xff
#define CAP_MODE1 0x00
#define CAP_MODE2 0xff

union I
{
	char s[2];
	uint16_t d;
};

struct Gimbal_Receive_Data_t
{
	fp32 ECD;
	uint8_t goal;
};

struct supercap_Receive_Data_t
{
  uint8_t situation;//0x00自检 0x0f关闭 0xf0开启 0xff错误
  uint8_t mode;//0x00模式1 0xff模式2
  fp32 power;
  uint8_t energy;
  uint8_t power_limit;
  uint8_t errorcode;
  
  uint8_t *ptr;
};

class Message_Ctrl
{
public:
	Gimbal_Receive_Data_t GimbalR;
	supercap_Receive_Data_t SuperCapR;
	const judge_type_t *robo;

	union I ecd_data;

	void Hook();
	void Init();
	void Feed(ID_e *Name);
private:
	void *ptr;
	void CAN1_Process(CanRxMsg *Rx_Message);
	void CAN2_Process(CanRxMsg *Rx_Message);
	void Usart3_Hook(Serial_Data_t *Rx_Message);
	void Usart6_Hook(Serial_Data_t *Rx_Message);
	void Usart7_Hook(Serial_Data_t *Rx_Message);
	void Usart8_Hook(Serial_Data_t *Rx_Message);
};

Message_Ctrl * get_message_ctrl_pointer(void);

typedef struct
{
	u8	key_flag;
	u8  count;//次数
	u8  last_count;
}count_num_key;

typedef enum
{
	single = 0,
	even,
}key_count_e;

struct rc_key_v_t
{
	//键盘
	count_num_key W;
	count_num_key S;
	count_num_key A;
	count_num_key D;
	count_num_key shift;
	count_num_key ctrl;
	count_num_key Q;
	count_num_key E;
	count_num_key R;
	count_num_key F;
	count_num_key G;
	count_num_key Z;
	count_num_key X;
	count_num_key C;
	count_num_key V;
	count_num_key B;
};

struct rc_press_t
{
	//鼠标
	count_num_key L;
	count_num_key R;
};

class rc_key_c
{
public:
	rc_key_v_t Key;
	rc_press_t Press;
	
	void rc_key_v_set(RC_ctrl_t *RC);
	uint8_t read_key(count_num_key *temp_count, key_count_e mode,bool clear);
	bool read_key(count_num_key *temp_count, key_count_e mode, bool *temp_bool);
	void clear_key_count(count_num_key *temp_count);
private:
	bool read_key_single(count_num_key *temp_count);
	bool read_key_single(count_num_key *temp_count, bool *temp_bool);
	bool read_key_even(count_num_key *temp_count);
	bool read_key_even(count_num_key *temp_count, bool *temp_bool);
	void sum_key_count(int16_t key_num, count_num_key *temp_count);
};

#endif
