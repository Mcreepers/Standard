#ifndef Message_TASK_H
#define Message_TASK_H

#include "dev_system.h"

#include "queue.h"
// #include "dev_serial.h"
// #include <vector>

#ifdef __cplusplus
extern "C" {
#endif

void Message_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#define Message_CONTROL_TIME_MS 1

#define Message_Q_NUM    1  		    //消息队列的数量
extern QueueHandle_t Message_Queue;   		//消息队列句柄

union I
{
    char s[2];
    uint16_t d;
};
union F 
{
    char s[8];
    float d;
};

typedef enum
{
	CanData1 = 0x00,
	CanData2,
	serial3,
	serial6,
	serial7,
	serial8,
	RC_ctrl,
	chassis,
	UIdraw,
	ID_t_count
}ID_t;

struct Message_Data_t
{
	ID_t Data_ID;
	uint32_t DataValue;
	void *Data_Ptr;
	Message_Data_t(){}
};

struct Gimbal_Data_t
{
	fp32 ECD;
	uint8_t gimbal_grade;
	uint8_t compensation_state;
	uint8_t follow_on;
	uint8_t goal;
	uint8_t energy_state;
	uint8_t predict;
};

typedef struct
{
	u8	key_flag;
	u8  count;//次数
    u8  last_count;
}count_num_key;

struct rc_key_v_t
{
	//按键
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
	//按键
	count_num_key L;
	count_num_key R;
};

class Message_Ctrl
{
public:
	Message_Ctrl(){
		Data = new Message_Data_t();
	}
	Message_Data_t *Data;
};

extern Message_Data_t Message_Data;
extern Message_Ctrl Message;
extern rc_key_v_t Key;
extern rc_press_t Press;
extern Gimbal_Data_t Gimbal;
extern void rc_key_v_set(void);

const float *get_yaw_motor_point(void);
// const GIMBAL_DADTA_T *get_gimbal_data_point(void);
void sum_key_count(int16_t key_num, count_num_key *temp_count);
void clear_key_count(count_num_key *temp_count);
uint8_t read_key_count(count_num_key *temp_count);
void read_key_single(count_num_key *temp_count,bool *temp_bool);
void read_key_even(count_num_key *temp_count, bool *temp_bool);
void Usart3_hook(void);
void Usart6_hook(void);
void Usart7_hook(void);
void Usart8_hook(void);

#endif
