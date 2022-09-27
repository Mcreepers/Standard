#include "Message_Task.h"
#include "Chassis_Task.h"
#include "Guard_Task.h"
#include "Correspondence_Task.h"
#include "queue.h"
#include "device.h"
 
Message_Data_t Message_Data;
Message_Ctrl Message;
Guard_Ctrl *Message_Guard;

extern void uart7_dma_get(void);
void Message_Task(void *pvParameters)
{
	while (1)
    {
        if (xQueueReceive(Message_Queue, &Message_Data, portMAX_DELAY))
		{
			Message.Hook();
			Message.Feed(&Message_Data.Data_ID);
		}
	}
}

void Message_Ctrl::Init()
{
	Message_Guard = get_guard_ctrl_pointer();
}

void Message_Ctrl::Feed(ID_e *ID)
{
	Message_Guard->Guard_Feed(ID);
}

//消息处理
void Message_Ctrl::Hook()
{
	switch (Message_Data.Data_ID)
	{
	case CanData1:
		CAN1_Hook((CanRxMsg *)Message_Data.Data_Ptr);
		break;
	case CanData2:
		CAN2_Hook((CanRxMsg *)Message_Data.Data_Ptr);
		break;
	case serial3:
		// Usart3_Hook();
		break;
	case serial6:
		Usart6_Hook();
		break;
	case serial7:
		Usart7_Hook();
		break;
	case serial8:
		// Usart8_Hook();
		break;
	case RC_ctrl:
	rc_key_v_fresh((RC_ctrl_t *)Message_Data.Data_Ptr);
		break;
	default:
		break;
	}
}

void Message_Ctrl::Usart3_Hook()
{

}

void Message_Ctrl::Usart6_Hook()
{
	ecd_data.s[0] = Usart6.Data[1];
	ecd_data.s[1] = Usart6.Data[2];
	GimbalR.ECD = -motor_ecd_to_relative_ecd(ecd_data.d, Gimbal_Motor_Yaw_Offset_ECD);
	GimbalR.goal = Usart6.Data[3];
	if (GimbalR.ECD > 8192)
	{
	}
	if (GimbalR.goal > 1)
	{
	}
}

void Message_Ctrl::Usart7_Hook()
{
	uart7_dma_get();
}

void Message_Ctrl::Usart8_Hook()
{

}

Message_Ctrl *get_message_ctrl_pointer(void)
{
	return &Message;
}

//统计按键 按下次数：eg:  按下-松开  按下-松开  2次
//key_num==1代表有键盘按下
//key_num==0代表键盘松开
void rc_key_c::sum_key_count(int16_t key_num, count_num_key *temp_count)
{
    if (key_num == 1 && temp_count->key_flag == 0)
    {
		temp_count->key_flag=1;
	}
    if (temp_count->key_flag == 1 && key_num == 0)
    {
        temp_count->count++;
		temp_count->key_flag=0;
	}
}

void rc_key_c::clear_key_count(count_num_key *temp_count)
{
    temp_count->count = 0;
	temp_count->key_flag=0;
}
//按键单点赋值
bool rc_key_c::read_key_single(count_num_key *temp_count,bool *temp_bool)
{
    if ((temp_count->count >= 1) && *temp_bool == 0)
    {
        temp_count->count = 0;
        *temp_bool = true;
    }
    else if ((temp_count->count >= 1) && *temp_bool == 1)
    {
        temp_count->count = 0;
        *temp_bool = false;
    }
	return *temp_bool;
}
//按键单点
bool rc_key_c::read_key_single(count_num_key *temp_count)
{
    if (temp_count->count >= 1)
    {
        temp_count->count = 0;
        return true;
    }else {
        temp_count->count = 0;
        return false;
    }
}
//按键长按赋值
bool rc_key_c::read_key_even(count_num_key *temp_count,bool *temp_bool)
{
	if (temp_count->key_flag == 1)
	{
		*temp_bool = true;
	}
	else if (temp_count->key_flag == 0)
	{
		*temp_bool = false;
	}
	return *temp_bool;
}
//按键长按
bool rc_key_c::read_key_even(count_num_key *temp_count)
{
    if (temp_count->key_flag == 1)
    {
        return true;
    }else{
        return false;
    }
}

uint8_t rc_key_c::read_key(count_num_key *temp_count, key_count_e mode, bool clear)
{
	uint8_t result;
	if (clear == true)
	{
		if (mode == single)
		{
			result=read_key_single(temp_count);
		}
		else if (mode == even)
		{
			result=read_key_even(temp_count);
		}
	}
	else
	{
		if (mode == single)
		{
			result = temp_count->count;
		}
		else if (mode == even)
		{
			result = temp_count->key_flag;
		}
	}
	return result;
}

bool rc_key_c::read_key(count_num_key *temp_count, key_count_e mode, bool *temp_bool)
{
	if (mode == single)
	{
		read_key_single(temp_count, temp_bool);
	}
	else if (mode == even)
	{
		read_key_even(temp_count, temp_bool);
	}
	return *temp_bool;
}

//更新按键
void rc_key_c::rc_key_v_set(RC_ctrl_t *RC)
{
	if (RC->key.v  &  KEY_PRESSED_OFFSET_W){
		sum_key_count(1,&Key.W);
	}else{
		sum_key_count(0,&Key.W);
	}
	if(RC->key.v & KEY_PRESSED_OFFSET_S){
		sum_key_count(1,&Key.S);
	}else{
		sum_key_count(0,&Key.S);
	}
	if(RC->key.v & KEY_PRESSED_OFFSET_A){
		sum_key_count(1,&Key.A);
	}else{
		sum_key_count(0,&Key.A);
	}
	if(RC->key.v & KEY_PRESSED_OFFSET_D){
		sum_key_count(1,&Key.D);
	}else{
		sum_key_count(0,&Key.D);
	}
	if(RC->key.v & KEY_PRESSED_OFFSET_SHIFT){
		sum_key_count(1,&Key.shift);
	}else{
		sum_key_count(0,&Key.shift);
	}
	if(RC->key.v & KEY_PRESSED_OFFSET_CTRL){
		sum_key_count(1,&Key.ctrl);
	}else{
		sum_key_count(0,&Key.ctrl);
	}
	if(RC->key.v & KEY_PRESSED_OFFSET_Q){
		sum_key_count(1,&Key.Q);
	}else{
		sum_key_count(0,&Key.Q);
	}
	if(RC->key.v & KEY_PRESSED_OFFSET_E){
		sum_key_count(1,&Key.E);
	}else{
		sum_key_count(0,&Key.E);
	}
	if(RC->key.v & KEY_PRESSED_OFFSET_R){
		sum_key_count(1,&Key.R);
	}else{
		sum_key_count(0,&Key.R);
	}
	if(RC->key.v & KEY_PRESSED_OFFSET_F){
		sum_key_count(1,&Key.F);
	}else{
		sum_key_count(0,&Key.F);
	}
	if(RC->key.v & KEY_PRESSED_OFFSET_G){
		sum_key_count(1,&Key.G);
	}else{
		sum_key_count(0,&Key.G);
	}
	if(RC->key.v & KEY_PRESSED_OFFSET_Z){
		sum_key_count(1,&Key.Z);
	}else{
		sum_key_count(0,&Key.Z);
	}
	if(RC->key.v & KEY_PRESSED_OFFSET_X){
		sum_key_count(1,&Key.X);
	}else{
		sum_key_count(0,&Key.X);
	}
	if(RC->key.v & KEY_PRESSED_OFFSET_C){
		sum_key_count(1,&Key.C);
	}else{
		sum_key_count(0,&Key.C);
	}
	if(RC->key.v & KEY_PRESSED_OFFSET_V){
		sum_key_count(1,&Key.V);
	}else{
		sum_key_count(0,&Key.V);
	}
	if(RC->key.v & KEY_PRESSED_OFFSET_B){
		sum_key_count(1,&Key.B);
	}else{
		sum_key_count(0,&Key.B);
	}
	//鼠标
	if (RC->mouse.press_l == 1){
		sum_key_count(1,&Press.L);
	}else{
		sum_key_count(0,&Press.L);
	}
	if(RC->mouse.press_r==1){
		sum_key_count(1,&Press.R);
	}else{
		sum_key_count(0,&Press.R);
	}
}
