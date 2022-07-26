#include "Message_Task.h"
#include "Chassis_Task.h"
#include "Guard_Task.h"
#include "Correspondence_Task.h"
#include "queue.h"
#include "device.h"

union I int_data;

Message_Data_t Message_Data;
Message_Ctrl Message;
ID_e Message_ID;
Gimbal_Data_t Gimbal;

extern void uart7_dma_get(void);
void Message_Task(void *pvParameters)
{
	while (1)
    {
        if (xQueueReceive(Message_Queue, &Message_ID, portMAX_DELAY))
		{
			Message.Hook(Message_Data.Data_Ptr);
			Guard.Guard_Feed(&Message_ID);
		}
	}
}
//消息处理
void Message_Ctrl::Hook(void *ptr)
{
	ptr=Message_Data.Data_Ptr;
	switch (Message_ID)
	{
	case CanData1:
		CAN1_Ctrl.Hook((CanRxMsg *)ptr);
		break;
	case CanData2:
		CAN2_Ctrl.Hook((CanRxMsg *)ptr);
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
		Chassis.rc_key_v_set((RC_ctrl_t *)ptr);
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
	uint8_t i;
	for (i = 0;i < Usart6.Len ;i++)
	{
		Usart6.Data[i] = Serial6.read();
	}

	int_data.s[0] = Usart6.Data[1];
	int_data.s[1] = Usart6.Data[2];
	Gimbal.ECD = -motor_ecd_to_relative_ecd(int_data.d, Gimbal_Motor_Yaw_Offset_ECD);
	Gimbal.gimbal_grade = Usart6.Data[3];
	Gimbal.compensation_state = Usart6.Data[4];
	Gimbal.follow_on = Usart6.Data[5];
	Gimbal.goal = Usart6.Data[6];
	Gimbal.energy_state = Usart6.Data[7];
	//	Gimbal.predict = Usart6.Data[8];
	if (Gimbal.ECD > 8192 || Gimbal.gimbal_grade > 3)
	{
		Error_Flag.Gimbal = 1;
	}
	if (Gimbal.compensation_state > 2 || Gimbal.follow_on > 1 || Gimbal.energy_state > 1
		|| Gimbal.goal > 1|| Gimbal.predict > 1)
	{
		Error_Flag.Visual = 1;
	}
}

void Message_Ctrl::Usart7_Hook()
{
}

void Message_Ctrl::Usart8_Hook()
{

}

const Gimbal_Data_t *get_gimbal_data_point(void)
{
	return &Gimbal;
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
    if ((temp_count->key_flag == 1)&&*temp_bool==0)
    {
        *temp_bool = true;
    }
    else if ((temp_count->key_flag == 1)&&*temp_bool==1)
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

bool rc_key_c::read_key(count_num_key *temp_count, key_count_e mode)
{
	bool result;
	if (mode == single)
	{
		result=read_key_single(temp_count);
	}
	else if (mode == even)
	{
		result=read_key_even(temp_count);
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
	if (RC->key.v == KEY_PRESSED_OFFSET_W){
		sum_key_count(1,&Key.W);
	}else{
		sum_key_count(0,&Key.W);
	}
	if(RC->key.v==KEY_PRESSED_OFFSET_S){
		sum_key_count(1,&Key.S);
	}else{
		sum_key_count(0,&Key.S);
	}
	if(RC->key.v==KEY_PRESSED_OFFSET_A){
		sum_key_count(1,&Key.A);
	}else{
		sum_key_count(0,&Key.A);
	}
	if(RC->key.v==KEY_PRESSED_OFFSET_D){
		sum_key_count(1,&Key.D);
	}else{
		sum_key_count(0,&Key.D);
	}
	if(RC->key.v==KEY_PRESSED_OFFSET_SHIFT){
		sum_key_count(1,&Key.shift);
	}else{
		sum_key_count(0,&Key.shift);
	}
	if(RC->key.v==KEY_PRESSED_OFFSET_CTRL){
		sum_key_count(1,&Key.ctrl);
	}else{
		sum_key_count(0,&Key.ctrl);
	}
	if(RC->key.v==KEY_PRESSED_OFFSET_Q){
		sum_key_count(1,&Key.Q);
	}else{
		sum_key_count(0,&Key.Q);
	}
	if(RC->key.v==KEY_PRESSED_OFFSET_E){
		sum_key_count(1,&Key.E);
	}else{
		sum_key_count(0,&Key.E);
	}
	if(RC->key.v==KEY_PRESSED_OFFSET_R){
		sum_key_count(1,&Key.R);
	}else{
		sum_key_count(0,&Key.R);
	}
	if(RC->key.v==KEY_PRESSED_OFFSET_F){
		sum_key_count(1,&Key.F);
	}else{
		sum_key_count(0,&Key.F);
	}
	if(RC->key.v==KEY_PRESSED_OFFSET_G){
		sum_key_count(1,&Key.G);
	}else{
		sum_key_count(0,&Key.G);
	}
	if(RC->key.v==KEY_PRESSED_OFFSET_Z){
		sum_key_count(1,&Key.Z);
	}else{
		sum_key_count(0,&Key.Z);
	}
	if(RC->key.v==KEY_PRESSED_OFFSET_X){
		sum_key_count(1,&Key.X);
	}else{
		sum_key_count(0,&Key.X);
	}
	if(RC->key.v==KEY_PRESSED_OFFSET_C){
		sum_key_count(1,&Key.C);
	}else{
		sum_key_count(0,&Key.C);
	}
	if(RC->key.v==KEY_PRESSED_OFFSET_V){
		sum_key_count(1,&Key.V);
	}else{
		sum_key_count(0,&Key.V);
	}
	if(RC->key.v==KEY_PRESSED_OFFSET_B){
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
