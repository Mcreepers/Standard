#include "Message_Task.h"
#include "Chassis_Task.h"
#include "Guard_Task.h"
#include "queue.h"
#include "device.h"

union I ecd_data;
union I int_data;
union F float_data;

Usart_Data_t Usart3( Serial3_Buffer_Size, Serial3_Data_Header, Serial3_Data_tail );
Usart_Data_t Usart6( Serial6_Buffer_Size, Serial6_Data_Header, Serial6_Data_tail );
Usart_Data_t Usart7( Serial7_Buffer_Size, Serial7_Data_Header, Serial7_Data_tail );
Usart_Data_t Usart8( Serial8_Buffer_Size, Serial8_Data_Header, Serial8_Data_tail );

Message_Data_t Message_Data;
Message_Ctrl Message;
extern QueueHandle_t Message_Queue;   		//消息队列句柄
ID_t Message_ID;
Gimbal_Data_t Gimbal;
rc_key_v_t Key;
rc_press_t Press;
void Message_Task(void *pvParameters)
{
    
    while (1)
    {
        if (xQueueReceive(Message_Queue, &Message_ID, portMAX_DELAY))
        {
            switch (Message_ID)
			{
#if useSteering
			case CanData1:
				CAN1_Ctrl.Hook((CanRxMsg *)Message_Data.Data_Ptr);
				break;
#endif
			case CanData2:
				CAN2_Ctrl.Hook((CanRxMsg *)Message_Data.Data_Ptr);
                break;
            case serial3:
//                Usart3_hook((Usart_Data_t *)Message_Data.Data_Ptr);
                break;
            case serial6:
                memcpy(&Usart6.Data, &Serial6.usart->Data, sizeof(Serial6.usart->Data));
                Usart6_hook((Usart_Data_t *)Message_Data.Data_Ptr);
                break;
            case serial7:
                // Usart7_hook((Usart_Data_t *)Message_Data.Data_Ptr);
                break;
            case serial8:
                // Usart8_hook((Usart_Data_t *)Message_Data.Data_Ptr);
                break;
            case RC_ctrl:
                rc_key_v_set();
                break;
            default:
                break;
            }
		}
	}
}

void Usart3_hook(void)
{
	Guard.Guard_Feed(&Message_ID);

}

void Usart6_hook(Usart_Data_t *Usart6)
{
	int_data.s[0] = Usart6->Data[1];
	int_data.s[1] = Usart6->Data[2];
	Gimbal.ECD = Chassis.motor_angle_to_set_change(int_data.d,940);
	Gimbal.gimbal_grade = Usart6->Data[3];
	Gimbal.compensation_state = Usart6->Data[4];
	Gimbal.follow_on = Usart6->Data[5];
	Gimbal.goal = Usart6->Data[6];
	Gimbal.energy_state = Usart6->Data[7];
//	Gimbal.predict = Usart6->Data[8];
	Guard.Guard_Feed(&Message_ID);
}

void Usart7_hook(void)
{
	
	Guard.Guard_Feed(&Message_ID);

}

void Usart8_hook(void)
{
	Guard.Guard_Feed(&Message_ID);

}
const float *get_yaw_motor_point(void)
{
	return &Gimbal.ECD;
}

extern QueueHandle_t Chassis_Queue;
void sum_key_count(int16_t key_num, count_num_key *temp_count);
void clear_key_count(count_num_key *temp_count);

//统计按键 按下次数：eg:  按下-松开  按下-松开  2次
//key_num==1代表有键盘按下
//key_num==0代表键盘松开
void sum_key_count(int16_t key_num, count_num_key *temp_count)
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

void clear_key_count(count_num_key *temp_count)
{
    temp_count->count = 0;
	temp_count->key_flag=0;
}
//返回按键状态0没按 1单点 2长按
uint8_t read_key_count(count_num_key *temp_count)
{
    if (temp_count->count)//待完善
    {
        temp_count->count = 0;
        return 1;
    }
    else if (temp_count->key_flag == 1)
    {
        return 2;
    }
    else return 0;
}
//按键单点赋值
void read_key_single(count_num_key *temp_count,bool *temp_bool)
{
    if ((temp_count->count >= 1) && *temp_bool == 0)
    {
        temp_count->count = 0;
        *temp_bool = true;
    }
    else if ((temp_count->count == 0) && *temp_bool == 1)
    {
        *temp_bool = false;
    }
}
//按键长按赋值
void read_key_even(count_num_key *temp_count,bool *temp_bool)
{
    if ((temp_count->key_flag == 1)&&*temp_bool==0)
    {
        *temp_bool = true;
    }
    else if ((temp_count->key_flag == 1)&&*temp_bool==1)
    {
        *temp_bool = false;
    }
}

//更新按键
void rc_key_v_set(void)
{
	if(rc_ctrl.key.v==KEY_PRESSED_OFFSET_W){
		sum_key_count(1,&Key.W);
	}else{
		sum_key_count(0,&Key.W);
	}
	if(rc_ctrl.key.v==KEY_PRESSED_OFFSET_S){
		sum_key_count(1,&Key.S);
	}else{
		sum_key_count(0,&Key.S);
	}
	if(rc_ctrl.key.v==KEY_PRESSED_OFFSET_A){
		sum_key_count(1,&Key.A);
	}else{
		sum_key_count(0,&Key.A);
	}
	if(rc_ctrl.key.v==KEY_PRESSED_OFFSET_D){
		sum_key_count(1,&Key.D);
	}else{
		sum_key_count(0,&Key.D);
	}
	if(rc_ctrl.key.v==KEY_PRESSED_OFFSET_SHIFT){
		sum_key_count(1,&Key.shift);
	}else{
		sum_key_count(0,&Key.shift);
	}
	if(rc_ctrl.key.v==KEY_PRESSED_OFFSET_CTRL){
		sum_key_count(1,&Key.ctrl);
	}else{
		sum_key_count(0,&Key.ctrl);
	}
	if(rc_ctrl.key.v==KEY_PRESSED_OFFSET_Q){
		sum_key_count(1,&Key.Q);
	}else{
		sum_key_count(0,&Key.Q);
	}
	if(rc_ctrl.key.v==KEY_PRESSED_OFFSET_E){
		sum_key_count(1,&Key.E);
	}else{
		sum_key_count(0,&Key.E);
	}
	if(rc_ctrl.key.v==KEY_PRESSED_OFFSET_R){
		sum_key_count(1,&Key.R);
	}else{
		sum_key_count(0,&Key.R);
	}
	if(rc_ctrl.key.v==KEY_PRESSED_OFFSET_F){
		sum_key_count(1,&Key.F);
	}else{
		sum_key_count(0,&Key.F);
	}
	if(rc_ctrl.key.v==KEY_PRESSED_OFFSET_G){
		sum_key_count(1,&Key.G);
	}else{
		sum_key_count(0,&Key.G);
	}
	if(rc_ctrl.key.v==KEY_PRESSED_OFFSET_Z){
		sum_key_count(1,&Key.Z);
	}else{
		sum_key_count(0,&Key.Z);
	}
	if(rc_ctrl.key.v==KEY_PRESSED_OFFSET_X){
		sum_key_count(1,&Key.X);
	}else{
		sum_key_count(0,&Key.X);
	}
	if(rc_ctrl.key.v==KEY_PRESSED_OFFSET_C){
		sum_key_count(1,&Key.C);
	}else{
		sum_key_count(0,&Key.C);
	}
	if(rc_ctrl.key.v==KEY_PRESSED_OFFSET_V){
		sum_key_count(1,&Key.V);
	}else{
		sum_key_count(0,&Key.V);
	}
	if(rc_ctrl.key.v==KEY_PRESSED_OFFSET_B){
		sum_key_count(1,&Key.B);
	}else{
		sum_key_count(0,&Key.B);
	}
	//鼠标按键
	if (rc_ctrl.mouse.press_l == 1){
		sum_key_count(1,&Press.L);
	}else{
		sum_key_count(0,&Press.L);
	}
	if(rc_ctrl.mouse.press_r==1){
		sum_key_count(1,&Press.R);
	}else{
		sum_key_count(0,&Press.R);
	}

	Guard.Guard_Feed(&Message_ID);
}
