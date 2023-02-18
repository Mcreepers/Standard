#include "Message_Task.h"
#include "Chassis_Task.h"
#include "Guard_Task.h"
#include "Correspondence_Task.h"
#include "queue.h"
#include "device.h"

Message_Ctrl Message;
Guard_Ctrl *Message_Guard;

extern void uart7_dma_get(void);

void Message_Task(void *pvParameters)
{
	Message.Init();
	while(1)
	{
		if(xQueueReceive(Message_Queue, &ID_Data[MessageData], portMAX_DELAY))
		{
			Message.Hook();
			Message.Feed(&ID_Data[MessageData].Data_ID);
		}
	}
}

void Message_Ctrl::Init()
{
	Message_Guard = get_guard_ctrl_pointer();
	robo = get_robo_data_Point();
}

void Message_Ctrl::Feed(ID_e *ID)
{
	Message_Guard->Feed(*ID);
}

//消息处理
void Message_Ctrl::Hook()
{
	switch(ID_Data[MessageData].Data_ID)
	{
	case CanData1:
	CAN1_Process((CanRxMsg *)ID_Data[MessageData].Data_Ptr);
	break;
	case CanData2:
	CAN2_Process((CanRxMsg *)ID_Data[MessageData].Data_Ptr);
	break;
	case SerialData3:
	Usart3_Hook((uint8_t *)ID_Data[MessageData].Data_Ptr);
	break;
	case SerialData6:
	Usart6_Hook((uint8_t *)ID_Data[MessageData].Data_Ptr);
	break;
	case SerialData7:
	Usart7_Hook((uint8_t *)ID_Data[MessageData].Data_Ptr);
	break;
	case SerialData8:
	Usart8_Hook((uint8_t *)ID_Data[MessageData].Data_Ptr);
	break;
	case RC_Data:
	rc_key_v_fresh((RC_ctrl_t *)ID_Data[MessageData].Data_Ptr);
	break;
	default:
	break;
	}
}

void Message_Ctrl::CAN1_Process(CanRxMsg *Rx_Message)
{
	switch(Rx_Message->StdId)
	{
	case CAN_CAP_GET_ID:
	{
		SuperCapR.situation = (uint8_t)((Rx_Message)->Data[0]);
		SuperCapR.mode = (uint8_t)((Rx_Message)->Data[1]);
		SuperCapR.power = (float)((uint16_t)(((Rx_Message)->Data[2]) | ((Rx_Message)->Data[3]) << 8)) * 0.1f;
		SuperCapR.energy = (uint8_t)((Rx_Message)->Data[4]);
		SuperCapR.power_limit = (uint8_t)((Rx_Message)->Data[5]);
		SuperCapR.errorcode = (uint8_t)((Rx_Message)->Data[6]);
		// SuperCapR.enable = (uint8_t)((Rx_Message)->Data[7]);
		// SuperCapR.enable = (uint8_t)((Rx_Message)->Data[8]);
	}
	case CAN_DJI_Motor1_ID:
	case CAN_DJI_Motor2_ID:
	case CAN_DJI_Motor3_ID:
	case CAN_DJI_Motor4_ID:
	{
		static uint8_t i = 0;
		//处理电机ID号
		i = Rx_Message->StdId - CAN_DJI_Motor1_ID;
		//处理电机数据宏函数
		get_motor_measure(CAN_Cmd.Chassis->GetData(i), Rx_Message);
		break;
	}
	default:
	break;
	}
}

void Message_Ctrl::CAN2_Process(CanRxMsg *Rx_Message)
{
	switch(Rx_Message->StdId)
	{
#ifdef useMecanum
	case CAN_YAW_MOTOR_ID:
	{
		//处理电机数据宏函数
		get_gimbal_motor_measuer(&CAN_Cmd.Gimbal.Yaw_Measure, Rx_Message);
		break;
	}
	case CAN_FRIC_MOTOR_ID:
	{
		get_motor_measure(&CAN_Cmd.Fric.Fric_Measure, Rx_Message);//底盘大弹丸拨弹轮电机
		break;
	}
#endif
#ifdef useSteering
	case CAN_DJI_Motor5_ID:
	case CAN_DJI_Motor6_ID:
	case CAN_DJI_Motor7_ID:
	case CAN_DJI_Motor8_ID:
	{
		static uint8_t i = 0;
		//处理电机ID号
		i = Rx_Message->StdId - CAN_DJI_Motor5_ID;
		//处理电机数据宏函数
		get_motor_measure(CAN_Cmd.Gimbal->GetData(i), Rx_Message);
		break;
	}
#endif
	default:
	break;
	}
}

void Message_Ctrl::Usart3_Hook(uint8_t *Rx_Message)
{
	uint8_t len = Rx_Message[0];
//	Message.comm.ReceiveData(&Rx_Message[1], len);
}

void Message_Ctrl::Usart6_Hook(uint8_t *Rx_Message)
{
	uint8_t len = Rx_Message[0];
	ecd_data.s[0] = Rx_Message[2];
	ecd_data.s[1] = Rx_Message[3];
	GimbalR.ECD = -motor_ecd_to_relative_ecd(ecd_data.d, Gimbal_Motor_Yaw_Offset_ECD);
	GimbalR.goal = Rx_Message[4];
	if(GimbalR.ECD > 8192)
	{
	}
	if(GimbalR.goal > 1)
	{
	}
}

void Message_Ctrl::Usart7_Hook(uint8_t *Rx_Message)
{
	uart7_dma_get();
	if(robo->game_robot_state.robot_id % 100 == RobotID)
	{//%100忽视红蓝
		Message_Guard->Feed(RobotId);
	}
}

void Message_Ctrl::Usart8_Hook(uint8_t *Rx_Message)
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
	if(key_num == 1 && temp_count->key_flag == 0)
	{
		temp_count->key_flag = 1;
	}
	if(temp_count->key_flag == 1 && key_num == 0)
	{
		temp_count->count++;
		temp_count->key_flag = 0;
	}
}

void rc_key_c::clear_key_count(count_num_key *temp_count)
{
	temp_count->count = 0;
	temp_count->key_flag = 0;
}
//按键单点赋值
bool rc_key_c::read_key_single(count_num_key *temp_count, bool *temp_bool)
{
	if((temp_count->count >= 1) && *temp_bool == 0)
	{
		temp_count->count = 0;
		*temp_bool = true;
	}
	else if((temp_count->count >= 1) && *temp_bool == 1)
	{
		temp_count->count = 0;
		*temp_bool = false;
	}
	return *temp_bool;
}
//按键单点
bool rc_key_c::read_key_single(count_num_key *temp_count)
{
	if(temp_count->count >= 1)
	{
		temp_count->count = 0;
		return true;
	}
	else
	{
		temp_count->count = 0;
		return false;
	}
}
//按键长按赋值
bool rc_key_c::read_key_even(count_num_key *temp_count, bool *temp_bool)
{
	if(temp_count->key_flag == 1)
	{
		*temp_bool = true;
	}
	else if(temp_count->key_flag == 0)
	{
		*temp_bool = false;
	}
	return *temp_bool;
}
//按键长按
bool rc_key_c::read_key_even(count_num_key *temp_count)
{
	if(temp_count->key_flag == 1)
	{
		return true;
	}
	else
	{
		return false;
	}
}

uint8_t rc_key_c::read_key(count_num_key *temp_count, key_count_e mode, bool clear)
{
	uint8_t result;
	if(clear == true)
	{
		if(mode == single)
		{
			result = read_key_single(temp_count);
		}
		else if(mode == even)
		{
			result = read_key_even(temp_count);
		}
	}
	else
	{
		if(mode == single)
		{
			result = temp_count->count;
		}
		else if(mode == even)
		{
			result = temp_count->key_flag;
		}
	}
	return result;
}

bool rc_key_c::read_key(count_num_key *temp_count, key_count_e mode, bool *temp_bool)
{
	if(mode == single)
	{
		read_key_single(temp_count, temp_bool);
	}
	else if(mode == even)
	{
		read_key_even(temp_count, temp_bool);
	}
	return *temp_bool;
}

//更新按键
void rc_key_c::rc_key_v_set(RC_ctrl_t *RC)
{
	count_num_key *p = &Key.W;
	for(uint8_t i = 0; i < 16; i++)
	{
		if(RC->key.v & ((uint16_t)1 << i))
		{
			sum_key_count(1, (p + i));
		}
		else
		{
			sum_key_count(0, (p + i));
		}
	}

	// if(RC->key.v & KEY_PRESSED_OFFSET_W)
	// {
	// 	sum_key_count(1,&Key.W);
	// }else{
	// 	sum_key_count(0,&Key.W);
	// }
	// if(RC->key.v & KEY_PRESSED_OFFSET_S){
	// 	sum_key_count(1,&Key.S);
	// }else{
	// 	sum_key_count(0,&Key.S);
	// }
	// if(RC->key.v & KEY_PRESSED_OFFSET_A){
	// 	sum_key_count(1,&Key.A);
	// }else{
	// 	sum_key_count(0,&Key.A);
	// }
	// if(RC->key.v & KEY_PRESSED_OFFSET_D){
	// 	sum_key_count(1,&Key.D);
	// }else{
	// 	sum_key_count(0,&Key.D);
	// }
	// if(RC->key.v & KEY_PRESSED_OFFSET_SHIFT){
	// 	sum_key_count(1,&Key.shift);
	// }else{
	// 	sum_key_count(0,&Key.shift);
	// }
	// if(RC->key.v & KEY_PRESSED_OFFSET_CTRL){
	// 	sum_key_count(1,&Key.ctrl);
	// }else{
	// 	sum_key_count(0,&Key.ctrl);
	// }
	// if(RC->key.v & KEY_PRESSED_OFFSET_Q){
	// 	sum_key_count(1,&Key.Q);
	// }else{
	// 	sum_key_count(0,&Key.Q);
	// }
	// if(RC->key.v & KEY_PRESSED_OFFSET_E){
	// 	sum_key_count(1,&Key.E);
	// }else{
	// 	sum_key_count(0,&Key.E);
	// }
	// if(RC->key.v & KEY_PRESSED_OFFSET_R){
	// 	sum_key_count(1,&Key.R);
	// }else{
	// 	sum_key_count(0,&Key.R);
	// }
	// if(RC->key.v & KEY_PRESSED_OFFSET_F){
	// 	sum_key_count(1,&Key.F);
	// }else{
	// 	sum_key_count(0,&Key.F);
	// }
	// if(RC->key.v & KEY_PRESSED_OFFSET_G){
	// 	sum_key_count(1,&Key.G);
	// }else{
	// 	sum_key_count(0,&Key.G);
	// }
	// if(RC->key.v & KEY_PRESSED_OFFSET_Z){
	// 	sum_key_count(1,&Key.Z);
	// }else{
	// 	sum_key_count(0,&Key.Z);
	// }
	// if(RC->key.v & KEY_PRESSED_OFFSET_X){
	// 	sum_key_count(1,&Key.X);
	// }else{
	// 	sum_key_count(0,&Key.X);
	// }
	// if(RC->key.v & KEY_PRESSED_OFFSET_C){
	// 	sum_key_count(1,&Key.C);
	// }else{
	// 	sum_key_count(0,&Key.C);
	// }
	// if(RC->key.v & KEY_PRESSED_OFFSET_V){
	// 	sum_key_count(1,&Key.V);
	// }else{
	// 	sum_key_count(0,&Key.V);
	// }
	// if(RC->key.v & KEY_PRESSED_OFFSET_B){
	// 	sum_key_count(1,&Key.B);
	// }else{
	// 	sum_key_count(0,&Key.B);
	// }
	//鼠标
	if(RC->mouse.press_l == 1)
	{
		sum_key_count(1, &Press.L);
	}
	else
	{
		sum_key_count(0, &Press.L);
	}
	if(RC->mouse.press_r == 1)
	{
		sum_key_count(1, &Press.R);
	}
	else
	{
		sum_key_count(0, &Press.R);
	}
}
