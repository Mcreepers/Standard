#include "app_motor.h"
#include "queue.h"
#include "Guard_Task.h"
CAN_Ctrl CAN_Cmd;

supercap_measure_t SuperCap;
Message_Data_t Message_Data_Can[2];

void CAN2_Hook(CanRxMsg *Rx_Message)
{
	switch (Rx_Message->StdId)
	{
	case CAN_3508_M1_ID:
	case CAN_3508_M2_ID:
	case CAN_3508_M3_ID:
	case CAN_3508_M4_ID:
	{
		static uint8_t i = 0;
		//处理电机ID号
		i = Rx_Message->StdId - CAN_3508_M1_ID;
		//处理电机数据宏函数
		get_motor_measure(&CAN_Cmd.Chassis.Chassis_Measure[i], Rx_Message);
		break;
	}
	default:
	{
		break;
	}
	}
}

void CAN1_Hook(CanRxMsg *Rx_Message)
{
#if useMecanum
	switch (Rx_Message->StdId)
	{
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
	default:
	{
		break;
	}
	}
#elif useSteering
	switch (Rx_Message->StdId)
	{
	case CAN_6020_M1_ID:
	case CAN_6020_M2_ID:
	case CAN_6020_M3_ID:
	case CAN_6020_M4_ID:
	{
		static uint8_t i = 0;
		//处理电机ID号
		i = Rx_Message->StdId - CAN_6020_M1_ID;
		//处理电机数据宏函数
		get_motor_measure(&CAN_Cmd.Gimbal.Steering_Measure[i], Rx_Message);
		break;
	}
	case 0x301:
	{
		SuperCap.ptr = (uint8_t *)&SuperCap.supercap_voltage;
		SuperCap.ptr[0] = (uint8_t)((Rx_Message)->Data[0]);
		SuperCap.ptr[1] = (uint8_t)((Rx_Message)->Data[1]);
		SuperCap.ptr[2] = (uint8_t)((Rx_Message)->Data[2]);
		SuperCap.ptr[3] = (uint8_t)((Rx_Message)->Data[3]);
		SuperCap.ptr = (uint8_t *)&SuperCap.supercap_energy_percent;
		SuperCap.ptr[0] = (uint8_t)((Rx_Message)->Data[4]);
		SuperCap.ptr[1] = (uint8_t)((Rx_Message)->Data[5]);
		SuperCap.ptr[2] = (uint8_t)((Rx_Message)->Data[6]);
		SuperCap.ptr[3] = (uint8_t)((Rx_Message)->Data[7]);
		break;
	}
	default:
	{
		break;
	}
	}
#endif
}

void CAN1_Send(CanRxMsg *Rx_Message)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	Message_Data_Can[0].Data_Ptr = Rx_Message;
	xQueueSendFromISR(Message_Queue, &Message_Data_Can[0], &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void CAN2_Send(CanRxMsg *Rx_Message)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	Message_Data_Can[1].Data_Ptr = Rx_Message;
	xQueueSendFromISR(Message_Queue, &Message_Data_Can[1], &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void CAN_ALL_Init(void)
{
	//必须先初始化CAN1，在初始化CAN2
	CAN1_Ctrl.CANx_Init();
	CAN2_Ctrl.CANx_Init();

	Message_Data_Can[0].Data_ID = CanData1;
	Message_Data_Can[1].Data_ID = CanData2;
	
	CAN1_Ctrl.attachInterrupt(CAN1_Send);
	CAN2_Ctrl.attachInterrupt(CAN2_Send);
}
