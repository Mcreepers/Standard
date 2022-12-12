#include "app_motor.h"
#include "queue.h"
#include "Message_Task.h"
CAN_Ctrl CAN_Cmd;

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
	switch (Rx_Message->StdId)
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
#endif
	default:
		break;
	}
}

void CAN1_Send(CanRxMsg *Rx_Message)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	ID_Data[CanData1].Data_Ptr = Rx_Message;
	xQueueSendFromISR(Message_Queue, &ID_Data[CanData1], &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void CAN2_Send(CanRxMsg *Rx_Message)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	ID_Data[CanData2].Data_Ptr = Rx_Message;
	xQueueSendFromISR(Message_Queue, &ID_Data[CanData2], &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void CAN_ALL_Init(void)
{
	//必须先初始化CAN1，在初始化CAN2
	CAN1_Ctrl.CANx_Init();
	CAN2_Ctrl.CANx_Init();
	
	CAN1_Ctrl.attachInterrupt(CAN1_Send);
	CAN2_Ctrl.attachInterrupt(CAN2_Send);
}
