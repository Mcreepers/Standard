#include "app_motor.h"
#include "queue.h"
#include "Message_Task.h"
CAN_Ctrl CAN_Cmd;

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
	//必须先初始化CAN1，再初始化CAN2
	CAN1_Ctrl.CANx_Init();
	CAN2_Ctrl.CANx_Init();

	CAN1_Ctrl.attachInterrupt(CAN1_Send);
	CAN2_Ctrl.attachInterrupt(CAN2_Send);
}

void CAN_Ctrl::SendData(CANctrl *CANx_Ctrl, uint32_t StdID, void *buf, uint8_t len)
{
	CANx_Ctrl->ChangeID(StdID);
	CANx_Ctrl->SendData(buf, len);
}

void CAN_Ctrl::SendData(CAN_TypeDef *CANx, uint32_t StdID, void *buf, uint8_t len)
{
	if(CANx == CAN1)
	{
		SendData(&CAN1_Ctrl, StdID, buf, len);
	}
	if(CANx == CAN2)
	{
		SendData(&CAN2_Ctrl, StdID, buf, len);
	}
}

void CAN_Ctrl::SendData(Motor_CAN_Ctrl *Motor, int16_t Motor1, int16_t Motor2, int16_t Motor3, int16_t Motor4)
{
	CAN_TypeDef *CANx;
	uint32_t StdID;
	uint8_t Num;
	uint8_t Data[8];
	Data[0] = Motor1 >> 8;
	Data[1] = Motor1;
	Data[2] = Motor2 >> 8;
	Data[3] = Motor2;
	Data[4] = Motor3 >> 8;
	Data[5] = Motor3;
	Data[6] = Motor4 >> 8;
	Data[7] = Motor4;

	Motor->GetData(CANx, StdID, Num);
	SendData(CANx, StdID, Data, 8);
}

void CAN_Ctrl::SendData(Motor_CAN_Ctrl *Motor, int16_t Motor1, int16_t Motor2, int16_t Motor3)
{
	CAN_TypeDef *CANx;
	uint32_t StdID;
	uint8_t Num;
	uint8_t Data[6];
	Data[0] = Motor1 >> 8;
	Data[1] = Motor1;
	Data[2] = Motor2 >> 8;
	Data[3] = Motor2;
	Data[4] = Motor3 >> 8;
	Data[5] = Motor3;

	Motor->GetData(CANx, StdID, Num);
	SendData(CANx, StdID, Data, 6);
}

void CAN_Ctrl::SendData(Motor_CAN_Ctrl *Motor, int16_t Motor1, int16_t Motor2)
{
	CAN_TypeDef *CANx;
	uint32_t StdID;
	uint8_t Num;
	uint8_t Data[4];
	Data[0] = Motor1 >> 8;
	Data[1] = Motor1;
	Data[2] = Motor2 >> 8;
	Data[3] = Motor2;

	Motor->GetData(CANx, StdID, Num);
	SendData(CANx, StdID, Data, 4);
}

void CAN_Ctrl::SendData(Motor_CAN_Ctrl *Motor, int16_t Motor1)
{
	CAN_TypeDef *CANx;
	uint32_t StdID;
	uint8_t Num;
	uint8_t Data[2];
	Data[0] = Motor1 >> 8;
	Data[1] = Motor1;

	Motor->GetData(CANx, StdID, Num);
	SendData(CANx, StdID, Data, 2);
}

void CAN_Ctrl::CAN_CMD_RESET_ID(Motor_CAN_Ctrl *Motor)
{
	CAN_TypeDef *CANx;
	uint32_t StdID;
	uint8_t Num;
	Motor->GetData(CANx, StdID, Num);
	StdID = 0x700;
	uint8_t Data[8];
	for(uint8_t i = 0; i < 8; i++)
	{
		Data[i] = 0;
	}
	SendData(CANx, StdID, Data, 8);
}
