#include "Correspondence_Task.h"
#include "device.h"
#include "Message_Task.h"

union F Gimbal_Union;
	
correspondence_ctrl corres;
Message_Data_t Message_Data_corres[5];
Usart_Data_t Usart3( Serial3_Buffer_Size, Serial3_Data_Header, Serial3_Data_tail );
Usart_Data_t Usart6( Serial6_Buffer_Size, Serial6_Data_Header, Serial6_Data_tail );
Usart_Data_t Usart7( Serial7_Buffer_Size, Serial7_Data_Header, Serial7_Data_tail );
Usart_Data_t Usart8( Serial8_Buffer_Size, Serial8_Data_Header, Serial8_Data_tail );

void Serial3_Hook(void);
void Serial6_Hook(void);
void Serial7_Hook(void);

void Correspondence_Task(void *pvParameters)
{
    corres.Corres_Init();
    while (1)
    {
        corres.Corres_Feedback();
        corres.Corres_Send();

        xQueueSend(Message_Queue, &Message_Data_corres[0], 0);
        vTaskDelay(2);
    }
}

void correspondence_ctrl::Corres_Init(void)
{
	Message_Data_corres[0].Data_ID = correspondence;
	Message_Data_corres[1].Data_ID = serial3;
	Message_Data_corres[2].Data_ID = serial6;
	Message_Data_corres[3].Data_ID = serial7;
	Message_Data_corres[4].Data_ID = serial8;
	
	robo = get_robo_data_Point();
	Corres_Message = get_message_ctrl_pointer();
	Corres_Chassis = get_chassis_ctrl_pointer();

    GimbalS.Header = GIMBAL_SERIAL_HEADER;
    GimbalS.Tail = GIMBAL_SERIAL_TAIL;

    Serial3.attachInterrupt(Serial3_Hook);
	Serial6.attachInterrupt(Serial6_Hook);
	Serial7.attachInterrupt(Serial7_Hook);
	
	usart7_DMA_init();
}

void correspondence_ctrl::Corres_Send(void)
{
	GIMBAL_SERIAL.sendData(&GimbalS, 8);

	CAP_SendData(&SuperCapS,4);
}

void correspondence_ctrl::Corres_Feedback(void)
{
	uint8_t i;
    
    GimbalS.Grade = robo->robo_level;
	Gimbal_Union.F=robo->robo_17_Speed;
	for(i=0;i<4;i++)
	{
		GimbalS.Shoot[i]=Gimbal_Union.I[i];
	}
	GimbalS.robo_ID = robo->robo_ID;

	if ((Corres_Message->SuperCapR.situation == CAP_CLOSE || Corres_Message->SuperCapR.situation == CAP_OPEN) && robo->robo_HP > 1)
	{
		SuperCapS.enable = 0xff;
	}
	else
	{
		SuperCapS.enable = 0x00;
	}

	if (Corres_Chassis->Flags.Speed_Up_Flag == true && Corres_Message->SuperCapR.mode == 0x00)
	{
		SuperCapS.mode = 0xff;
	}
	else if (Corres_Chassis->Flags.Speed_Up_Flag == false && Corres_Message->SuperCapR.mode == 0xff)
	{
		SuperCapS.mode = 0x00;
	}
	SuperCapS.power = (uint8_t)robo->chassis_power;
//	SuperCapS.power_limit = (uint8_t)robo->power_limit;
	SuperCapS.power_limit = 60;
}

void Serial3_Hook(void)
{
    if (Serial3.peek() != Usart3.Header)
	{
		Usart3.Temp=Serial3.read();
		return;
	}
	if (Serial3.available()==Usart3.Len-1)
    {
		for (uint8_t i = 0;i < Usart3.Len-1;i++)
		{
			Usart3.Data[i] = Serial3.read();
		}
		if(Usart3.Data[Usart3.Len-2]==Usart3.Tail)
		{
			xQueueSendFromISR(Message_Queue, &Message_Data_corres[1], 0);
		}
    }
}

void Serial6_Hook(void)
{
    if (Serial6.peek() != Usart6.Header)
	{
		Usart6.Temp=Serial6.read();
		return;
	}
	if (Serial6.available()==Usart6.Len-1)
    {
		for (uint8_t i = 0;i < Usart6.Len-1;i++)
		{
			Usart6.Data[i] = Serial6.read();
		}
		if (Usart6.Data[Usart6.Len - 2] == Usart6.Tail)
		{
			xQueueSendFromISR(Message_Queue, &Message_Data_corres[2], 0);
		}
    }
}

void Serial7_Hook(void)
{
	xQueueSendFromISR(Message_Queue, &Message_Data_corres[3], 0);
//    if (Serial7.peek() != Usart7.Header)
//	{
//		Usart7.Temp=Serial7.read();
//		return;
//	}
//	if (Serial7.available()==Usart7.Len-1)
//    {
//		for (uint8_t i = 0;i < Usart7.Len-1;i++)
//		{
//			Usart7.Data[i] = Serial7.read();
//		}
//		if(Usart7.Data[Usart7.Len-2]==Usart7.Tail)
//		{
//			xQueueSendFromISR(Message_Queue, &Message_Data_corres[3], 0);
//		}
//    }
}

void Serial8_Hook(void)
{
    if (Serial8.peek() != Usart8.Header)
	{
		Usart8.Temp=Serial8.read();
		return;
	}
	if (Serial8.available()==Usart8.Len-1)
    {
		for (uint8_t i = 0;i < Usart8.Len-1;i++)
		{
			Usart8.Data[i] = Serial8.read();
		}
		if(Usart8.Data[Usart8.Len-2]==Usart8.Tail)
		{
			xQueueSendFromISR(Message_Queue, &Message_Data_corres[4], 0);
		}
    }
}

void correspondence_ctrl::CAP_SendData(const void *buf, uint8_t len)
{
	CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CAP_SENT_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
	
	uint8_t num = 0;
	uint8_t *ch = (uint8_t *)buf;
	while (len--)
	{
		TxMessage.Data[num++] = (*ch++);
	}
	
	while (num < 7)
	{
		TxMessage.Data[num++] = 0x00;
	}

    CAN_Transmit( CAN1, &TxMessage );
}







