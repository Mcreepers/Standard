#include "app_serial.h"
#include "Message_Task.h"

Serial_Data_t Usart3(Serial3_Buffer_Size, Serial3_Data_Header, Serial3_Data_tail);
Serial_Data_t Usart6(Serial6_Buffer_Size, Serial6_Data_Header, Serial6_Data_tail);
Serial_Data_t Usart7(Serial7_Buffer_Size, Serial7_Data_Header, Serial7_Data_tail);
Serial_Data_t Usart8(Serial8_Buffer_Size, Serial8_Data_Header, Serial8_Data_tail);

void Serial3_Hook(bool mode)
{
	if (mode == 0)
	{
		if (Serial3.peek() != Usart3.Header)
		{
			Usart3.Temp = Serial3.read();
			return;
		}
		if (Serial3.available() == Usart3.Len - 1)
		{
			for (uint8_t i = 0;i < Usart3.Len - 1;i++)
			{
				Usart3.Data[i] = Serial3.read();
			}
			if (Usart3.Data[Usart3.Len - 2] == Usart3.Tail)
			{
				ID_Data[SerialData3].Data_Ptr = &Usart3;
				xQueueSendFromISR(Message_Queue, &ID_Data[SerialData3], 0);
			}
		}
	}
	else if (mode == 1)
	{
		xQueueSendFromISR(Message_Queue, &ID_Data[SerialData3], 0);
	}
}

void Serial6_Hook(bool mode)
{
	if (mode == 0)
	{
		if (Serial6.peek() != Usart6.Header)
		{
			Usart6.Temp = Serial6.read();
			return;
		}
		if (Serial6.available() == Usart6.Len - 1)
		{
			for (uint8_t i = 0;i < Usart6.Len - 1;i++)
			{
				Usart6.Data[i] = Serial6.read();
			}
			if (Usart6.Data[Usart6.Len - 2] == Usart6.Tail)
			{
				ID_Data[SerialData6].Data_Ptr = &Usart6;
				xQueueSendFromISR(Message_Queue, &ID_Data[SerialData6], 0);
			}
		}

	}
	else if (mode == 1)
	{
		xQueueSendFromISR(Message_Queue, &ID_Data[SerialData6], 0);
	}
}

void Serial7_Hook(bool mode)
{
	if (mode == 0)
	{
		if (Serial7.peek() != Usart7.Header)
		{
			Usart7.Temp = Serial7.read();
			return;
		}
		if (Serial7.available() == Usart7.Len - 1)
		{
			for (uint8_t i = 0;i < Usart7.Len - 1;i++)
			{
				Usart7.Data[i] = Serial7.read();
			}
			if (Usart7.Data[Usart7.Len - 2] == Usart7.Tail)
			{
				ID_Data[SerialData7].Data_Ptr = &Usart7;
				xQueueSendFromISR(Message_Queue, &ID_Data[SerialData7], 0);
			}
		}
	}
	else if (mode == 1)
	{
		xQueueSendFromISR(Message_Queue, &ID_Data[SerialData7], 0);
	}
}

void Serial8_Hook(bool mode)
{
	if (mode == 0)
	{
		if (Serial8.peek() != Usart8.Header)
		{
			Usart8.Temp = Serial8.read();
			return;
		}
		if (Serial8.available() == Usart8.Len - 1)
		{
			for (uint8_t i = 0;i < Usart8.Len - 1;i++)
			{
				Usart8.Data[i] = Serial8.read();
			}
			if (Usart8.Data[Usart8.Len - 2] == Usart8.Tail)
			{
				ID_Data[SerialData8].Data_Ptr = &Usart8;
				xQueueSendFromISR(Message_Queue, &ID_Data[SerialData8], 0);
			}
		}

	}
	else if (mode == 1)
	{
		xQueueSendFromISR(Message_Queue, &ID_Data[SerialData8], 0);
	}
}

void Serial_ALL_Init(void)
{
	JUDGE_SERIAL.Serial_Init(JUDGE_SERIAL_BAUD);
	//	GIMBAL_SERIAL.Serial_Init(GIMBAL_SERIAL_BAUD);
		// Serial8.Serial_Init(115200, SERIAL_8N1);
	Serial6.Serial_Init(115200, 6, 2);
	// Serial7.Serial_Init( 115200, SERIAL_8N1); //哪种初始化方式都可以
	//Serial7.Serial_Init( 115200, 6,2);

	Serial3.attachInterrupt(Serial3_Hook);
	Serial6.attachInterrupt(Serial6_Hook);
	Serial7.attachInterrupt(Serial7_Hook);
	Serial8.attachInterrupt(Serial8_Hook);
}
