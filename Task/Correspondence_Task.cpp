#include "Correspondence_Task.h"
#include "device.h"
#include "Message_Task.h"

correspondence_ctrl corres;

usart_Data_t Usart3( Serial3_Buffer_Size, Serial3_Data_Header, Serial3_Data_tail );
usart_Data_t Usart6( Serial6_Buffer_Size, Serial6_Data_Header, Serial6_Data_tail );
usart_Data_t Usart7( Serial7_Buffer_Size, Serial7_Data_Header, Serial7_Data_tail );
usart_Data_t Usart8( Serial8_Buffer_Size, Serial8_Data_Header, Serial8_Data_tail );

void Correspondence_Task(void *pvParameters)
{
    corres.Corres_Init();
    while (1)
    {
        
        corres.Corres_Feedback();
        corres.Corres_Send();

        
//        xQueueSend(Message_Queue, &(Message_Data.Data_ID = correspondence), 0);
        vTaskDelay(2);
    }
}

void correspondence_ctrl::Corres_Init(void)
{
    flag = &Chassis.Flags;
    velocity = &Chassis.Velocity;
    gimbal = get_gimbal_data_point();
    robo = get_robo_data_Point();

    Gimbal.Header = GIMBAL_SERIAL_HEADER;
    Gimbal.Tail = GIMBAL_SERIAL_TAIL;

    Serial3.attachInterrupt(Serial3_Hook);
	Serial6.attachInterrupt(Serial6_Hook);
	
	usart7_DMA_init();
}

void correspondence_ctrl::Corres_Send(void)
{
    Serial6.sendData(&Gimbal);
}

void correspondence_ctrl::Corres_Feedback(void)
{
    uart7_dma_get();
    
    Gimbal.grade = robo->robo_level;
}

void Serial3_Hook(void)
{
    if (Serial3.peek() == Usart3.Header&&Usart3.Num==0)
    {
        Usart3.Num = Usart3.Len;
    }
    else if (Usart3.Num > 0 && Usart3.Num <= Usart3.Len)
    {
        Usart3.Num--;
    }
    else if ((Serial3.peek() == Usart3.Tail||Usart3.Tail==NULL) && Usart3.Num == 0)
    {
//        xQueueSendFromISR(Message_Queue, &(Message_Data.Data_ID=serial3), 0);
    }
}

void Serial6_Hook(void)
{
    if (Serial6.peek() == Usart6.Header&&Usart6.Num==0)
    {
        Usart6.Num = Usart6.Len;
    }
    else if (Usart6.Num > 0 && Usart6.Num <= Usart6.Len)
    {
        Usart6.Num--;
    }
    else if ((Serial6.peek() == Usart6.Tail||Usart6.Tail==NULL) && Usart6.Num == 0)
    {
//        xQueueSendFromISR(Message_Queue, &(Message_Data.Data_ID=serial6), 0);
    }
}









