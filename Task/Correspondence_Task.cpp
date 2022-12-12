#include "Correspondence_Task.h"
#include "device.h"

union F Gimbal_Union;

correspondence_ctrl corres;

void Correspondence_Task(void *pvParameters)
{
    corres.Corres_Init();
    while (1)
    {
        corres.Corres_Feedback();
        corres.Corres_Send();

        xQueueSend(Message_Queue, &ID_Data[CorrespondenceData], 0);
        vTaskDelay(2);
    }
}

void correspondence_ctrl::Corres_Init(void)
{	
	robo = get_robo_data_Point();
	Corres_Message = get_message_ctrl_pointer();
	Corres_Chassis = get_chassis_ctrl_pointer();

    GimbalS.Header = GIMBAL_SERIAL_HEADER;
    GimbalS.Tail = GIMBAL_SERIAL_TAIL;
	
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
    
    GimbalS.Grade = robo->game_robot_state.robot_level;
	Gimbal_Union.F=robo->shoot_data.bullet_speed;
	for(i=0;i<4;i++)
	{
		GimbalS.Shoot[i]=Gimbal_Union.I[i];
	}
	GimbalS.robo_ID = robo->game_robot_state.robot_id;

	if ((Corres_Message->SuperCapR.situation == CAP_CLOSE || Corres_Message->SuperCapR.situation == CAP_OPEN) && robo->game_robot_state.robot_level >= 1)
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
	SuperCapS.power = (uint8_t)robo->power_heat_data.chassis_power;
//	SuperCapS.power_limit = (uint8_t)robo->power_limit;
	SuperCapS.power_limit = 60;
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







