#include "app_serial.h"

#include "Message_Task.h"

Serial_Ctrl Serial_Com;

void Serial3_Hook(bool mode)
{
    Serial_Com.Hook(SERIAL3, mode);
}

void Serial6_Hook(bool mode)
{
    Serial_Com.Hook(SERIAL6, mode);
}

void Serial7_Hook(bool mode)
{
    Serial_Com.Hook(SERIAL7, mode);
}

void Serial8_Hook(bool mode)
{
    Serial_Com.Hook(SERIAL8, mode);
}

void Serial_ALL_Init(void)
{
    JUDGE_SERIAL.Serial_Init(JUDGE_SERIAL_BAUD);
    //	GIMBAL_SERIAL.Serial_Init(GIMBAL_SERIAL_BAUD);
    // Serial8.Serial_Init(115200, SERIAL_8N1);
    Serial6_Ctrl.Serial_Init(115200, 6, 2);
    // Serial7.Serial_Init( 115200, SERIAL_8N1); //哪种初始化方式都可以
    //Serial7.Serial_Init( 115200, 6,2);

    Serial3_Ctrl.attachInterrupt(Serial3_Hook);
    Serial6_Ctrl.attachInterrupt(Serial6_Hook);
    Serial7_Ctrl.attachInterrupt(Serial7_Hook);
    Serial8_Ctrl.attachInterrupt(Serial8_Hook);
}

Serialctrl *Serial_Ctrl::Tran(USART_TypeDef *SERIAL)
{
    if(SERIAL == SERIAL3)
    {
        return &Serial3_Ctrl;
    }
    if(SERIAL == SERIAL6)
    {
        return &Serial6_Ctrl;
    }
    if(SERIAL == SERIAL7)
    {
        return &Serial7_Ctrl;
    }
    if(SERIAL == SERIAL8)
    {
        return &Serial8_Ctrl;
    }
	return NULL;
}

void Serial_Ctrl::Hook(USART_TypeDef *SERIAL, bool mode)
{
    if(SERIAL == SERIAL3)
    {
        Handle(&Serial3_Ctrl, Serial3, mode);
    }
    if(SERIAL == SERIAL6)
    {
        Handle(&Serial6_Ctrl, Serial6, mode);
    }
    if(SERIAL == SERIAL7)
    {
        Handle(&Serial7_Ctrl, Serial7, mode);
    }
    if(SERIAL == SERIAL8)
    {
        Handle(&Serial8_Ctrl, Serial8, mode);
    }
}

void Serial_Ctrl::Handle(Serialctrl *SerialCtrl, Serial_Data_t *Serial, bool mode)
{
    if(mode == 0)
    {
        if(Serial->Header == NULL && Serial->Tail == NULL)
        {
            return;
        }
        if(Serial->Header != NULL && SerialCtrl->peek() != Serial->Header)
        {
            Serial->Temp = SerialCtrl->read();
        }
    }
    else if(mode == 1)
    {
        Serial->Len = SerialCtrl->available();
        Serial->Data[0] = Serial->Len;
        for(uint8_t i = 0; i < Serial->Len; i++)
        {
            Serial->Data[i + 1] = SerialCtrl->read();
        }
        if(Serial->Lenth != NULL && Serial->Lenth != Serial->Len)
        {
            Serial->Data[0] = 0;
        }
        if(Serial->Tail != NULL && Serial->Data[Serial->Len] != Serial->Tail)
        {
            Serial->Data[0] = 0;
        }
		if(Serial->Data[0] != 0)
		{
			Send_to_Message(SerialCtrl);
		}
    }
}

uint8_t Serial_Ctrl::Get_Data(Serial_Data_t *Serial, uint8_t *buf)
{
    if(Serial->Len == 0)
    {
        return 0;
    }
    buf = Serial->Data;
    return Serial->Len;
}

void Serial_Ctrl::Send_to_Message(Serialctrl *SerialCtrl)
{
    if(SerialCtrl == &Serial3_Ctrl)
    {
        ID_Data[SerialData3].Data_Ptr = Serial3->Data;
        xQueueSendFromISR(Message_Queue, &ID_Data[SerialData3], 0);
    }
    if(SerialCtrl == &Serial6_Ctrl)
    {
        ID_Data[SerialData6].Data_Ptr = Serial6->Data;
        xQueueSendFromISR(Message_Queue, &ID_Data[SerialData6], 0);
    }
    if(SerialCtrl == &Serial7_Ctrl)
    {
        ID_Data[SerialData7].Data_Ptr = Serial7->Data;
        xQueueSendFromISR(Message_Queue, &ID_Data[SerialData7], 0);
    }
    if(SerialCtrl == &Serial8_Ctrl)
    {
        ID_Data[SerialData8].Data_Ptr = Serial8->Data;
        xQueueSendFromISR(Message_Queue, &ID_Data[SerialData8], 0);
    }
}

void Serial_Ctrl::SendData(USART_TypeDef *SERIAL, const void *buf, uint8_t len)
{
    Serialctrl *Serial = Tran(SERIAL);
    SendData(Serial, buf, len);
}

void Serial_Ctrl::SendData(USART_TypeDef *SERIAL, uint8_t ch)
{
    Serialctrl *Serial = Tran(SERIAL);
    SendData(Serial, ch);
}

void Serial_Ctrl::SendData(USART_TypeDef *SERIAL, const void *str)
{
    Serialctrl *Serial = Tran(SERIAL);
    SendData(Serial, str);
}

void Serial_Ctrl::SendData(Serialctrl *Serial, const void *buf, uint8_t len)
{
    Serial->sendData(buf, len);
}

void Serial_Ctrl::SendData(Serialctrl *Serial, uint8_t ch)
{
    Serial->sendData(ch);
}

void Serial_Ctrl::SendData(Serialctrl *Serial, const void *str)
{
    Serial->sendData(str);
}

void Serial_Comm::SendData()
{
    SumCheck = 0;
    AddCheck = 0;
    uint8_t *ch = (uint8_t *)buf;

    Check_Calc(Head);
    SerialCtrl->sendData(Head);
    Check_Calc(Addr);
    SerialCtrl->sendData(Addr);
    Check_Calc(ID);
    SerialCtrl->sendData(ID);
    Check_Calc(len);
    SerialCtrl->sendData(len);
    while(len--)
    {
        Check_Calc(*ch++);
    }
    SerialCtrl->sendData(int(SumCheck) & 0xff);
    SerialCtrl->sendData(int(AddCheck) & 0xff);
}

void Serial_Comm::ReceiveData(void *buf, uint8_t len)
{
    SumCheck = 0;
    AddCheck = 0;
    uint8_t *ch = (uint8_t *)buf;

    if(ch[0] != 0xff || ch[1] != 0x01)
    {
        return;
    }
    for(uint8_t i = 0; i < len - 2; i++)
    {
        Check_Calc(ch[i]);
    }
    if(SumCheck != ch[len - 1] || AddCheck != ch[len])
    {
        return;
    }

    switch(ch[2])
    {
    case 0xf0: {
        HeartBeat[1].heartbeat = ch[3];
        break;
    }
    case 0x10: {
        ChassisVelocity[1].chassis_vx = ch[3];
        ChassisVelocity[1].chassis_vy = ch[4];
        ChassisVelocity[1].chassis_angle = ch[5];
    }
    default:
    break;
    }
}

void Serial_Comm::Check_Calc(uint8_t data)
{
    SumCheck += data;
    AddCheck += SumCheck;
};

void Serial_Comm::SendHeartBeat()
{
    HeartBeat[0].heartbeat != HeartBeat[0].heartbeat;
    buf = &HeartBeat;
    ID = 0xf0;
    len = 1;
    SendData();
}

void Serial_Comm::SendChassisVelocity()
{
    buf = &ChassisVelocity;
    ID = 0x30;
    len = 6;
    SendData();
}
