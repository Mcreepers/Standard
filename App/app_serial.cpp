#include "app_serial.h"

#include "Message_Task.h"

Serial_Ctrl Serial3(
    USART3, Serial3_Buffer_Size, USART_IT_RXNE_AND_IDLE, Serial3_Data_Header, Serial3_Data_tail, 0);
Serial_Ctrl Serial6(
    USART6, Serial6_Buffer_Size, USART_IT_RXNE_AND_IDLE, Serial6_Data_Header, Serial6_Data_tail, 0);
Serial_Ctrl Serial7(
    UART7, Serial7_Buffer_Size, USART_IT_IDLE, Serial7_Data_Header, Serial7_Data_tail, 0);
Serial_Ctrl Serial8(
    UART8, Serial8_Buffer_Size, USART_IT_IDLE, Serial8_Data_Header, Serial8_Data_tail, 0);

Serial_Ctrl::Serial_Ctrl(
    USART_TypeDef *_USARTx, uint32_t BufferSize, uint8_t header, uint8_t tail, uint8_t lenth)
    : Serialctrl(_USARTx, BufferSize)
{
    this->buffer_size = BufferSize;
    this->Header = header;
    this->Tail = tail;
    this->Lenth = lenth;
    Data = new uint8_t[BufferSize + 1];
}

Serial_Ctrl::Serial_Ctrl(
    USART_TypeDef *_USARTx, uint32_t BufferSize, uint32_t USART_ITPending, uint8_t header,
    uint8_t tail, uint8_t lenth)
    : Serialctrl(_USARTx, BufferSize, USART_ITPending)
{
    this->buffer_size = BufferSize;
    this->Header = header;
    this->Tail = tail;
    this->Lenth = lenth;
    Data = new uint8_t[BufferSize + 1];
}

void Serial_Ctrl::Hook(bool mode)
{
    if(mode == 0)
    {
        if(Header == NULL && Tail == NULL)
        {
            return;
        }
        if(Header != NULL && this->peek() != Header)
        {
            Temp = this->read();
        }
    }
    else if(mode == 1)
    {
        Len = this->available();
        Data[0] = Len;
        for(uint8_t i = 0; i < Len; i++)
        {
            Data[i + 1] = this->read();
        }
        if(Lenth != NULL && Lenth != Len)
        {
            Data[0] = 0;
        }
        if(Tail != NULL && Data[Len + 1] != Tail)
        {
            Data[0] = 0;
        }
        Send_to_Message();
    }
}

uint8_t Serial_Ctrl::Get_Data(void *buf)
{
    if(Len == 0)
    {
        return 0;
    }
    buf = Data;
    return Len;
}

void Serial_Ctrl::Send_to_Message()
{
    if(this == &Serial3)
    {
        ID_Data[SerialData3].Data_Ptr = Data;
        xQueueSendFromISR(Message_Queue, &ID_Data[SerialData3], 0);
    }
    if(this == &Serial6)
    {
        ID_Data[SerialData3].Data_Ptr = Data;
        xQueueSendFromISR(Message_Queue, &ID_Data[SerialData6], 0);
    }
    if(this == &Serial7)
    {
        ID_Data[SerialData3].Data_Ptr = Data;
        xQueueSendFromISR(Message_Queue, &ID_Data[SerialData7], 0);
    }
    if(this == &Serial8)
    {
        ID_Data[SerialData3].Data_Ptr = Data;
        xQueueSendFromISR(Message_Queue, &ID_Data[SerialData7], 0);
    }
}

void Serial_Ctrl::IRQHandler()
{
    if(USART_GetITStatus(USARTx, USART_IT_RXNE) != RESET)
    {
        uint8_t c = USART_ReceiveData(USARTx);
        Buffer_Write(&_rx_buffer, c);

        Hook(0);

        USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
    }

    if(USART_GetITStatus(USARTx, USART_IT_IDLE) != RESET)
    {
        uint8_t c = USART_ReceiveData(USARTx);

        Hook(1);

        USART_ClearITPendingBit(USARTx, USART_IT_IDLE);
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

    // Serial3.attachInterrupt(Serial3_Hook);
    // Serial6.attachInterrupt(Serial6_Hook);
    // Serial7.attachInterrupt(Serial7_Hook);
    // Serial8.attachInterrupt(Serial8_Hook);
}

extern "C" {
    void USART3_IRQHandler(void)
    {
        Serial3.IRQHandler();
    }
    void USART6_IRQHandler(void)
    {
        Serial6.IRQHandler();
    }
    void UART7_IRQHandler(void)
    {
        Serial7.IRQHandler();
    }
    void UART8_IRQHandler(void)
    {
        Serial8.IRQHandler();
    }
}

void Serial_Com::SendData()
{
    SumCheck = 0;
    AddCheck = 0;
    uint8_t *ch = (uint8_t *)buf;

    Check_Calc(Head);
    Serial3.sendData(Head);
    Check_Calc(Addr);
    Serial3.sendData(Addr);
    Check_Calc(ID);
    Serial3.sendData(ID);
    Check_Calc(len);
    Serial3.sendData(len);
    while(len--)
    {
        Check_Calc(*ch++);
    }
    Serial3.sendData(int(SumCheck) & 0xff);
    Serial3.sendData(int(AddCheck) & 0xff);
}

void Serial_Com::ReceiveData(void *buf, uint8_t len)
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

void Serial_Com::Check_Calc(uint8_t data)
{
    SumCheck += data;
    AddCheck += SumCheck;
};

void Serial_Com::SendHeartBeat()
{
    HeartBeat[0].heartbeat != HeartBeat[0].heartbeat;
    buf = &HeartBeat;
    ID = 0xf0;
    len = 1;
    SendData();
}

void Serial_Com::SendChassisVelocity()
{
    buf = &ChassisVelocity;
    ID = 0x30;
    len = 6;
    SendData();
}
