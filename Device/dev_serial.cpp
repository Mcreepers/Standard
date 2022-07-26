#include "dev_serial.h"

#include "Message_Task.h"
#include "protocol_judgement.h"

Serialctrl Serial3(USART3, Serial3_Buffer_Size);
Serialctrl Serial6(USART6, Serial6_Buffer_Size);
Serialctrl Serial7(UART7, Serial7_Buffer_Size);
Serialctrl Serial8(UART8, Serial8_Buffer_Size, USART_IT_IDLE);

void Serialctrl::attachInterrupt(USART_CallbackFunction_t Function)
{
    USART_Function = Function;
}

void Serialctrl::IRQHandler(void)
{
    if (USART_GetITStatus(USARTx, USART_IT_RXNE) != RESET)
    {
        uint8_t c = USART_ReceiveData(USARTx);
        Buffer_Write(&_rx_buffer, c);
        if (USART_Function)
        {
            USART_Function();
        }
        USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
    }
}

void Serialctrl::sendData(uint8_t ch)
{
    USART_SendData(this->USARTx, ch);
    while (USART_GetFlagStatus(this->USARTx, USART_FLAG_TXE) == RESET);
}

void Serialctrl::sendData(const void *str)
{
    unsigned int k = 0;
    do
    {
        sendData(*((uint8_t *)str + k));
        k++;
    } while (*((uint8_t *)str + k) != '\0');
    while (USART_GetFlagStatus(this->USARTx, USART_FLAG_TC) == RESET) {}
}

void Serialctrl::sendData(const void *buf, uint8_t len)
{
    uint8_t *ch = (uint8_t *)buf;
    while (len--)
    {
        sendData(*ch++);
    }
}

int Serialctrl::available(void)
{
    return ((unsigned int)(_rx_buffer.buf_size + _rx_buffer.pw - _rx_buffer.pr)) % _rx_buffer.buf_size;
}

uint8_t Serialctrl::read(void)
{
    uint8_t c = 0;
    Buffer_Read(&_rx_buffer, &c);
    return c;
}

int Serialctrl::peek(void)
{
    if (_rx_buffer.pr == _rx_buffer.pw)
    {
        return -1;
    }
    else
    {
        return _rx_buffer.fifo[_rx_buffer.pr];
    }
}

void Serialctrl::flush(void)
{
    _rx_buffer.pr = _rx_buffer.pw;
}

extern "C"{
    void USART3_IRQHandler(void)
    {
        Message_Data.Data_ID = serial3;
        Serial3.IRQHandler();
    }
    void USART6_IRQHandler(void)
    {
        Message_Data.Data_ID = serial6;
        Serial6.IRQHandler();
    }
    void UART7_IRQHandler(void)
    {
        Message_Data.Data_ID = serial7;
        xQueueSendFromISR(Message_Queue, &Message_Data.Data_ID, 0);
        // Serial7.IRQHandler();
    }
    // void UART8_IRQHandler(void)
    // {
    //     Message_Data.Data_ID = serial8;
    // 	Serial8.IRQHandler();
    // }
}
