#include "dev_serial.h"

#include "Message_Task.h"
#include "protocol_judgement.h"

Serialctrl Serial3_Ctrl(USART3, Serial3_Buffer_Size, USART_IT_RXNE_AND_IDLE);
Serialctrl Serial6_Ctrl(USART6, Serial6_Buffer_Size, USART_IT_RXNE_AND_IDLE);
Serialctrl Serial7_Ctrl(UART7, Serial7_Buffer_Size, USART_IT_IDLE);
Serialctrl Serial8_Ctrl(UART8, Serial8_Buffer_Size, USART_IT_IDLE);

void Serialctrl::attachInterrupt(USART_CallbackFunction_t Function)
{
    USART_Function = Function;
}

void Serialctrl::IRQHandler(void)
{
    if(USART_GetITStatus(USARTx, USART_IT_RXNE) != RESET)
    {
        uint8_t c = USART_ReceiveData(USARTx);
        Buffer_Write(&_rx_buffer, c);
        if(USART_Function)
        {
            USART_Function(0);
        }
        USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
    }

    if(USART_GetITStatus(USARTx, USART_IT_IDLE) != RESET)
    {
        uint8_t c = USART_ReceiveData(USARTx);
        if(USART_Function)
        {
            USART_Function(1);
        }
        USART_ClearITPendingBit(USARTx, USART_IT_IDLE);
    }
}

void Serialctrl::sendData(uint8_t ch)
{
    USART_SendData(this->USARTx, ch);
    while(USART_GetFlagStatus(this->USARTx, USART_FLAG_TXE) == RESET);
}

void Serialctrl::sendData(const void *str)
{
    unsigned int k = 0;
    do
    {
        sendData(*((uint8_t *)str + k));
        k++;
    } while(*((uint8_t *)str + k) != '\0');
    while(USART_GetFlagStatus(this->USARTx, USART_FLAG_TC) == RESET) {}
}

void Serialctrl::sendData(const void *buf, uint8_t len)
{
    uint8_t *ch = (uint8_t *)buf;
    while(len--)
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
    if(_rx_buffer.pr == _rx_buffer.pw)
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
        Serial3_Ctrl.IRQHandler();
    }
    void USART6_IRQHandler(void)
    {
        Serial6_Ctrl.IRQHandler();
    }
    void UART7_IRQHandler(void)
    {
        Serial7_Ctrl.IRQHandler();
    }
    void UART8_IRQHandler(void)
    {
        Serial8_Ctrl.IRQHandler();
    }
}
