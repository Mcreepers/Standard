#ifndef _DEV_SERIAL
#define _DEV_SERIAL

#ifdef __cplusplus
extern "C" {
#endif

#include "platform.h"

#ifdef __cplusplus
}
#endif

#include "drivers_serial.h"

#define Serial3_Buffer_Size 7
#define Serial6_Buffer_Size 8
#define Serial7_Buffer_Size 7
#define Serial8_Buffer_Size 7
//ÎÞÖ¡Î²ÔòÎª NULL
#define Serial3_Data_Header 0xff
#define Serial3_Data_tail 0xfe
#define Serial6_Data_Header 0xff
#define Serial6_Data_tail 0xfe
#define Serial7_Data_Header 0xff
#define Serial7_Data_tail 0xfe
#define Serial8_Data_Header 0xff
#define Serial8_Data_tail 0xfe

struct Usart_Data_t
{
	uint8_t Header;
	uint8_t Tail;
	uint8_t Len;
	uint8_t Num;
	uint8_t Data[10];
	Usart_Data_t(uint8_t Len = 0, uint8_t Header = 0, uint8_t Tail = 0)
	{
		this->Len = Len;
		this->Header = Header;
		this->Tail = Tail;
	}
};

class Serialctrl : public Serialdev
{
	public:
	Serialctrl(USART_TypeDef *_USARTx, uint32_t BufferSize, uint8_t leader, uint8_t tail) : Serialdev(_USARTx, BufferSize) { usart = new Usart_Data_t(BufferSize, leader, tail); }
	Serialctrl(USART_TypeDef *_USARTx, uint32_t BufferSize, uint16_t USART_ITPending, uint8_t leader, uint8_t tail) : Serialdev(_USARTx, BufferSize, USART_ITPending) { usart = new Usart_Data_t(BufferSize, leader, tail); }

	void attachInterrupt(USART_CallbackFunction_t Function);
	void IRQHandler(void);

	void sendData(uint8_t ch);
	void sendData(const void *str);
	void sendData(const void *buf, uint8_t len);

	int available(void);
	uint8_t read(void);
	int8_t Data[10];
	Usart_Data_t *usart;
	private:

	void Hook(void);
	int peek(void);
	void flush(void);
};

#endif /* _DEV_SERIAL */
