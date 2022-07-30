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
#define Serial6_Buffer_Size 11
#define Serial7_Buffer_Size 7
#define Serial8_Buffer_Size 7

class Serialctrl : public Serialdev
{
public:
	Serialctrl(USART_TypeDef *_USARTx, uint32_t BufferSize) : Serialdev(_USARTx, BufferSize) {}
	Serialctrl(USART_TypeDef *_USARTx, uint32_t BufferSize, uint16_t USART_ITPending) : Serialdev(_USARTx, BufferSize, USART_ITPending) {}

	void attachInterrupt(USART_CallbackFunction_t Function);
	void IRQHandler(void);

	void sendData(uint8_t ch);
	void sendData(const void *str);
	void sendData(const void *buf, uint8_t len);

	int available(void);
	uint8_t read(void);
	int peek(void);
private:

	void flush(void);
};

#endif /* _DEV_SERIAL */
