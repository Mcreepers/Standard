#ifndef __APP_SERIAL_H
#define __APP_SERIAL_H

#include "dev_system.h"

struct Serial_Data_t
{
	uint8_t Header;
	uint8_t Tail;
	uint8_t Len;
	uint8_t Temp;
	uint8_t Data[max(max(Serial3_Buffer_Size,Serial6_Buffer_Size),max(Serial7_Buffer_Size,Serial8_Buffer_Size))];
	Serial_Data_t(uint8_t Len = 0, uint8_t Header = 0, uint8_t Tail = 0)
	{
		this->Len = Len;
		this->Header = Header;
        this->Tail = Tail;
    }
};

void Serial_ALL_Init(void);


#endif
