#ifndef Correspondence_TASK_H
#define Correspondence_TASK_H

#include "dev_system.h"
#include "Chassis_Task.h"
#include "protocol_judgement.h"

#ifdef __cplusplus
extern "C" {
#endif

void Correspondence_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#define GIMBAL_SERIAL_HEADER 0xff
#define GIMBAL_SERIAL_TAIL 0xfe

//ÎÞÖ¡Î²ÔòÎª NULL
#define Serial3_Data_Header 0xff
#define Serial3_Data_tail 0xfe
#define Serial6_Data_Header 0xff
#define Serial6_Data_tail 0xfe
#define Serial7_Data_Header 0xff
#define Serial7_Data_tail 0xfe
#define Serial8_Data_Header 0xff
#define Serial8_Data_tail 0xfe

struct gimbal_data_t
{
    uint8_t Header;
    uint8_t grade;
    uint8_t Tail;
};

struct usart_Data_t
{
	uint8_t Header;
	uint8_t Tail;
	uint8_t Len;
	uint8_t Num;
	uint8_t Data[20];
	usart_Data_t(uint8_t Len = 0, uint8_t Header = 0, uint8_t Tail = 0)
	{
		this->Len = Len;
		this->Header = Header;
		this->Tail = Tail;
	}
};

class correspondence_ctrl
{
public:
    Chassis_Ctrl_Flags_t *flag;
    Chassis_Velocity_t *velocity;
    const Gimbal_Data_t *gimbal;
    const robo_data_t *robo;
    
    void Corres_Init(void);
    void Corres_Send(void);
    void Corres_Feedback(void);
private:
    gimbal_data_t Gimbal;

    void Visual_Feedback(void);
    void Chassis_Feedback(void);
};

extern usart_Data_t Usart3;
extern usart_Data_t Usart6;
extern usart_Data_t Usart7;
extern usart_Data_t Usart8;
extern void Serial3_Hook(void);
extern void Serial6_Hook(void);

#endif
