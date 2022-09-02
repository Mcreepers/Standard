#ifndef Correspondence_TASK_H
#define Correspondence_TASK_H

#include "dev_system.h"
#include "Chassis_Task.h"
#include "Message_Task.h"
#include "protocol_judgement.h"

#ifdef __cplusplus
extern "C" {
#endif

void Correspondence_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

union F
{
    uint8_t I[4];
    fp32 F;
};

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

#define RATE_5_HZ		5
#define RATE_10_HZ		10
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000

#define MAIN_LOOP_RATE 	RATE_500_HZ

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)

struct Gimbal_Send_Data_t
{
    uint8_t Header;
    uint8_t Grade;
	uint8_t Shoot[4];
    uint8_t Tail;
};

struct Usart_Data_t
{
	uint8_t Header;
	uint8_t Tail;
	uint8_t Len;
	uint8_t Temp;
	uint8_t Data[max(max(Serial3_Buffer_Size,Serial6_Buffer_Size),max(Serial7_Buffer_Size,Serial8_Buffer_Size))];
	Usart_Data_t(uint8_t Len = 0, uint8_t Header = 0, uint8_t Tail = 0)
	{
		this->Len = Len;
		this->Header = Header;
        this->Tail = Tail;
    }
};

class correspondence_ctrl
{
public:
    const robo_data_t *robo;

    void Corres_Init(void);
    void Corres_Send(void);
    void Corres_Feedback(void);
private:
    Message_Ctrl *Corres_Message;
    Chassis_Ctrl *Corres_Chassis;
    Gimbal_Send_Data_t GimbalS;

    void Visual_Feedback(void);
    void Chassis_Feedback(void);
};

extern Usart_Data_t Usart3;
extern Usart_Data_t Usart6;
extern Usart_Data_t Usart7;
extern Usart_Data_t Usart8;

#endif
