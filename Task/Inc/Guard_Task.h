#ifndef Guard_TASK_H
#define Guard_TASK_H

#include "dev_system.h"
#include "queue.h"
#include "timers.h"
#ifdef __cplusplus
extern "C" {
#endif

void Guard_Task(void *pvParameters);
extern void Guard_Scan_Time(TimerHandle_t xTimer);

#ifdef __cplusplus
}
#endif

typedef enum
{
    message=0x00,
    chassis,
    UIdraw,
   	Can1,
	Can2,
	usart3,
	usart6,
	usart7,
	usart8,
	rc_ctrl1,
	Guard_ID_t_Count
}Guard_ID_t;

#define GUARD_TOTAL_NUM Guard_ID_t_Count

struct Error_Flags_t
{
	bool Gimbal;
	bool Visual;
	bool Judge;
};

struct SG_Data_t
{
	uint8_t Name;//名称
	uint8_t Enable;//使能开关
	uint32_t Counter;//计数器
	uint32_t MaxValue;//最大超时值

	void (*errcallback)(void);//异常回调函数
};
	
class Guard_Ctrl
{
public:
	Guard_Ctrl() { ID = new  Guard_ID_t; }
	void Guard_Start(void);
	void Guard_Init(Guard_ID_t num, Guard_ID_t *Name, uint32_t MaxValue, void(*errcb)(void));
	void Guard_Scan(void);
	Guard_ID_t *ID;
	SG_Data_t SG_Structure[GUARD_TOTAL_NUM];
private:
};

void IWDG_Init(uint8_t prer, uint16_t rlr);//IWDG初始化
extern void IWDG_Feed(void);  //喂狗函数
extern void Error_Send(void);
extern void Guard_Feed(Guard_ID_t *Name);
extern void Error_Send(void);
extern void Guard_Return(void);

void System_Reset(void);
extern QueueHandle_t Guard_Queue;
extern QueueHandle_t Error_Queue;
extern Guard_ID_t Guard_ID;
extern Guard_Ctrl Guard;
extern Error_Flags_t Error_Flag;
#endif
