#ifndef Guard_TASK_H
#define Guard_TASK_H

#include "dev_system.h"
#include "queue.h"
#include "timers.h"
#include "Message_Task.h"
#include "Chassis_Task.h"

#ifdef __cplusplus
extern "C" {
#endif

void Guard_Task(void *pvParameters);
extern void Guard_Scan_Time(TimerHandle_t xTimer);

#ifdef __cplusplus
}
#endif

#define GUARD_TOTAL_NUM ID_e_count

struct Error_Flags_t
{
	bool Gimbal;
	bool Visual;
	bool Judge;
};

struct SG_Data_t
{
	ID_e Name;//名称
	bool Enable;//使能开关
	bool start;//初始化等待完成标志位
	bool close;//任务异常次数过多关闭标志位(触发次数超过100次//200ms关闭该id警戒任务)
	uint32_t Time;//计数器
	uint32_t StartValue;//初始化等待时间
	uint32_t MaxValue;//最大超时值
	uint32_t Error;//时间差
	void (*errcallback)(uint8_t id);//异常回调函数
};
	
class Guard_Ctrl
{
public:
	void Guard_Start(void);
	void Guard_Init(ID_e Name, bool close, uint32_t StartValue, uint32_t MaxValue, void(*errcb)(uint8_t id));
	void Guard_Scan(void);
	void Guard_Feed(ID_e *Name);
	void Guard_Enable(void);
	ID_e ID;
private:
	Chassis_Ctrl *Guard_Chassis;
	Message_Ctrl *Guard_Message;

	SG_Data_t SG_Structure[GUARD_TOTAL_NUM];
};

void IWDG_Init(uint8_t prer, uint16_t rlr);//IWDG初始化
extern void IWDG_Feed(void);  //喂狗函数
void Error_Enable(uint8_t name);
void System_RESET(uint8_t id);
void Guard_Return(uint8_t id);

Guard_Ctrl *get_guard_ctrl_pointer();
extern ID_e Guard_ID;
#endif
