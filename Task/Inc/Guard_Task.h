#ifndef Guard_TASK_H
#define Guard_TASK_H

#include "dev_system.h"
#include "queue.h"
#include "timers.h"
#include "Message_Task.h"

#ifdef __cplusplus
extern "C" {
#endif

void Guard_Task(void *pvParameters);
extern void Guard_Scan_Time(TimerHandle_t xTimer);

#ifdef __cplusplus
}
#endif

#define GUARD_TOTAL_NUM ID_t_count

struct Error_Flags_t
{
	bool Gimbal;
	bool Visual;
	bool Judge;
};

struct SG_Data_t
{
	ID_t Name;//����
	uint8_t Enable;//ʹ�ܿ���
	uint32_t Time;//������
	uint32_t MaxValue;//���ʱֵ
	uint32_t Error;
	void (*errcallback)(uint8_t id);//�쳣�ص�����
};
	
class Guard_Ctrl
{
public:
	void Guard_Start(void);
	void Guard_Init(ID_t Name, uint32_t MaxValue, void(*errcb)(uint8_t name));
	void Guard_Scan(void);
	void Guard_Feed(ID_t *Name);
	void Guard_Enable(void);
	ID_t ID;
private:
	SG_Data_t SG_Structure[GUARD_TOTAL_NUM];
};

void IWDG_Init(uint8_t prer, uint16_t rlr);//IWDG��ʼ��
extern void IWDG_Feed(void);  //ι������
void Error_Enable(uint8_t name);
void System_RESET(uint8_t id);
void Guard_Return(uint8_t id);

extern QueueHandle_t Guard_Queue;
extern ID_t Guard_ID;
extern Guard_Ctrl Guard;
extern Error_Flags_t Error_Flag;
#endif
