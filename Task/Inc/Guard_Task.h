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
	ID_e Name;//����
	bool Enable;//ʹ�ܿ���
	bool start;//��ʼ���ȴ���ɱ�־λ
	bool close;//�����쳣��������رձ�־λ(������������100��//200ms�رո�id��������)
	uint32_t Time;//������
	uint32_t StartValue;//��ʼ���ȴ�ʱ��
	uint32_t MaxValue;//���ʱֵ
	uint32_t Error;//ʱ���
	void (*errcallback)(uint8_t id);//�쳣�ص�����
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

void IWDG_Init(uint8_t prer, uint16_t rlr);//IWDG��ʼ��
extern void IWDG_Feed(void);  //ι������
void Error_Enable(uint8_t name);
void System_RESET(uint8_t id);
void Guard_Return(uint8_t id);

Guard_Ctrl *get_guard_ctrl_pointer();
extern ID_e Guard_ID;
#endif
