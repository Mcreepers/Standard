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
	bool Start;//��ʼ���ȴ���ɱ�־λ
	bool Close;//�����쳣��������رձ�־λ(�����ص�����200ms�رո�id��������)
	bool Error;//��������־λ
	uint32_t Time;//������
	uint32_t StartValue;//��ʼ���ȴ�ʱ��
	uint32_t MaxValue;//���ʱֵ
	uint32_t CloseValue;//��������رճ�ʱֵ
	uint32_t DiffValue;//ʱ���
	void (*errcallback)(uint8_t id);//�쳣�ص�����
	void (*closecallback)(uint8_t id);//�رջص�����
};

class Guard_Ctrl
{
public:
	void Start(void);
	void Init(ID_e Name, uint32_t StartValue, uint32_t MaxValue, void(*errcb)(uint8_t id));
	void Init(ID_e Name, uint32_t StartValue, uint32_t MaxValue, void(*errcb)(uint8_t id), bool Close);
	void Init(ID_e Name, uint32_t StartValue, uint32_t MaxValue, void(*errcb)(uint8_t id), bool Close, uint32_t CloseValue, void(*closecb)(uint8_t id));
	void Scan(void);
	void Feed(ID_e Name);
	bool Return(ID_e Name);

	ID_e ID;
private:
	void Guard_Enable(void);

	SG_Data_t SG_Structure[GUARD_TOTAL_NUM];
};

void IWDG_Init(uint8_t prer, uint16_t rlr);//IWDG��ʼ��
extern void IWDG_Feed(void);  //ι������
void Error_Enable(uint8_t name);
void System_RESET(uint8_t id);
void Guard_Return(uint8_t id);
void Close_Enable(uint8_t id);

Guard_Ctrl *get_guard_ctrl_pointer();
extern ID_e Guard_ID;
#endif
