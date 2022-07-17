#include "Guard_Task.h"
#include "Message_Task.h"
#include "timers.h"
Guard_Ctrl Guard;
Guard_ID_t Guard_ID;
Error_Flags_t Error_Flag;
//��������
void Guard_Task(void *pvParameters)
{
    IWDG_Init(4, 100);
    Guard.Guard_Start();
    
    while (1)
    {
        if (xQueueReceive(Guard_Queue, &Guard_ID, portMAX_DELAY))
        {
            Guard_Feed(&Guard_ID);
            IWDG_Feed();
        }
    }
}
//��������ʼ
void Guard_Ctrl::Guard_Start(void)
{
    // uint8_t i;
    *Guard.ID = message;
    Guard_Init(message, ID, 100, &System_Reset);
    *Guard.ID = chassis;
    Guard_Init(chassis, ID ,100, &System_Reset);
    *Guard.ID = UIdraw;
    // Guard_Init(UIdraw2, ID ,100, &System_Reset);
    *Guard.ID = Can1;
    // Guard_Init(Can1, ID ,100, &System_Reset);
    *Guard.ID = Can2;
    Guard_Init(Can2, ID ,100, &System_Reset);
    *Guard.ID = usart3;
    // Guard_Init(usart3, ID ,100, &Error_Send);
    *Guard.ID = usart6;
    Guard_Init(usart6, ID ,100, &Error_Send);
    *Guard.ID = usart7;
    // Guard_Init(usart7, ID ,100, &Error_Send);
    *Guard.ID = usart8;
    // Guard_Init(usart8, ID ,100, &Error_Send);
    *Guard.ID = rc_ctrl1;
    Guard_Init(rc_ctrl1, ID ,200, &System_Reset);
}
//���������ʼ��
void Guard_Ctrl::Guard_Init(Guard_ID_t num, Guard_ID_t *Name, uint32_t MaxValue, void(*errcb)(void))
{
    SG_Structure[num].Name = *Name;
    SG_Structure[num].Enable = 0;
    SG_Structure[num].Counter = 0;
    SG_Structure[num].MaxValue = MaxValue;
    if (errcb == NULL)
    {
        SG_Structure[num].errcallback = &Guard_Return;
    }
    else
    {
        SG_Structure[num].errcallback = errcb;
    }
}

//��������ɨ�裨�������ʱ����ִ�У�
void Guard_Ctrl::Guard_Scan(void)
{
    uint8_t i;
    for (i = 0;i < GUARD_TOTAL_NUM;i++)
    {
        if ((SG_Structure[i].Enable == 1) && (SG_Structure[i].MaxValue != 0))
        {
            SG_Structure[i].Counter++;
            if (SG_Structure[i].Counter > SG_Structure[i].MaxValue)
            {
                SG_Structure[i].errcallback();
            }
            if ((int16_t)(SG_Structure[i].Counter - SG_Structure[i].MaxValue) > 20)
            {
                SG_Structure[i].Enable = 0;
            }
        }
    }
}
//��������ι��
void Guard_Feed(Guard_ID_t *Name)
{
    uint8_t i;
    for (i = 0;i < GUARD_TOTAL_NUM;i++)
    {
        if (Guard.SG_Structure[i].Name == *Name)
        {
            Guard.SG_Structure[i].Enable = 1;
            Guard.SG_Structure[i].Counter = 0;
            break;
        }
    }
}
//�������Ϣ���з�����Ϣ
void Error_Send(void)
{
    xQueueOverwriteFromISR(Error_Queue, &Guard.ID, 0);
}
//��������Ĭ�ϻص�����
void Guard_Return(void)
{
    return;
}
//�����ʱ��
void Guard_Scan_Time(TimerHandle_t xTimer)
{
    Guard.Guard_Scan();
}
//��ʼ���������Ź�
//prer:��Ƶ��:0~7(ֻ�е�3λ��Ч!)
//rlr:�Զ���װ��ֵ,0~0XFFF.
//��Ƶ����=4*2^prer.�����ֵֻ����256!
//rlr:��װ�ؼĴ���ֵ:��11λ��Ч.
//ʱ�����(���):Tout=((4*2^prer)*rlr)/32 (ms).
void IWDG_Init(uint8_t prer,uint16_t rlr)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //ʹ�ܶ�IWDG->PR IWDG->RLR��д
	
	IWDG_SetPrescaler(prer); //����IWDG��Ƶϵ��

	IWDG_SetReload(rlr);   //����IWDGװ��ֵ

	IWDG_ReloadCounter(); //reload
	
	IWDG_Enable();       //ʹ�ܿ��Ź�
}

//ι�������Ź�
void IWDG_Feed(void)
{
	IWDG_ReloadCounter();//reload
}

//�Ĵ��������λ
void System_Reset(void)
{
	SCB->AIRCR  = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos)    |
		   (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
			SCB_AIRCR_SYSRESETREQ_Msk);
}
