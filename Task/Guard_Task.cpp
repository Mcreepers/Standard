#include "Guard_Task.h"
#include "Message_Task.h"
#include "timers.h"

Guard_Ctrl Guard;
Error_Flags_t Error_Flag;

//��������
void Guard_Task(void *pvParameters)
{
    IWDG_Init(4, 100);
    Guard.Guard_Start();

    while (1)
    {
        Guard.Guard_Scan();
        IWDG_Feed();
        
        vTaskDelay(1);
    }
}
//��������ʼ
void Guard_Ctrl::Guard_Start(void)
{
    Guard_Init(chassis, 100, &System_Reset);
    // Guard_Init(UIdraw ,100, &System_Reset);
    // Guard_Init(CanData1 ,100, &System_Reset);
    Guard_Init(CanData2, 100, &System_Reset);
    // Guard_Init(serial3 ,100, &Error_Send);
    Guard_Init(serial6, 100, &Error_Enable);
    // Guard_Init(serial7 ,100, &Error_Send);
    // Guard_Init(serial8 ,100, &Error_Send);
    Guard_Init(RC_ctrl, 200, &System_Reset);
}
//���������ʼ��
void Guard_Ctrl::Guard_Init(ID_t Name, uint32_t MaxValue, void(*errcb)(uint8_t id))
{
    SG_Structure[Name].Name = Name;
    SG_Structure[Name].Enable = 0;//Ĭ�Ϲر�
    SG_Structure[Name].Time = 0;
    SG_Structure[Name].Error = 0;
    SG_Structure[Name].MaxValue = MaxValue;
    if (errcb == NULL)
    {
        SG_Structure[Name].errcallback = &Guard_Return;
    }
    else
    {
        SG_Structure[Name].errcallback = errcb;
    }
}
//��������ɨ��
void Guard_Ctrl::Guard_Scan(void)
{
    uint8_t i;
    for (i = 0;i < GUARD_TOTAL_NUM;i++)
    {
        if ((SG_Structure[i].Enable == 1) && (SG_Structure[i].MaxValue != 0))
        {
            SG_Structure[i].Error=xTaskGetTickCount()-SG_Structure[i].Time;
            if (SG_Structure[i].Error > SG_Structure[i].MaxValue)
            {
                SG_Structure[i].errcallback(i);
            }
            if ((int16_t)(SG_Structure[i].Error - SG_Structure[i].MaxValue) > 100)
            {
                SG_Structure[i].Enable = 0;
            }
        }
    }
}
//��������ι��
void Guard_Ctrl::Guard_Feed(ID_t *Name)
{
    if (Name == NULL||*Name==fault)
    {
        return;
    }
    
    uint8_t i;
    for (i = 0;i < GUARD_TOTAL_NUM;i++)
    {
        if (Guard.SG_Structure[i].Name == *Name)
        {
            Guard.SG_Structure[i].Enable = 1;
            Guard.SG_Structure[i].Time = xTaskGetTickCount();
            break;
        }
    }
}
//��������ʹ��(feed�е�ʹ��ֻ�������������)
void Guard_Ctrl::Guard_Enable(void)
{
    uint8_t i;
    for (i = 0;i < GUARD_TOTAL_NUM;i++)
    {
        Guard.SG_Structure[i].Enable = 1;
    }
}
//��������Ĭ�ϻص�����
void Guard_Return(uint8_t id)
{
    return;
}
void Error_Enable(uint8_t id)
{
    switch (id)
    {
    case serial6:
        Error_Flag.Gimbal = 1;
        Error_Flag.Visual = 1;
        break;
    case serial7:
        Error_Flag.Judge = 1;
        break;
    default:
        break;
    }
}

//��ʼ���������Ź�
//prer:��Ƶ��:0~7(ֻ�е�3λ��Ч!)
//rlr:�Զ���װ��ֵ,0~0XFFF.
//��Ƶ����=4*2^prer.�����ֵֻ����256!
//rlr:��װ�ؼĴ���ֵ:��11λ��Ч.
//ʱ�����(���):Tout=((4*2^prer)*rlr)/32 (ms).
void IWDG_Init(uint8_t prer, uint16_t rlr)
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
void System_Reset(uint8_t id)
{
    SCB->AIRCR = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
        (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
        SCB_AIRCR_SYSRESETREQ_Msk);
}
