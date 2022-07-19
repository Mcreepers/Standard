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
    // uint8_t i;
    ID = chassis;
    Guard_Init(&ID, 100, &System_Reset);
    ID = UIdraw;
    // Guard_Init(&ID ,100, &System_Reset);
    ID = CanData1;
    // Guard_Init(&ID ,100, &System_Reset);
    ID = CanData2;
    Guard_Init(&ID, 100, &System_Reset);
    ID = serial3;
    // Guard_Init(&ID ,100, &Error_Send);
    ID = serial6;
    Guard_Init(&ID, 100, &Error_Send);
    ID = serial7;
    // Guard_Init(&ID ,100, &Error_Send);
    ID = serial8;
    // Guard_Init(&ID ,100, &Error_Send);
    ID = RC_ctrl;
    Guard_Init( &ID, 200, &System_Reset);
}
//���������ʼ��
void Guard_Ctrl::Guard_Init(ID_t *Name, uint32_t MaxValue, void(*errcb)(void))
{
    SG_Structure[*Name].Name = Name;
    SG_Structure[*Name].Enable = 0;
    SG_Structure[*Name].Time = 0;
    SG_Structure[*Name].Error = 0;
    SG_Structure[*Name].MaxValue = MaxValue;
    if (errcb == NULL)
    {
        SG_Structure[*Name].errcallback = &Guard_Return;
    }
    else
    {
        SG_Structure[*Name].errcallback = errcb;
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
            SG_Structure[i].Error=xTaskGetTickCount()-SG_Structure[i].Time;
            if (SG_Structure[i].Error > SG_Structure[i].MaxValue)
            {
                SG_Structure[i].errcallback();
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
    uint8_t i;
    for (i = 0;i < GUARD_TOTAL_NUM;i++)
    {
        if (Guard.SG_Structure[i].Name == Name)
        {
            Guard.SG_Structure[i].Enable = 1;
            Guard.SG_Structure[i].Time = xTaskGetTickCount();
            break;
        }
    }
}
//��������Ĭ�ϻص�����
void Guard_Return(void)
{
    return;
}
void Error_Send(void)
{
    switch (Guard.ID)
    {
    case serial6:
        /* code */
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
void System_Reset(void)
{
    SCB->AIRCR = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
        (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
        SCB_AIRCR_SYSRESETREQ_Msk);
}
