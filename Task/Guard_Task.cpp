#include "Guard_Task.h"
#include "Message_Task.h"
#include "timers.h"
#include "Chassis_Task.h"

Guard_Ctrl Guard;
Error_Flags_t Error_Flag;

//��������
void Guard_Task(void *pvParameters)
{
    Guard.Guard_Start();
    vTaskDelay(300);
    //    IWDG_Init(4, 100);

    while (1)
    {//scan�Ż�����������
        Guard.Guard_Scan();
        IWDG_Feed();

        vTaskDelay(2);
    }
}
//��������ʼ
void Guard_Ctrl::Guard_Start(void)
{
    Guard_Chassis = get_chassis_ctrl_pointer();
    Guard_Message = get_message_ctrl_pointer();

    // Guard_Init(CanData1, 1000 ,100, &System_RESET);
    // Guard_Init(CanData2, 1000, 100, &System_RESET);
    // Guard_Init(serial3, 1000, 100, &Error_Enable, true);
    // Guard_Init(serial6, 1000, 100, &Error_Enable, true);
    // Guard_Init(serial7, 1000 ,100, &Error_Enable, true);
    // Guard_Init(serial8, 1000 ,100, &Error_Enable, true);
    // Guard_Init(RC_ctrl, 1000, 200, &System_RESET);
    // Guard_Init(chassis, 1000, 100, &System_RESET);
    // Guard_Init(UIdraw, 1000 ,100, &System_RESET);
    // Guard_Init(correspondence, 1000 ,100, &System_RESET);
    // Guard_Init(robotid, 20000, 5000, &Error_Enable, true, 30000, &Close_Enable);
}
//���������ʼ��
void Guard_Ctrl::Guard_Init(ID_e Name, uint32_t StartValue, uint32_t MaxValue, void(*errcb)(uint8_t id), bool Close,uint32_t CloseValue, void(*closecb)(uint8_t id))
{
    SG_Structure[Name].Enable = false;//Ĭ�Ϲر�
    SG_Structure[Name].Start = true;//Ĭ�ϴ�
    SG_Structure[Name].Time = 0;
    SG_Structure[Name].Error = 0;
    SG_Structure[Name].Name = Name;
    SG_Structure[Name].StartValue = StartValue;
    SG_Structure[Name].MaxValue = MaxValue;
    SG_Structure[Name].Close = Close;
    SG_Structure[Name].CloseValue = CloseValue;
    if (errcb == NULL)
    {
        SG_Structure[Name].errcallback = &Guard_Return;
    }
    else
    {
        SG_Structure[Name].errcallback = errcb;
    }
    if (closecb == NULL)
    {
        SG_Structure[Name].closecallback = &Guard_Return;
    }
    else
    {
        SG_Structure[Name].closecallback = closecb;
    }
}
void Guard_Ctrl::Guard_Init(ID_e Name, uint32_t StartValue, uint32_t MaxValue, void(*errcb)(uint8_t id))
{
    Guard_Init(Name, StartValue, MaxValue, errcb, false, 0, &Guard_Return);
}
void Guard_Ctrl::Guard_Init(ID_e Name, uint32_t StartValue, uint32_t MaxValue, void(*errcb)(uint8_t id), bool Close)
{//Ĭ��200ms
    Guard_Init(Name, StartValue, MaxValue, errcb, Close, 200, &Guard_Return);
}
//��������ɨ��
void Guard_Ctrl::Guard_Scan(void)
{
    uint8_t i;
    for (i = 0;i < GUARD_TOTAL_NUM;i++)
    {
        if (SG_Structure[i].Start == true && (SG_Structure[i].StartValue != 0))
        {//��ʼ�����
            SG_Structure[i].Error=xTaskGetTickCount()-SG_Structure[i].Time;
            if (SG_Structure[i].Error > SG_Structure[i].StartValue)
            {//��ʱִ�лص�
                SG_Structure[i].errcallback(i);
                if (((int32_t)(SG_Structure[i].Error - SG_Structure[i].StartValue) > SG_Structure[i].CloseValue) && (SG_Structure[i].Close == true))
                {//��ʱ��ȴ��ر�
					SG_Structure[i].Start = false;
					SG_Structure[i].closecallback(i);
				}
			}
		}
		else if ((SG_Structure[i].Enable == true) && (SG_Structure[i].MaxValue != 0))
		{//���м��
			SG_Structure[i].Error=xTaskGetTickCount()-SG_Structure[i].Time;
			if (SG_Structure[i].Error > SG_Structure[i].MaxValue)
			{//��ʱִ�лص�
				SG_Structure[i].errcallback(i);
                if (((int32_t)(SG_Structure[i].Error - SG_Structure[i].MaxValue) > SG_Structure[i].CloseValue) && (SG_Structure[i].Close == true))
                {//��ʱ��ȴ��ر�
					SG_Structure[i].Enable = false;
					SG_Structure[i].closecallback(i);
				}
			}
			else
            {//������ִ�йرջص�
                if (SG_Structure[i].Close == true)
                {
                    SG_Structure[i].closecallback(i);
                }
			}
		}
    }
}
//��������ι��
void Guard_Ctrl::Guard_Feed(ID_e Name)
{
    if (Name == fault)
    {
        return;
    }
    
    Guard.SG_Structure[Name].Enable = true;
    Guard.SG_Structure[Name].Start = false;
    Guard.SG_Structure[Name].Time = xTaskGetTickCount();
}
//��������
void Error_Enable(uint8_t id)
{
    switch (id)
    {
    case serial6:
    {

    }
    break;
    case serial7:
    {
        
    }
    break;
    case robotid:
    {
		pwmWrite(PH6,1000);
    }
    default:
    break;
    }
}
//�رմ�����
void Close_Enable(uint8_t id)
{
    switch (id)
    {
    case robotid:
    {
		pwmWrite(PH6,0);
    }
    break;
    default:
    break;
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

void Guard_Return(uint8_t id)
{
    return;
}

Guard_Ctrl *get_guard_ctrl_pointer()
{
    return &Guard;
}

void IWDG_Init(uint8_t prer, uint16_t rlr)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

    IWDG_SetPrescaler(prer);

    IWDG_SetReload(rlr);

    IWDG_ReloadCounter();

    IWDG_Enable();
}

void IWDG_Feed(void)
{
    IWDG_ReloadCounter();//reload
}

void System_RESET(uint8_t id)
{
    SCB->AIRCR = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
        (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
        SCB_AIRCR_SYSRESETREQ_Msk);
}
