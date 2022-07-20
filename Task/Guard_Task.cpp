#include "Guard_Task.h"
#include "Message_Task.h"
#include "timers.h"

Guard_Ctrl Guard;
Error_Flags_t Error_Flag;

//警戒任务
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
//警戒任务开始
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
//警戒任务初始化
void Guard_Ctrl::Guard_Init(ID_t Name, uint32_t MaxValue, void(*errcb)(uint8_t id))
{
    SG_Structure[Name].Name = Name;
    SG_Structure[Name].Enable = 0;//默认关闭
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
//警戒任务扫描
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
//警戒任务喂狗
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
//警戒任务使能(feed中的使能只针对已运行任务)
void Guard_Ctrl::Guard_Enable(void)
{
    uint8_t i;
    for (i = 0;i < GUARD_TOTAL_NUM;i++)
    {
        Guard.SG_Structure[i].Enable = 1;
    }
}
//警戒任务默认回调函数
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

//初始化独立看门狗
//prer:分频数:0~7(只有低3位有效!)
//rlr:自动重装载值,0~0XFFF.
//分频因子=4*2^prer.但最大值只能是256!
//rlr:重装载寄存器值:低11位有效.
//时间计算(大概):Tout=((4*2^prer)*rlr)/32 (ms).
void IWDG_Init(uint8_t prer, uint16_t rlr)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //使能对IWDG->PR IWDG->RLR的写

    IWDG_SetPrescaler(prer); //设置IWDG分频系数

    IWDG_SetReload(rlr);   //设置IWDG装载值

    IWDG_ReloadCounter(); //reload

    IWDG_Enable();       //使能看门狗
}

//喂独立看门狗
void IWDG_Feed(void)
{
    IWDG_ReloadCounter();//reload
}

//寄存器软件复位
void System_Reset(uint8_t id)
{
    SCB->AIRCR = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
        (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
        SCB_AIRCR_SYSRESETREQ_Msk);
}
