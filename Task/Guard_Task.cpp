#include "Guard_Task.h"
#include "Message_Task.h"
#include "timers.h"
Guard_Ctrl Guard;
Guard_ID_t Guard_ID;
Error_Flags_t Error_Flag;
//警戒任务
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
//警戒任务开始
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
//警戒任务初始化
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

//警戒任务扫描（在软件定时器中执行）
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
//警戒任务喂狗
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
//向错误消息队列发送消息
void Error_Send(void)
{
    xQueueOverwriteFromISR(Error_Queue, &Guard.ID, 0);
}
//警戒任务默认回调函数
void Guard_Return(void)
{
    return;
}
//软件定时器
void Guard_Scan_Time(TimerHandle_t xTimer)
{
    Guard.Guard_Scan();
}
//初始化独立看门狗
//prer:分频数:0~7(只有低3位有效!)
//rlr:自动重装载值,0~0XFFF.
//分频因子=4*2^prer.但最大值只能是256!
//rlr:重装载寄存器值:低11位有效.
//时间计算(大概):Tout=((4*2^prer)*rlr)/32 (ms).
void IWDG_Init(uint8_t prer,uint16_t rlr)
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
void System_Reset(void)
{
	SCB->AIRCR  = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos)    |
		   (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
			SCB_AIRCR_SYSRESETREQ_Msk);
}
