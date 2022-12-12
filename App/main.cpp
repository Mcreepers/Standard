#include "dev_system.h"

#include "start_task.h"
#include "app_power_ctrl.h"

#include "protocol_dbus.h"
#include "protocol_judgement.h"

QueueHandle_t Message_Queue;   		//��Ϣ���о��

void SoftWareInit(void)
{
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	delay_init((uint32_t )1000);
	power_ctrl_configuration();
	
	Message_Queue = xQueueCreate(Message_Q_NUM, sizeof(ID_Data_t));

	Prefence_Init();
	PWM_Init(PH6, 30000, 66);	//buzzer
	pwmWrite(PH6,0);

	for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
	{
		power_ctrl_on(i);
		Delay_us(709);//����ʱΪ�궨�壬��������ʱͳһʹ��Delay_us��Delay_ms������ϵͳ��ʱʹ��osDelay
	}

	remote_control_init();
}

int main(void)
{	
	  SoftWareInit();
	
	  startTast();
	  vTaskStartScheduler();

	  while(1)
	  { 

	  }
}

