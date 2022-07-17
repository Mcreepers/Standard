#include "dev_system.h"

#include "start_task.h"
#include "Guard_Task.h"
#include "app_power_ctrl.h"

#include "protocol_dbus.h"
#include "protocol_judgement.h"

QueueHandle_t Message_Queue;   		//消息队列句柄
QueueHandle_t Guard_Queue;
QueueHandle_t Error_Queue;

void SoftWareInit(void)
{
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	delay_init((uint32_t )1000);
	power_ctrl_configuration();
	
	Message_Queue = xQueueCreate(Message_Q_NUM, sizeof(uint8_t));
	Guard_Queue = xQueueCreate(5, sizeof(uint8_t));
	Error_Queue = xQueueCreate(Message_Q_NUM, sizeof(uint8_t));
	
	Serial3.Serial_Init(115200);
	//   Serial8.Serial_Init( 115200, SERIAL_8N1 );
	usart8_init();
	Serial6.Serial_Init(115200, 6, 2);
	Serial7.Serial_Init( 115200, SERIAL_8N1, 6, 2 ); //哪种初始化方式都可以

	for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
	{
		power_ctrl_on(i);
		Delay_us(709);//此延时为宏定义，非阻塞延时统一使用Delay_us，Delay_ms，操作系统延时使用osDelay
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

