#include "start_task.h"

/*创建任务*/
#define START_TASK_PRIO 1
#define START_STK_SIZE  256
static TaskHandle_t StartTask_Handler;

/*系统提示任务*/
#define RTOSsystem_TASK_PRIO     2
#define RTOSsystem_STK_SIZE      64
TaskHandle_t RTOSsystem_Handler;


void startTast(void){
	 xTaskCreate((TaskFunction_t)Start_Task,           //任务函数
                (const char *)"Start_Task",          //任务名称
                (uint16_t)START_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)START_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&StartTask_Handler); //任务句柄
}
/*执行任务创建*/
void Start_Task(void *pvParameters){
	 taskENTER_CRITICAL();//进入临阶段
	
			 xTaskCreate((TaskFunction_t)RTOSsystem_Task,        //创建RTOS信息提示任务
                (const char *)"RTOSsystem_Task",
                (uint16_t)RTOSsystem_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)RTOSsystem_TASK_PRIO,
                (TaskHandle_t *)&RTOSsystem_Handler);						
							
	 vTaskDelete(StartTask_Handler); //删除开始任务
	 taskEXIT_CRITICAL();            //退出临界区
}
