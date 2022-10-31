#include "start_task.h"
#include "Message_Task.h"
#include "Chassis_Task.h"

#include "UIDraw_Task.h"
#include "Guard_Task.h"
#include "Correspondence_Task.h"
/*创建任务*/
#define START_TASK_PRIO 1//任务优先级，一共32位优先级，优先级越大代表优先级越高
#define START_STK_SIZE  256//任务堆栈大小，以字为单位
static TaskHandle_t StartTask_Handler;//任务句柄

/*系统提示任务*/
#define RTOSsystem_TASK_PRIO     2
#define RTOSsystem_STK_SIZE      64
TaskHandle_t RTOSsystem_Handler;

/*底盘任务*/
#define Chassis_TASK_PRIO       19
#define Chassis_STK_SIZE        512
TaskHandle_t ChassisTask_Handler;

/*信息获取任务*/
#define Message_TASK_PRIO     30
#define Message_STK_SIZE      512
TaskHandle_t MessageTask_Handler;

/*绘制UI任务*/
#define UIDraw_TASK_PRIO     5
#define UIDraw_STK_SIZE      512
TaskHandle_t UIDrawTask_Handler;  

/*警戒任务*/
#define Guard_Task_PRIO     28
#define Guard_STK_SIZE      512
TaskHandle_t GuardTask_Handler;  

/*通信任务*/
#define Correspondence_TASK_PRIO     10
#define Correspondenced_STK_SIZE      512
TaskHandle_t CorrespondenceTask_Handler;

void startTast(void)
{
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
	
			 xTaskCreate((TaskFunction_t)RTOSsystem_Task,//创建RTOS信息提示任务
                (const char *)"RTOSsystem_Task",
                (uint16_t)RTOSsystem_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)RTOSsystem_TASK_PRIO,
                (TaskHandle_t *)&RTOSsystem_Handler);						

			 xTaskCreate((TaskFunction_t)Message_Task,//创建信息获取任务
                (const char *)"Message_Task",
                (uint16_t)Message_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Message_TASK_PRIO,
                (TaskHandle_t *)&MessageTask_Handler);
				
			 xTaskCreate((TaskFunction_t)Chassis_Task,//创建底盘主任务
                (const char *)"Chassis_Task",
                (uint16_t)Chassis_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Chassis_TASK_PRIO,
                (TaskHandle_t *)&ChassisTask_Handler);						

			 xTaskCreate((TaskFunction_t)UIDraw_Task,//创建绘制UI任务
               (const char *)"UIDraw_Task",
               (uint16_t)UIDraw_STK_SIZE,
               (void *)NULL,
               (UBaseType_t)UIDraw_TASK_PRIO,
               (TaskHandle_t *)&UIDrawTask_Handler);						

			 xTaskCreate((TaskFunction_t)Guard_Task,//创建警戒任务
                (const char *)"Correspondence_Task",
                (uint16_t)Guard_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Guard_Task_PRIO,
                (TaskHandle_t *)&GuardTask_Handler);						

			 xTaskCreate((TaskFunction_t)Correspondence_Task,//创建通信任务
               (const char *)"Correspondence_Task",
               (uint16_t)Correspondenced_STK_SIZE,
               (void *)NULL,
               (UBaseType_t)Correspondence_TASK_PRIO,
               (TaskHandle_t *)&CorrespondenceTask_Handler);						
            
            vTaskDelete(StartTask_Handler); //删除开始任务
	        taskEXIT_CRITICAL();            //退出临界区
}
