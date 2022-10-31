#include "start_task.h"
#include "Message_Task.h"
#include "Chassis_Task.h"

#include "UIDraw_Task.h"
#include "Guard_Task.h"
#include "Correspondence_Task.h"
/*��������*/
#define START_TASK_PRIO 1//�������ȼ���һ��32λ���ȼ������ȼ�Խ��������ȼ�Խ��
#define START_STK_SIZE  256//�����ջ��С������Ϊ��λ
static TaskHandle_t StartTask_Handler;//������

/*ϵͳ��ʾ����*/
#define RTOSsystem_TASK_PRIO     2
#define RTOSsystem_STK_SIZE      64
TaskHandle_t RTOSsystem_Handler;

/*��������*/
#define Chassis_TASK_PRIO       19
#define Chassis_STK_SIZE        512
TaskHandle_t ChassisTask_Handler;

/*��Ϣ��ȡ����*/
#define Message_TASK_PRIO     30
#define Message_STK_SIZE      512
TaskHandle_t MessageTask_Handler;

/*����UI����*/
#define UIDraw_TASK_PRIO     5
#define UIDraw_STK_SIZE      512
TaskHandle_t UIDrawTask_Handler;  

/*��������*/
#define Guard_Task_PRIO     28
#define Guard_STK_SIZE      512
TaskHandle_t GuardTask_Handler;  

/*ͨ������*/
#define Correspondence_TASK_PRIO     10
#define Correspondenced_STK_SIZE      512
TaskHandle_t CorrespondenceTask_Handler;

void startTast(void)
{
	 xTaskCreate((TaskFunction_t)Start_Task,           //������
                (const char *)"Start_Task",          //��������
                (uint16_t)START_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)START_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&StartTask_Handler); //������
}
/*ִ�����񴴽�*/
void Start_Task(void *pvParameters){
	 taskENTER_CRITICAL();//�����ٽ׶�
	
			 xTaskCreate((TaskFunction_t)RTOSsystem_Task,//����RTOS��Ϣ��ʾ����
                (const char *)"RTOSsystem_Task",
                (uint16_t)RTOSsystem_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)RTOSsystem_TASK_PRIO,
                (TaskHandle_t *)&RTOSsystem_Handler);						

			 xTaskCreate((TaskFunction_t)Message_Task,//������Ϣ��ȡ����
                (const char *)"Message_Task",
                (uint16_t)Message_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Message_TASK_PRIO,
                (TaskHandle_t *)&MessageTask_Handler);
				
			 xTaskCreate((TaskFunction_t)Chassis_Task,//��������������
                (const char *)"Chassis_Task",
                (uint16_t)Chassis_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Chassis_TASK_PRIO,
                (TaskHandle_t *)&ChassisTask_Handler);						

			 xTaskCreate((TaskFunction_t)UIDraw_Task,//��������UI����
               (const char *)"UIDraw_Task",
               (uint16_t)UIDraw_STK_SIZE,
               (void *)NULL,
               (UBaseType_t)UIDraw_TASK_PRIO,
               (TaskHandle_t *)&UIDrawTask_Handler);						

			 xTaskCreate((TaskFunction_t)Guard_Task,//������������
                (const char *)"Correspondence_Task",
                (uint16_t)Guard_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Guard_Task_PRIO,
                (TaskHandle_t *)&GuardTask_Handler);						

			 xTaskCreate((TaskFunction_t)Correspondence_Task,//����ͨ������
               (const char *)"Correspondence_Task",
               (uint16_t)Correspondenced_STK_SIZE,
               (void *)NULL,
               (UBaseType_t)Correspondence_TASK_PRIO,
               (TaskHandle_t *)&CorrespondenceTask_Handler);						
            
            vTaskDelete(StartTask_Handler); //ɾ����ʼ����
	        taskEXIT_CRITICAL();            //�˳��ٽ���
}
