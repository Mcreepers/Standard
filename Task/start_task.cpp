#include "start_task.h"

/*��������*/
#define START_TASK_PRIO 1
#define START_STK_SIZE  256
static TaskHandle_t StartTask_Handler;

/*ϵͳ��ʾ����*/
#define RTOSsystem_TASK_PRIO     2
#define RTOSsystem_STK_SIZE      64
TaskHandle_t RTOSsystem_Handler;


void startTast(void){
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
	
			 xTaskCreate((TaskFunction_t)RTOSsystem_Task,        //����RTOS��Ϣ��ʾ����
                (const char *)"RTOSsystem_Task",
                (uint16_t)RTOSsystem_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)RTOSsystem_TASK_PRIO,
                (TaskHandle_t *)&RTOSsystem_Handler);						
							
	 vTaskDelete(StartTask_Handler); //ɾ����ʼ����
	 taskEXIT_CRITICAL();            //�˳��ٽ���
}
