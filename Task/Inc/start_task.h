#ifndef __START_TASK_H
#define __START_TASK_H

#include "dev_system.h"

#ifdef __cplusplus
extern "C" {
#endif
	
#include "RTOSsystem_Task.h"
#include "Chassis_Task.h"

void Start_Task(void *pvParameters);
void startTast(void);
	
#ifdef __cplusplus
}
#endif

#endif /* __START_TASK_H */
