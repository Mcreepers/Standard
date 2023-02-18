#ifndef __APP_PREFERENCE_H
#define __APP_PREFERENCE_H

//ºöÂÔ¾¯¸æ£¬°ÚÀÃ
#pragma diag_suppress 117
#pragma diag_suppress 550
#pragma diag_suppress 3337

//id´íÎó·äÃùÆ÷¾¯¸æ
#define RobotID 3

//#warning RobotIDÎ»ÖÃÌáÊ¾, Çë×ÔÐÐ×¢ÊÍ

#ifndef RobotID
#define RobotID 100
#endif // !RobotID

#if RobotID == 1
#include "preference_1.h"
#elif RobotID == 2
#include "preference_2.h"
#elif RobotID == 3
#include "preference_3.h"
#elif RobotID == 4
#include "preference_4.h"
#elif RobotID == 5
#include "preference_5.h"
#elif RobotID == 6
#include "preference_6.h"
#elif RobotID == 7
#include "preference_7.h"
#elif RobotID == 8
#include "preference_8.h"
#endif

#ifndef useMecanum
#define useMecanum
#endif
#ifdef useSteering
#undef useMecanum
#endif

typedef enum
{
	CanData1 = 0x00,
	CanData2,
	SerialData3,
	SerialData6,
	SerialData7,
	SerialData8,
    RC_Data,
    MessageData,
    ChassisData,
	UIdrawData,
	CorrespondenceData,
	SupercapData,
	RobotId,
	FaultData,
	ID_e_count
}ID_e;

struct ID_Data_t
{
	ID_e Data_ID;
	void *Data_Ptr;
};

extern ID_Data_t ID_Data[ID_e_count];

void Prefence_Init(void);
#endif 
