#ifndef __APP_MOTOR_H
#define __APP_MOTOR_H

#include "dev_system.h"

//#define USE_PWM_CONTROL_FRIC

#ifdef USE_PWM_CONTROL_FRIC
#define LEFT_FRIC_PWM_PIN          PA2
#define RIGHT_FRIC_PWM_PIN         PA1
#define FRIC_MOTOR_STOP_DUTY_CYCLE 1000
#endif

//大疆电机数据读取
#define get_motor_measure(ptr, rx_message)                                                \
{                                                                                         \
	(ptr)->last_ecd = (ptr)->ecd;                                                         \
	(ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);          \
	(ptr)->speed_rpm = (int16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
	(ptr)->given_current = (int16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
	(ptr)->temperate = (rx_message)->Data[6];                                             \
}

typedef enum
{
	CAN_DJI_Motor_Group1_ID = 0x1ff,
	CAN_DJI_Motor_Group2_ID = 0x200,
	CAN_DJI_Motor_Group3_ID = 0x2ff,

	CAN_DJI_Motor1_ID = 0x201,
	CAN_DJI_Motor2_ID = 0x202,
	CAN_DJI_Motor3_ID = 0x203,
	CAN_DJI_Motor4_ID = 0x204,

	CAN_DJI_Motor5_ID = 0x205,
	CAN_DJI_Motor6_ID = 0x206,
	CAN_DJI_Motor7_ID = 0x207,
	CAN_DJI_Motor8_ID = 0x208,

	CAN_DJI_Motor9_ID = 0x209,
	CAN_DJI_Motor10_ID = 0x210,
	CAN_DJI_Motor11_ID = 0x211,

	CAN_CAP_GET_ID = 0x301,
	CAN_CAP_SENT_ID = 0x311,
} can_msg_id_e;

//rm电机统一数据结构体
typedef struct
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	int16_t last_ecd;
	uint8_t cnt;
	uint16_t offset;
	uint16_t angle_ecd;
	uint8_t mescnt;
} motor_measure_t;
//6020云台电机数据结构体
typedef struct
{
	int fdbPosition;        //电机的编码器反馈值
	int last_fdbPosition;   //电机上次的编码器反馈值
	int bias_position;      //机器人初始状态电机位置环设定值
	int real_current;         //实际电流
	int given_current;       //给定电流
	int round;              //电机转过的圈数
	int real_position;      //过零处理后的电机转子位置
	int last_real_position;//上次过零处理后的电机转子位置	
}gimbal_measure_t;

class Motor_CAN_Ctrl
{
public:
	Motor_CAN_Ctrl(CAN_TypeDef *CANx_, uint32_t StdID_, uint8_t Num_)
	{
		this->CANx = CANx_;
		this->StdID = StdID_;
		this->Num = Num_;
		Motor_Measure = new motor_measure_t[Num_];
	}
	const motor_measure_t *Get_Motor_Measure_Pointer(uint8_t i)
	{
		return &Motor_Measure[i];
	}
	void GetData(CAN_TypeDef *CANx_, uint32_t &StdID_, uint8_t &Num_)
	{
		CANx_ = this->CANx;
		StdID_ = this->StdID;
		Num_ = this->Num;
	}
	motor_measure_t *GetData(uint8_t i)
	{
		return &Motor_Measure[(i & 3)];
	}
private:
	CAN_TypeDef *CANx;
	uint16_t StdID;
	uint8_t Num;
	motor_measure_t *Motor_Measure;
};

class Motor_PWM_Ctrl
{
public:
	Motor_PWM_Ctrl(uint8_t Pin_):Pin(Pin_) {}
	void Init()
	{
		PWM_Init(Pin, (F_CPU / 1000000), 100);
	}
	void Open()
	{//Snail电机需要先启动,数据仅参考
		pwmWrite(Pin, 1500);
	}
	void Run(uint16_t Speed)
	{
		pwmWrite(Pin, Speed);
	}
	void Close()
	{//数据仅参考
		pwmWrite(Pin, 1000);
	}
private:
	uint16_t Pin;
};

class CAN_Ctrl
{
public:
	CAN_Ctrl()
	{
		Chassis = new Motor_CAN_Ctrl(CAN1, CAN_DJI_Motor_Group2_ID, 4);
		Gimbal = new Motor_CAN_Ctrl(CAN2, CAN_DJI_Motor_Group1_ID, 4);
	}
	Motor_CAN_Ctrl *Chassis;
	Motor_CAN_Ctrl *Gimbal;

	void SendData(CAN_TypeDef *CANx, uint32_t StdID, const void *buf, uint8_t len);
	void SendData(Motor_CAN_Ctrl *Motor, int16_t Motor1, int16_t Motor2, int16_t Motor3, int16_t Motor4);
	void SendData(Motor_CAN_Ctrl *Motor, int16_t Motor1, int16_t Motor2, int16_t Motor3);
	void SendData(Motor_CAN_Ctrl *Motor, int16_t Motor1, int16_t Motor2);
	void SendData(Motor_CAN_Ctrl *Motor, int16_t Motor1);

	void CAN_CMD_RESET_ID(Motor_CAN_Ctrl *Motor);
private:
	void SendData(CANctrl *CANx_Ctrl, uint32_t StdID, const void *buf, uint8_t len);
};

void CAN_ALL_Init(void);

extern CAN_Ctrl CAN_Cmd;
extern void CAN1_Hook(CanRxMsg *Rx_Message);
extern void CAN2_Hook(CanRxMsg *Rx_Message);
extern void CAN1_Send(CanRxMsg *Rx_Message);
extern void CAN2_Send(CanRxMsg *Rx_Message);

#endif /* __APP_MOTOR_H */
