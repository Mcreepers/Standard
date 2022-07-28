#ifndef __APP_MOTOR_H
#define __APP_MOTOR_H

#include "dev_system.h"

//#define USE_PWM_CONTROL_FRIC

#define Chassis_Motor_Numbers      ( 4 )

#define CAN_Gimbal_CAN             ( CAN1_Ctrl )
#define CAN_Chassis_CAN            ( CAN2_Ctrl )
#define CAN_Fric_CAN               ( CAN1_Ctrl )

#define CAN_Gimbal_StdId           ( CAN_GIMBAL_ALL_ID )
#define CAN_Chassis_StdId          ( CAN_CHASSIS_ALL_ID )
#define CAN_Fric_StdId             ( CAN_CHASSIS_ALL_ID )

#define Motor_Ctrl                 ( CAN_Cmd )

#ifdef USE_PWM_CONTROL_FRIC
#define LEFT_FRIC_PWM_PIN          PA2
#define RIGHT_FRIC_PWM_PIN         PA1
#define FRIC_MOTOR_STOP_DUTY_CYCLE 1000
#endif

//底盘电机数据读取
#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }
		
//云台电机数据读取
#define get_gimbal_motor_measuer(ptr, rx_message)                                              \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }
		
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
	
  	CAN_steering_ALL_ID = 0x1FF,
    CAN_6020_M1_ID = 0x205,
    CAN_6020_M2_ID = 0x206,
    CAN_6020_M3_ID = 0x207,
    CAN_6020_M4_ID = 0x208,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_FRIC_MOTOR_ID = 0x202,
    CAN_GIMBAL_ALL_ID = 0x1FF,
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

class Chassis_Motor_Ctrl
{
public:	
  CANctrl *CAN_Chassis;
	motor_measure_t Chassis_Measure[Chassis_Motor_Numbers];

  Chassis_Motor_Ctrl() : CAN_Chassis( &CAN_Chassis_CAN ){}
  const motor_measure_t *Get_Motor_Measure_Pointer( uint8_t i )
	{
	  return &Chassis_Measure[(i & 0x03)];
	}
};
//舵轮中使用云台电机处理函数平替舵向电机
class Gimbal_Motor_Ctrl
{
#if useSteering
  public:
    CANctrl *CAN_Gimbal;
    motor_measure_t Steering_Measure[Chassis_Motor_Numbers];

    Gimbal_Motor_Ctrl() : CAN_Gimbal( &CAN_Gimbal_CAN ){}
    const motor_measure_t *Get_Motor_Measure_Pointer( uint8_t i )
    {
      return &Steering_Measure[(i & 0x03)];
    }
#else
  public:
    CANctrl *CAN_Gimbal;
    motor_measure_t Yaw_Measure;
    //  motor_measure_t Motor_Pitch;
    
    Gimbal_Motor_Ctrl() : CAN_Gimbal( &CAN_Gimbal_CAN ){}
    const motor_measure_t *Get_Motor_Measure_Pointer( void )
    {
      return &Yaw_Measure;
    }
#endif
};

class Fric_Motor_Ctrl
{
public:
#ifndef USE_PWM_CONTROL_FRIC
	CANctrl *CAN_Fric;
  motor_measure_t Fric_Measure;

//	Fric_Motor_Ctrl() : CAN_Fric( &CAN_Fric_CAN ){}
  const motor_measure_t *Get_Motor_Measure_Pointer( void )
	{
	  return &Fric_Measure;
	}
#else
public:
	Fric_Motor_Ctrl() : Left_PWM_Pin( LEFT_FRIC_PWM_PIN ), Right_PWM_Pin( RIGHT_FRIC_PWM_PIN ){}
	void Fric_Motor_Init( void )
	{
	  PWM_Init( Left_PWM_Pin,  ( F_CPU / 1000000 ), 100 ); //100HZ
		PWM_Init( Right_PWM_Pin, ( F_CPU / 1000000 ), 100 ); //100HZ
	}
	
	void Fric_On( uint16_t Speed )
	{
	  pwmWrite( Left_PWM_Pin,  Speed );
		pwmWrite( Right_PWM_Pin, Speed );
	}
	
	void Fric_Off( void )
	{
	  pwmWrite( Left_PWM_Pin,  FRIC_MOTOR_STOP_DUTY_CYCLE );
		pwmWrite( Right_PWM_Pin, FRIC_MOTOR_STOP_DUTY_CYCLE );
	}
private:
  uint8_t Left_PWM_Pin, Right_PWM_Pin;
#endif
};

class CAN_Ctrl
{
public:
  Chassis_Motor_Ctrl Chassis;
  Gimbal_Motor_Ctrl  Gimbal;
  Fric_Motor_Ctrl    Fric;

  void CAN_CMD_RESET_ID( uint32_t StdId );
};
	
void CAN_ALL_Init( void );

extern CAN_Ctrl CAN_Cmd;
extern void CAN1_Hook(CanRxMsg *Rx_Message);
extern void CAN2_Hook(CanRxMsg *Rx_Message);
extern void CAN1_Send(CanRxMsg *Rx_Message);
extern void CAN2_Send(CanRxMsg *Rx_Message);

#endif /* __APP_MOTOR_H */
