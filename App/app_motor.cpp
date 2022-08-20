#include "app_motor.h"

CAN_Ctrl CAN_Cmd;

static void CAN2_Hook( CanRxMsg *Rx_Message )
{
  switch( Rx_Message->StdId )
  {
		
    case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    {
        static uint8_t i = 0;
        //处理电机ID号
        i = Rx_Message->StdId - CAN_3508_M1_ID;
        //处理电机数据宏函数
        get_motor_measure( &CAN_Cmd.Chassis.Chassis_Measure[i], Rx_Message );
        break;
    }
    default:
    {
        break;
    }
  }
}

static void CAN1_Hook( CanRxMsg *Rx_Message )
{
  switch (Rx_Message->StdId)
  {
    case CAN_YAW_MOTOR_ID:
    {
      //处理电机数据宏函数
      get_gimbal_motor_measuer( &CAN_Cmd.Gimbal.Yaw_Measure, Rx_Message );
      break;
    }
    case CAN_FRIC_MOTOR_ID:
    {		
		  get_motor_measure( &CAN_Cmd.Fric.Fric_Measure, Rx_Message );//底盘大弹丸拨弹轮电机
      break;
    }
    default:
    {
      break;
    }
  }
}

void CAN_ALL_Init( void )
{
	//必须先初始化CAN1，在初始化CAN2
	CAN1_Ctrl.CANx_Init();
	CAN2_Ctrl.CANx_Init();
	
	CAN1_Ctrl.attachInterrupt( CAN1_Hook );
	CAN2_Ctrl.attachInterrupt( CAN2_Hook );
}
