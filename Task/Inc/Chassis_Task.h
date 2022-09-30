
#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "dev_system.h"
#include "protocol_dbus.h"

#include "app_motor.h"
#include "Message_Task.h"

#include "algorithm_pid.h"
#include "algorithm_user_lib.h"

#include "drivers_state_machines.h"

#ifdef __cplusplus
extern "C" {
#endif

void Chassis_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 3
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 2
//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 1

//选择底盘状态 开关通道号
#define CHANNEL_LEFT  1
#define CHANNEL_RIGHT  0

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;

} Chassis_Motor_t;//底盘接收编码器数据


#ifdef useSteering
struct Steering_Data_t
{
  fp32 angle;
  fp32 angle_last;
  int8_t angle_round;

  fp32 angle_set;
  fp32 angle_set_last;
  int8_t angle_set_round;
};

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  Steering_Data_t data;

  fp32 accel;
  fp32 speed;
  fp32 speed_set;

  uint32_t offset_ecd;
  
  fp32 angle_real;
  fp32 angle_set_real;

  int16_t give_current;
} Chassis_Steering_t;//底盘接收编码器数据
#endif

typedef struct
{
  bool Spin_Flag;
  bool RC_Flag;
  bool Vision_Flag;
  bool Predict_Flag;
  bool Energy_Flag;
  bool Looding_Flag;
  bool Fric_Flag;
  bool Shoot_Flag;
  bool Shoot_Direction_Flag;
  bool Speed_Up_Flag;
} Chassis_Ctrl_Flags_t;//底盘控制标志位

typedef struct
{
  bool RC_Flag;
  bool Gimbal_Flag;
}
Chassis_Error_Flags_t;
typedef enum
{
  CHASSIS_NO_MOVE = 0,
  CHASSIS_FOLLOW_YAW,//跟随云台
  CHASSIS_NO_FOLLOW_YAW,//不跟随云台
  CHASSIS_LITTLE_TOP,//小陀螺
  CHASSIS_VISION,
} chassis_mode_e;//底盘工作状态

typedef enum
{
  STEERING_STOP,//舵轮停止旋转
  STEERING_NORMAL,//运动模式
  STEERING_FOLLOW_GIMBAL,//舵轮跟随云台
  STEERING_FOLLOW_CHASSIS,//舵轮跟随底盘
  STEERING_VECTOR_NO_FOLLOW,//舵轮保持上次设定角度
  STEERING_LIMIT,//舵轮限制旋转范围
  STEERING_LITTLE_TOP,//底盘小陀螺状态下的舵轮
} chassis_steering_mode_e;//舵轮工作状态

typedef struct
{
  fp32 vx;      //底盘速度 前进方向 前为正，单位 m/s
  fp32 vy;      //底盘速度 左右方向 左为正  单位 m/s
  fp32 wz;      //底盘旋转角速度，逆时针为正 单位 rad/s
  fp32 vx_set;  //底盘设定速度 前进方向 前为正，单位 m/s
  fp32 vy_set;  //底盘设定速度 左右方向 左为正，单位 m/s
  fp32 wz_set;  //底盘设定旋转角速度，逆时针为正 单位 rad/s

  fp32 vx_max_speed;  //前进方向最大速度 单位m/s
  fp32 vx_min_speed;  //前进方向最小速度 单位m/s
  fp32 vy_max_speed;  //左右方向最大速度 单位m/s
  fp32 vy_min_speed;  //左右方向最小速度 单位m/s

  uint8_t Speed_Gear;  //速度档位
  fp32    Speed_Set[4];//底盘速度
} Chassis_Velocity_t;

class Chassis_Ctrl : public rc_key_c
{
public:
  const RC_ctrl_t *RC_Ptr;
  const fp32 *chassis_yaw_relative_angle;   //底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角
  fp32 chassis_relative_ECD;   //底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角
  fp32 chassis_relative_RAD;
  
  Chassis_Motor_t Motor[Chassis_Motor_Numbers];

  PidTypeDef  Speed_Pid[Chassis_Motor_Numbers];
  PidTypeDef  Follow_Gimbal_Pid;
  PidTypeDef  chassis_setangle;
  PidTypeDef  chassis_setangle_gyro;

  first_order_filter_type_t Filter_vx;
  first_order_filter_type_t Filter_vy;
  first_order_filter_type_t Filter_vw;

  Chassis_Velocity_t   Velocity;
  Chassis_Ctrl_Flags_t Flags;
  chassis_mode_e Mode;
  chassis_mode_e Last_Mode;

  void Chassis_Init(void);
  void Feedback_Update(void);
  void Control(void);
  void Behaviour_Mode(void);
  void Control_loop(void);
#ifdef useSteering
  Chassis_Steering_t Steering[4];
  chassis_steering_mode_e Steering_Mode;

  PidTypeDef  steering_Speed_Pid[4];
  PidTypeDef  steering_Angle_Pid[4];
#endif
private:
  Message_Ctrl *Chassis_Message;
  
  void RC_to_Control(fp32 *vx_set, fp32 *vy_set);
  void Behaviour_Control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set);
  void Vector_to_Wheel_Speed(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);
#ifdef  useSteering
  void Steering_Behaviour_Control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);
  void Steering_Round_Calc(void);
  void Steering_Mode_Control(void);
#endif
};

extern void rc_key_v_fresh(RC_ctrl_t *RC);
extern void System_Reset(void);
extern fp32 motor_ecd_to_relative_ecd(uint16_t angle, uint16_t offset_ecd);
Chassis_Ctrl *get_chassis_ctrl_pointer(void);

#endif /* __CHASSIS_TASK_H */
