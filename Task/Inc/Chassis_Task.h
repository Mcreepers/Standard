
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

//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 3
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 2
//������ģʽ�£�����ͨ��ң����������ת
#define CHASSIS_WZ_CHANNEL 1

//ѡ�����״̬ ����ͨ����
#define CHANNEL_LEFT  1
#define CHANNEL_RIGHT  0

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;

} Chassis_Motor_t;//���̽��ձ���������


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
} Chassis_Steering_t;//���̽��ձ���������
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
} Chassis_Ctrl_Flags_t;//���̿��Ʊ�־λ

typedef struct
{
  bool RC_Flag;
  bool Gimbal_Flag;
}
Chassis_Error_Flags_t;
typedef enum
{
  CHASSIS_NO_MOVE = 0,
  CHASSIS_FOLLOW_YAW,//������̨
  CHASSIS_NO_FOLLOW_YAW,//��������̨
  CHASSIS_LITTLE_TOP,//С����
  CHASSIS_VISION,
} chassis_mode_e;//���̹���״̬

typedef enum
{
  STEERING_STOP,//����ֹͣ��ת
  STEERING_NORMAL,//�˶�ģʽ
  STEERING_FOLLOW_GIMBAL,//���ָ�����̨
  STEERING_FOLLOW_CHASSIS,//���ָ������
  STEERING_VECTOR_NO_FOLLOW,//���ֱ����ϴ��趨�Ƕ�
  STEERING_LIMIT,//����������ת��Χ
  STEERING_LITTLE_TOP,//����С����״̬�µĶ���
} chassis_steering_mode_e;//���ֹ���״̬

typedef struct
{
  fp32 vx;      //�����ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy;      //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  fp32 wz;      //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 vx_set;  //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy_set;  //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  fp32 wz_set;  //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s

  fp32 vx_max_speed;  //ǰ����������ٶ� ��λm/s
  fp32 vx_min_speed;  //ǰ��������С�ٶ� ��λm/s
  fp32 vy_max_speed;  //���ҷ�������ٶ� ��λm/s
  fp32 vy_min_speed;  //���ҷ�����С�ٶ� ��λm/s

  uint8_t Speed_Gear;  //�ٶȵ�λ
  fp32    Speed_Set[4];//�����ٶ�
} Chassis_Velocity_t;

class Chassis_Ctrl : public rc_key_c
{
public:
  const RC_ctrl_t *RC_Ptr;
  const fp32 *chassis_yaw_relative_angle;   //����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����
  fp32 chassis_relative_ECD;   //����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����
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
