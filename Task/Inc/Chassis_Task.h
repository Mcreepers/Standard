
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
//ѡ��ң��������ģʽ���л��ƶ������
#define MOVE_OR_SHOOT 0
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.001f//0.0015f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.001f//0.0015f
//�������yawģʽ�£�ң������yawң�ˣ�max 660�����ӵ�����Ƕȵı���
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN 0.01f

//�����������ڵ�ͨ�˲�//ֵԽ�����Խ��
#define CHASSIS_ACCEL_X_NUM 0.5f
#define CHASSIS_ACCEL_Y_NUM 0.5f
#define CHASSIS_ACCEL_Z_NUM 0.15f
//ң��������
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002
//�����������Ƶ�ʣ���δʹ�������
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 10000.0f

//��������
#define POWER_FIRST 1 
//Ѫ������
#define HP_FIRST 0 

#define FRONT_ECD 887
#define BACK_ECD  4974
#define LEFT_ECD  2935
#define RIGHT_ECD 7034

#define FRONT_LEFT_ECD  3977
#define FRONT_RIGHT_ECD 2050
#define BACK_RIGHT_ECD  6022
#define BACK_LEFT_ECD   7930


//m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//���̵������ٶ�
#define MAX_WHEEL_SPEED 10.0f//4.0
//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 10.0f//2.9
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 10.0f//2.9
//�����˶�����������ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Z 10.0f//2.9

//����������ת�ٶȣ�����ǰ�������ֲ�ͬ�趨�ٶȵı�����Ȩ 0Ϊ�ڼ������ģ�����Ҫ����
#define CHASSIS_WZ_SET_SCALE 0.1f

//ҡ��ԭ�ز���ҡ�����Ƕ�(rad)
#define SWING_NO_MOVE_ANGLE 0.1f
//ҡ�ڹ��̵����˶����Ƕ�(rad)
#define SWING_MOVE_ANGLE 0.5f

//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

#if useSteering
//����6020ecdƫ��ֵ
#define MOTOR_6020_1_offset 2082
#define MOTOR_6020_2_offset 719
#define MOTOR_6020_3_offset 7511
#define MOTOR_6020_4_offset 3450

//����6020����ǶȻ�PID
#define M6020_MOTOR_ANGLE_PID_KP 10.0f
#define M6020_MOTOR_ANGLE_PID_KI 0.0f
#define M6020_MOTOR_ANGLE_PID_KD 1.0f
#define M6020_MOTOR_ANGLE_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M6020_MOTOR_ANGLE_PID_MAX_IOUT 2000.0f
//����6020����ٶȻ�PID
#define M6020_MOTOR_SPEED_PID_KP 2.0f
#define M6020_MOTOR_SPEED_PID_KI 0.005f
#define M6020_MOTOR_SPEED_PID_KD 0.0f
#define M6020_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M6020_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
#endif
//������ת����PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 0.5f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.001f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 2000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 1000.0f


typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;

} Chassis_Motor_t;//���̽��ձ���������


#if useSteering
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
  void error_behaviour_control_set(void);
#if useSteering
  Chassis_Steering_t Steering[4];
  chassis_steering_mode_e Steering_Mode;

  PidTypeDef  steering_Speed_Pid[4];
  PidTypeDef  steering_Angle_Pid[4];
#endif
  private:
  void RC_to_Control(fp32 *vx_set, fp32 *vy_set);
  void Behaviour_Control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set);
  void Vector_to_Wheel_Speed(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);
#if  useSteering
  void Steering_Behaviour_Control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);
  void Steering_Round_Calc(void);
  void Steering_Mode_Control(void);
#endif
};

extern void System_Reset(void);
extern Chassis_Ctrl Chassis;
extern fp32 motor_ecd_to_relative_ecd(uint16_t angle, uint16_t offset_ecd);
extern const Chassis_Ctrl_Flags_t *get_chassis_flag_control_point(void);
extern const chassis_mode_e *get_chassis_mode_control_point(void);
extern const Chassis_Velocity_t *get_chassis_velocity_control_point(void);

#endif /* __CHASSIS_TASK_H */
