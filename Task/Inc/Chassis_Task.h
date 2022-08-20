#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "dev_system.h"
#include "protocol_dbus.h"

#include "app_motor.h"

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

#define useMecanum

//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357

//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 1
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 0
//������ģʽ�£�����ͨ��ң����������ת
#define CHASSIS_WZ_CHANNEL 2

//ѡ�����״̬ ����ͨ����
#define MODE_CHANNEL 0
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.005f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.005f
//�������yawģʽ�£�ң������yawң�ˣ�max 660�����ӵ�����Ƕȵı���
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN 0.01f

//�����������ڵ�ͨ�˲�//ֵԽ�����Խ��
#define CHASSIS_ACCEL_X_NUM 0.5f
#define CHASSIS_ACCEL_Y_NUM 0.5f
#define CHASSIS_ACCEL_Z_NUM 0.15f
//ң��������
#define CHASSIS_RC_DEADLINE 20

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


//����ǰ�����ҿ��ư���+���ټ�

//��������
#define POWER_FIRST 1 
//Ѫ������
#define HP_FIRST 0 

#define CHASSIS_FRONT_KEY        KEY_PRESSED_OFFSET_S
#define CHASSIS_BACK_KEY         KEY_PRESSED_OFFSET_W
#define CHASSIS_LEFT_KEY         KEY_PRESSED_OFFSET_D
#define CHASSIS_RIGHT_KEY        KEY_PRESSED_OFFSET_A

#define CHASSIS_SPEEDUP_KEY      KEY_PRESSED_OFFSET_SHIFT
#define CHASSIS_SPEEDDOWN_KEY    KEY_PRESSED_OFFSET_CTRL

#define FOLLOW 1
#define NOTFOLLOW 0
#define CHASSIS_STOP_KEY         KEY_PRESSED_OFFSET_C
#define CHASSIS_MODE_SWITH_KEY   KEY_PRESSED_OFFSET_X

#define CHASSIS_SPIN_KEY         KEY_PRESSED_OFFSET_G
#define CAPACITANCE_MODE_KEY     KEY_PRESSED_OFFSET_B   //��������
#define CHASSIS_UPGRADE_KEY      KEY_PRESSED_OFFSET_Z

#define CHASSIS_UI_FOLLOW_KEY    KEY_PRESSED_OFFSET_F   
#define CHASSIS_UI_SIZE_KEY      KEY_PRESSED_OFFSET_V  
#define CHASSIS_UI_COLOR_KEY     KEY_PRESSED_OFFSET_R

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

//������ת����PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 5.5f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 4.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} Chassis_Motor_t;//���̽��ձ���������

typedef struct
{
  bool Spin_Flag;
  bool RC_Flag;
  bool Vision_Flag;
} Chassis_Ctrl_Flags_t;//���̽��ձ���������

//typedef enum
//{
//	CHASSIS_NO_MOVE=0,
//	CHASSIS_FOLLOW_YAW,//������̨
//  CHASSIS_LITTLE_TOP,//С����
//  CHASSIS_VISION,
//} chassis_mode_e;//���̹���״̬

typedef struct{
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
} Chassis_Velocity_t;

class Chassis_Ctrl{
public:
    const RC_ctrl_t *RC_Ptr;

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

    void Chassis_Init( void );
private:
	  void chassis_rc_to_control_vector( fp32 *vx_set, fp32 *vy_set );
};

#endif /* __CHASSIS_TASK_H */
