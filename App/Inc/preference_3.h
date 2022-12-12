#define useSteering

#define JUDGE_SERIAL Serial7
#define JUDGE_SERIAL_BAUD 115200
//(��������id����ֱ��ʹ��)
#define GIMBAL_SERIAL Serial6
#define GIMBAL_SERIAL_BAUD 115200
//����
#define GIMBAL_SERIAL_HEADER 0xff
#define GIMBAL_SERIAL_TAIL 0xfe

//��֡β��Ϊ NULL  ����
#define Serial3_Data_Header 0xff
#define Serial3_Data_tail 0xfe
#define Serial6_Data_Header 0xff
#define Serial6_Data_tail 0xfe
#define Serial7_Data_Header 0xff
#define Serial7_Data_tail 0xfe
#define Serial8_Data_Header 0xff
#define Serial8_Data_tail 0xfe

#define Gimbal_Motor_Yaw_Offset_ECD 3243// 5 7631

//ѡ��ң��������ģʽ(����λ��������0:������ ���� С���� 1:������ �Ӿ� �Ӿ�����)
#define RC_CONTRAL_MODE 0
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.003030303f//0.0015f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.003030303f//0.0015f
//�������yawģʽ�£�ң������yawң�ˣ�max 660�����ӵ�����Ƕȵı���
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN 0.01f

//�ȼ��ٶȶ�Ӧ��
#define CHASSIS_SPEED_GEAR_0 (0.5f, 1.f, 1.f)
#define CHASSIS_SPEED_GEAR_1 (1.f, 1.5f, 1.5f)
#define CHASSIS_SPEED_GEAR_2 (2.f, 2.5f, 2.f)
#define CHASSIS_SPEED_GEAR_3 (3.f, 3.5f, 2.f)

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

//���̹����ٶȿ���PID
#define VELOCILY_SPEED_PID_KP 0.1f
#define VELOCILY_SPEED_PID_KI 0.1f
#define VELOCILY_SPEED_PID_KD 0.0f
#define VELOCILY_SPEED_PID_MAX_OUT 5.0f
#define VELOCILY_SPEED_PID_MAX_IOUT 1.0f

//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 1.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//������ת����PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 1.1f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.001f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 2000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 1000.0f


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
