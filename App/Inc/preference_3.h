#define useSteering

#define JUDGE_SERIAL Serial7
#define JUDGE_SERIAL_BAUD 115200
//(光改上面的id不能直接使用)
#define GIMBAL_SERIAL Serial6
#define GIMBAL_SERIAL_BAUD 115200
//发送
#define GIMBAL_SERIAL_HEADER 0xff
#define GIMBAL_SERIAL_TAIL 0xfe

//无帧尾则为 NULL  接收
#define Serial3_Data_Header 0xff
#define Serial3_Data_tail 0xfe
#define Serial6_Data_Header 0xff
#define Serial6_Data_tail 0xfe
#define Serial7_Data_Header 0xff
#define Serial7_Data_tail 0xfe
#define Serial8_Data_Header 0xff
#define Serial8_Data_tail 0xfe

#define Gimbal_Motor_Yaw_Offset_ECD 3243// 5 7631

//选择遥控器控制模式(三挡位功能如下0:不跟随 跟随 小陀螺 1:不跟随 视觉 视觉发弹)
#define RC_CONTRAL_MODE 0
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.003030303f//0.0015f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.003030303f//0.0015f
//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f

//等级速度对应表
#define CHASSIS_SPEED_GEAR_0 (0.5f, 1.f, 1.f)
#define CHASSIS_SPEED_GEAR_1 (1.f, 1.5f, 1.5f)
#define CHASSIS_SPEED_GEAR_2 (2.f, 2.5f, 2.f)
#define CHASSIS_SPEED_GEAR_3 (3.f, 3.5f, 2.f)

//这两个宏用于低通滤波//值越大底盘越软
#define CHASSIS_ACCEL_X_NUM 0.5f
#define CHASSIS_ACCEL_Y_NUM 0.5f
#define CHASSIS_ACCEL_Z_NUM 0.15f
//遥控器死区
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 10000.0f

//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//底盘电机最大速度
#define MAX_WHEEL_SPEED 10.0f//4.0
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 10.0f//2.9
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 10.0f//2.9
//底盘运动过程最大旋速度
#define NORMAL_MAX_CHASSIS_SPEED_Z 10.0f//2.9

//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define CHASSIS_WZ_SET_SCALE 0.1f

//摇摆原地不动摇摆最大角度(rad)
#define SWING_NO_MOVE_ANGLE 0.1f
//摇摆过程底盘运动最大角度(rad)
#define SWING_MOVE_ANGLE 0.5f

//底盘功率速度控制PID
#define VELOCILY_SPEED_PID_KP 0.1f
#define VELOCILY_SPEED_PID_KI 0.1f
#define VELOCILY_SPEED_PID_KD 0.0f
#define VELOCILY_SPEED_PID_MAX_OUT 5.0f
#define VELOCILY_SPEED_PID_MAX_IOUT 1.0f

//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 1.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 1.1f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.001f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 2000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 1000.0f


//底盘6020ecd偏差值
#define MOTOR_6020_1_offset 2082
#define MOTOR_6020_2_offset 719
#define MOTOR_6020_3_offset 7511
#define MOTOR_6020_4_offset 3450
//底盘6020电机角度环PID
#define M6020_MOTOR_ANGLE_PID_KP 10.0f
#define M6020_MOTOR_ANGLE_PID_KI 0.0f
#define M6020_MOTOR_ANGLE_PID_KD 1.0f
#define M6020_MOTOR_ANGLE_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M6020_MOTOR_ANGLE_PID_MAX_IOUT 2000.0f
//底盘6020电机速度环PID
#define M6020_MOTOR_SPEED_PID_KP 2.0f
#define M6020_MOTOR_SPEED_PID_KI 0.005f
#define M6020_MOTOR_SPEED_PID_KD 0.0f
#define M6020_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M6020_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
