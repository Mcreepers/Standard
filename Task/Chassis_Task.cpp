#include "Chassis_Task.h"
#include "message_Task.h"
#include "Guard_Task.h"
#include "arm_math.h"

#include "app_motor.h"
#include "dev_can.h"
#include "protocol_dbus.h"

Chassis_Ctrl Chassis;
const Gimbal_Receive_Data_t *Gimbal_chassis;

u8 UIsend = 0;
//�����ٶȻ�pidֵ
const static fp32 Motor_Speed_Pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
//������ת��pidֵ
const static fp32 Chassis_Follow_Gimbal_Pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
const static fp32 chassis_z_order_filter[1] = {CHASSIS_ACCEL_Z_NUM};
#if useSteering
const static fp32 m6020_motor_angle_pid[3] = {M6020_MOTOR_ANGLE_PID_KP, M6020_MOTOR_ANGLE_PID_KI, M6020_MOTOR_ANGLE_PID_KD};
const static fp32 m6020_motor_speed_pid[3] = {M6020_MOTOR_SPEED_PID_KP, M6020_MOTOR_SPEED_PID_KI, M6020_MOTOR_SPEED_PID_KD};
const static fp32 MOTOR_6020_offset[4] = {MOTOR_6020_1_offset, MOTOR_6020_2_offset, MOTOR_6020_3_offset, MOTOR_6020_4_offset};//6020 ecd��ֵ
#endif

void Chassis_Task(void *pvParameters)
{
	CAN_ALL_Init();//���е�����ƶ���һ������
	Chassis.Chassis_Init();
	
	while (1)
	{
		Chassis.Behaviour_Mode();
		if (Chassis.Mode == CHASSIS_NO_MOVE)
		{
			CAN_Cmd.Chassis.CAN_Chassis->SendData(0, 0, 0, 0);
#if useSteering
			CAN_Cmd.Gimbal.CAN_Gimbal->SendData(0, 0, 0, 0);
#endif
		}
		else
		{
			Chassis.Feedback_Update();
			Chassis.Control();
			Chassis.Control_loop();
			
			CAN_Cmd.Chassis.CAN_Chassis->SendData(Chassis.Motor[0].give_current, Chassis.Motor[1].give_current,
				Chassis.Motor[2].give_current, Chassis.Motor[3].give_current);
#if useSteering
			CAN_Cmd.Gimbal.CAN_Gimbal->SendData(Chassis.Steering[0].give_current, Chassis.Steering[1].give_current,
				Chassis.Steering[2].give_current, Chassis.Steering[3].give_current);
#endif
		}
		
        xQueueSend(Message_Queue, &(Message_Data.Data_ID = chassis), 0);
		
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);
	}
}

//���̳�ʼ��
void Chassis_Ctrl::Chassis_Init(void)
{
  	RC_Ptr = get_remote_control_point();
	Gimbal_chassis = get_gimbal_data_point();
	chassis_yaw_relative_angle=&get_gimbal_data_point()->ECD;
	Mode=CHASSIS_NO_MOVE;
	
	for ( uint8_t i = 0; i < 4; i++ )
	{
		Motor[i].chassis_motor_measure = Motor_Ctrl.Chassis.Get_Motor_Measure_Pointer(i);
		PID_Init( &Speed_Pid[i], PID_POSITION, Motor_Speed_Pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT );
	}
#if useSteering/* useSteering */
	for ( uint8_t i = 0; i < 4; i++ )
	{
		Steering[i].chassis_motor_measure = Motor_Ctrl.Gimbal.Get_Motor_Measure_Pointer(i);
		PID_Init( &steering_Angle_Pid[i], PID_POSITION, m6020_motor_angle_pid, M6020_MOTOR_ANGLE_PID_MAX_OUT, M6020_MOTOR_ANGLE_PID_MAX_IOUT );
		PID_Init( &steering_Speed_Pid[i], PID_POSITION, m6020_motor_speed_pid, M6020_MOTOR_SPEED_PID_MAX_OUT, M6020_MOTOR_SPEED_PID_MAX_IOUT );
		Steering[i].offset_ecd = MOTOR_6020_offset[i];
	}
#endif
	//��ʼ����תPID
	PID_Init( &Follow_Gimbal_Pid, PID_POSITION, Chassis_Follow_Gimbal_Pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT );
	//��һ���˲�����б����������
	first_order_filter_init( &Filter_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter );
	first_order_filter_init( &Filter_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter );
	first_order_filter_init( &Filter_vw, CHASSIS_CONTROL_TIME, chassis_z_order_filter );
	
	//��� ��С�ٶ�
	Velocity.vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
	Velocity.vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

	Velocity.vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
	Velocity.vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
	
	Velocity.Speed_Gear = 0;
	Velocity.Speed_Set[0] = Velocity.Speed_Set[1] = Velocity.Speed_Set[2] = 0.2;	
	
	Feedback_Update();
}
//���ݸ���
void Chassis_Ctrl::Feedback_Update( void )
{
	uint8_t i = 0;
	for (i = 0; i < 4; i++)
    {
        //���µ���ٶȣ����ٶ����ٶȵ�PID΢��
        Motor[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * Motor[i].chassis_motor_measure->speed_rpm;
        Motor[i].accel = Speed_Pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
	}
	chassis_relative_ECD = *(chassis_yaw_relative_angle);//�����̨�Ƕ�
	chassis_relative_RAD = chassis_relative_ECD * ECD_TO_PI;
#if useMecanum
    //���µ���ǰ���ٶ� x��ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ
    Velocity.vx = (-Motor[0].speed + Motor[1].speed + Motor[2].speed - Motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    Velocity.vy = (-Motor[0].speed - Motor[1].speed + Motor[2].speed + Motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    Velocity.wz = (-Motor[0].speed - Motor[1].speed - Motor[2].speed - Motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
#elif	useSteering
	for (i = 0; i < 4; i++)
    {
        //���µ���ٶȣ����ٶ����ٶȵ�PID΢��
		Steering[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * Motor[i].chassis_motor_measure->speed_rpm;
		Steering[i].accel = Speed_Pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
		Steering[i].data.angle_last=Steering[i].data.angle;
		Steering[i].data.angle = motor_ecd_to_relative_ecd(Steering[i].chassis_motor_measure->ecd,Steering[i].offset_ecd);
		Steering[i].data.angle_set_last=Steering[i].data.angle_set;
	}
#endif
}
//������Ϊ״̬����
void Chassis_Ctrl::Behaviour_Mode(void)
{
	if (switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT]))
		Flags.RC_Flag = false;
	else Flags.RC_Flag = true;
	//��������
	if (Flags.RC_Flag == false)
	{
		if (read_key(&Key.C,single,true))
		{
			Mode = CHASSIS_NO_MOVE;//���̱��ֲ���
		}
		if (read_key(&Key.X,single,true))
		{
			if (Mode == CHASSIS_NO_MOVE||Mode == CHASSIS_FOLLOW_YAW||Mode == CHASSIS_LITTLE_TOP)
			{
				Mode = CHASSIS_NO_FOLLOW_YAW;//���̲�������̨			
			}
			else if (Mode == CHASSIS_NO_FOLLOW_YAW)
			{
				Mode=CHASSIS_FOLLOW_YAW;//���̸�����̨
			}			
		}
		if (read_key(&Key.G,single,true))
		{//����С����
			if (Mode != CHASSIS_LITTLE_TOP&&Mode != CHASSIS_NO_MOVE)
			{
				Mode = CHASSIS_LITTLE_TOP;
			}
			else if (Mode != CHASSIS_NO_MOVE)
			{
				Mode = CHASSIS_NO_FOLLOW_YAW;
			}
		}
		//�ٶ�ѡ��
		if (read_key(&Key.shift,single,true))
		{//����  SHIFT
			if (Velocity.Speed_Gear < 3)
				Velocity.Speed_Gear++;
		}
		else if (read_key(&Key.ctrl, single,true))
		{//����  CTRL
			if (Velocity.Speed_Gear > 0)
				Velocity.Speed_Gear--;
		}
		//�Ӿ�����
		if (read_key(&Press.R, single, &Flags.Vision_Flag))
		{//Ԥ�⿪��
			read_key(&Key.E, single, &Flags.Predict_Flag);
		}
		//UI���
		if (read_key(&Key.B, single,true))
		{
			UIsend = 10;
			//�������ؿ���
			if ((Flags.Vision_Flag == 1) && (Flags.Energy_Flag == 0))
			{
				Flags.Energy_Flag = 1;
			}
			else if ((Flags.Vision_Flag == 1) && (Flags.Energy_Flag == 1))
			{
				Flags.Energy_Flag = 0;
			}
		}
		//װ������
		read_key(&Key.V, single, &Flags.Looding_Flag);
		//Ħ���ֿ���
		if (read_key(&Key.R, single, &Flags.Fric_Flag))
		{//�����ֿ���
			read_key(&Press.L, even, &Flags.Shoot_Flag);
		}else{
			Flags.Shoot_Flag = 0;
		}
		//���ٿ���
		read_key(&Key.F, single, &Flags.Speed_Up_Flag);
	}
	//ң�ؿ���ģʽ�л�
	if (switch_is_down(RC_Ptr->rc.s[CHANNEL_RIGHT]))
	{
		Mode = CHASSIS_NO_MOVE;
	}
#if MOVE_OR_SHOOT
	if (switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_down(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Mode = CHASSIS_NO_FOLLOW_YAW;
	}
	else if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_mid(RC_Ptr->rc.s[CHANNEL_LEFT])){
		Mode = CHASSIS_FOLLOW_YAW;
	}
	else if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT])){
		Mode = CHASSIS_LITTLE_TOP;
	}
#else
	else if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]))
	{
		Mode = CHASSIS_NO_FOLLOW_YAW;
	}
	if (switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_down(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Flags.Shoot_Flag=0;
		Flags.Fric_Flag=0;
	}
	else if (switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_mid(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Flags.Shoot_Flag = 1;
	}
	else if (switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Flags.Fric_Flag = 1;
	}
#endif
	else if (switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		System_Reset();
	}
	else if (switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_mid(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		UIsend = 10;
	}
}

//ң���������ݴ���ɵ��̵�ǰ��vx�ٶȣ�vy�ٶ�
void Chassis_Ctrl::RC_to_Control( fp32 *vx_set, fp32 *vy_set)
{
	if ( vx_set == NULL || vy_set == NULL )
	{
			return;
	}
	//ң����ԭʼͨ��ֵ
	static int16_t vx_channel, vy_channel;
  	static fp32 vx_set_channel, vy_set_channel;
	
  	if( Velocity.Speed_Gear == 0 ){
	  Velocity.Speed_Set[0] = 0.5;	Velocity.Speed_Set[1] = 1; Velocity.Speed_Set[2] = 1; 
		
	}else if( Velocity.Speed_Gear == 1 ){ 
	  Velocity.Speed_Set[0] = 1;	Velocity.Speed_Set[1] = 1.5; Velocity.Speed_Set[2] = 1.5;
		
	}else if( Velocity.Speed_Gear == 2 ){ 
	  Velocity.Speed_Set[0] = 2; Velocity.Speed_Set[1] = 2; Velocity.Speed_Set[2] = 2; 
		
	}else if( Velocity.Speed_Gear == 3 ){ 
	  Velocity.Speed_Set[0] = 3;	Velocity.Speed_Set[1] = 2.5; Velocity.Speed_Set[2] = 2.5;
		
	}else{
	  Velocity.Speed_Set[0] = 0.2;	Velocity.Speed_Set[1] = 0.2; Velocity.Speed_Set[2] = 0.2; 
	}

	if( Flags.RC_Flag == false )
	{
		//��WDAS����
		if (Chassis.RC_Ptr->key.v & KEY_PRESSED_OFFSET_W||Chassis.RC_Ptr->key.v & KEY_PRESSED_OFFSET_S
			||Chassis.RC_Ptr->key.v & KEY_PRESSED_OFFSET_A||Chassis.RC_Ptr->key.v & KEY_PRESSED_OFFSET_D)
		{
			if(Chassis.RC_Ptr->key.v & KEY_PRESSED_OFFSET_W)
			{
				vx_set_channel = Velocity.Speed_Set[Flags.Speed_Up_Flag];
			}
			if(Chassis.RC_Ptr->key.v & KEY_PRESSED_OFFSET_S)
			{
				vx_set_channel = -Velocity.Speed_Set[Flags.Speed_Up_Flag];
			}
			if(Chassis.RC_Ptr->key.v & KEY_PRESSED_OFFSET_A)
			{
				vy_set_channel = -Velocity.Speed_Set[Flags.Speed_Up_Flag];
			}
			if(Chassis.RC_Ptr->key.v & KEY_PRESSED_OFFSET_D)
			{
				vy_set_channel = Velocity.Speed_Set[Flags.Speed_Up_Flag];
			}
		}
		else//��WSAD������һֱ��ֹ
		{
			vx_set_channel = 0;
			vy_set_channel = 0;
		}
	}
	else
	{
		//��ң���������ݴ������� int16_t yaw_channel,pitch_channel
		rc_deadline_limit( RC_Ptr->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE  );
		rc_deadline_limit( RC_Ptr->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE  );
 
		vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
		vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN;
	}
		 
		
	//ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
	if ( vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN )
    {
        vx_set_channel = 0.0f;
    }

    if ( vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN )
    {
        vy_set_channel = 0.0f;
    }
		
    *vx_set = vx_set_channel;
    *vy_set = -vy_set_channel;
}

void Chassis_Ctrl::Behaviour_Control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set)
{

	fp32 vw_set;

	if (Mode == CHASSIS_NO_MOVE)//����״̬
	{
		*vx_set = 0.0f;
		*vy_set = 0.0f;
		*angle_set = 0.0f;
	}
	else if (Mode == CHASSIS_FOLLOW_YAW)//������̨
	{
		RC_to_Control(vx_set, vy_set);//��ң��ֵת��Ϊ�����趨��

		if (chassis_relative_ECD > 30)// ���800
		{
			PID_Calc(&Follow_Gimbal_Pid, chassis_relative_ECD, 50, 0);
			vw_set = Follow_Gimbal_Pid.out * 0.001f;
		}
		else if (chassis_relative_ECD < -30)
		{
			PID_Calc(&Follow_Gimbal_Pid, chassis_relative_ECD, -50, 0);
			vw_set = Follow_Gimbal_Pid.out * 0.001f;
		}else{
			vw_set = 0;
		}
		*angle_set = vw_set;
	}
	else if(Mode==CHASSIS_NO_FOLLOW_YAW)
	{
		RC_to_Control(vx_set, vy_set);
	}
	else if (Mode == CHASSIS_LITTLE_TOP)
	{
		RC_to_Control(vx_set, vy_set);

		*angle_set = Velocity.Speed_Set[2];
	}
}
//����
void Chassis_Ctrl::Control(void)
{
   //�����ٶ�
	fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
	fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
	Behaviour_Control(&vx_set, &vy_set, &angle_set);

   //������̨ģʽ
    if (Mode == CHASSIS_FOLLOW_YAW)
    {
        sin_yaw = arm_sin_f32(-chassis_relative_RAD);
        cos_yaw = arm_cos_f32(-chassis_relative_RAD);
        Velocity.vx_set = cos_yaw * vx_set - sin_yaw * vy_set;
        Velocity.vy_set = sin_yaw * vx_set + cos_yaw * vy_set;
        Velocity.wz_set = angle_set;
    }
	//��������̨ģʽ
    else if (Mode == CHASSIS_NO_FOLLOW_YAW)
    {			
        //��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨����
        sin_yaw = arm_sin_f32(-chassis_relative_RAD);
		cos_yaw = arm_cos_f32(-chassis_relative_RAD);
		
		if (1)//����ǰ������Ϊ��̨������
		{
			Velocity.vx_set = cos_yaw * vx_set - sin_yaw * vy_set;
			Velocity.vy_set = sin_yaw * vx_set + cos_yaw * vy_set;
			Velocity.wz_set = angle_set;
		}
		else//����ǰ������Ϊ����������
		{		
			Velocity.vx_set = vx_set;
			Velocity.vy_set = vy_set;
			Velocity.wz_set = angle_set;
		}
	}
	else if (Mode == CHASSIS_LITTLE_TOP)
	{
        //��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
        sin_yaw = arm_sin_f32(-chassis_relative_RAD);
        cos_yaw = arm_cos_f32(-chassis_relative_RAD);
		Velocity.vx_set = cos_yaw * vx_set - sin_yaw * vy_set;
		Velocity.vy_set = sin_yaw * vx_set + cos_yaw * vy_set;

		Velocity.wz_set=angle_set;
        //�ٶ��޷�
        Velocity.vx_set = fp32_constrain(Velocity.vx_set, Velocity.vx_min_speed, Velocity.vx_max_speed);
        Velocity.vy_set = fp32_constrain(Velocity.vy_set, Velocity.vy_min_speed, Velocity.vy_max_speed);

	}
	//����ģʽ
	else if (Mode == CHASSIS_NO_MOVE)
	{
		Velocity.vx_set = 0;
		Velocity.vy_set = 0;
		Velocity.wz_set = 0;
	}
}


void Chassis_Ctrl::Vector_to_Wheel_Speed(fp32 *vx_set,fp32 *vy_set,fp32 *wz_set)
{

	fp32 vx_temp = *vx_set;
	fp32 vy_temp = *vy_set;
	fp32 wz_temp = *wz_set;
#if useMecanum
	//��ת��ʱ�� ������̨��ǰ��������ǰ������ 0 ��1 ��ת���ٶȱ����� �������� 2,3 ��ת���ٶȱ��
	Motor[0].speed_set = vx_temp - vy_temp + MOTOR_DISTANCE_TO_CENTER * wz_temp;
	Motor[1].speed_set = -vx_temp - vy_temp +  MOTOR_DISTANCE_TO_CENTER * wz_temp;
	Motor[2].speed_set = vx_temp + vy_temp +  MOTOR_DISTANCE_TO_CENTER * wz_temp;
	Motor[3].speed_set = -vx_temp + vy_temp +  MOTOR_DISTANCE_TO_CENTER * wz_temp;
#elif useSteering
	uint8_t i = 0;
	fp32 wheel_speed[4], wheel_angle[4];
	fp32 vx_mid[2],vy_mid[2];
	fp32 vz_mid = sin45 * wz_temp;
		
	vx_mid[0] = vx_temp - vz_mid;
	vx_mid[1] = vx_temp + vz_mid;
	vy_mid[0] = vy_temp - vz_mid;
	vy_mid[1] = vy_temp + vz_mid;
	
	wheel_speed[0] = sqrt(sq(vx_mid[0]) + sq(vy_mid[0]));
	wheel_speed[1] = sqrt(sq(vx_mid[1]) + sq(vy_mid[0]));
	wheel_speed[2] = sqrt(sq(vx_mid[1]) + sq(vy_mid[1]));
	wheel_speed[3] = sqrt(sq(vx_mid[0]) + sq(vy_mid[1]));
	
	wheel_angle[0] = atan2f(vy_mid[1], vx_mid[0]);
	wheel_angle[1] = atan2f(vy_mid[0], vx_mid[0]);
	wheel_angle[2] = atan2f(vy_mid[0], vx_mid[1]);
	wheel_angle[3] = atan2f(vy_mid[1], vx_mid[1]);

	for(i=0;i<4;i++)
	{		
		Motor[i].speed_set = -wheel_speed[i];
		Steering[i].data.angle_set = -wheel_angle[i]* PI_TO_ECD;
	}
#endif
}
//���̿��Ƽ���
void Chassis_Ctrl::Control_loop(void)
{
	uint8_t i = 0;
#if useMecanum
	Vector_to_Wheel_Speed(&Velocity.vx_set, &Velocity.vy_set, &Velocity.wz_set);
    //����pid
    for (i = 0; i < 4; i++)
    {
		PID_Calc(&Speed_Pid[i], Motor[i].speed, Motor[i].speed_set,0);
	}
    //��ֵ����ֵ
    for (i = 0; i < 4; i++)
    {
        Motor[i].give_current = (int16_t)(Speed_Pid[i].out);
    }
#elif useSteering
	//�����˶��ֽ�
	Steering_Behaviour_Control(&Velocity.vx_set, &Velocity.vy_set, &Velocity.wz_set);
	
    //��Ȧ���ĽǶȼ��趨ֵ����
	Steering_Round_Calc();
    //����pid
    for (i = 0; i < 4; i++)
    {
		PID_Calc(&Speed_Pid[i], Motor[i].speed, Motor[i].speed_set,0);
		PID_Calc(&steering_Angle_Pid[i],Steering[i].angle_real,Steering[i].angle_set_real,0);							
		PID_Calc(&steering_Speed_Pid[i],Steering[i].chassis_motor_measure->speed_rpm, steering_Angle_Pid[i].out,0);
	}
    //��ֵ����ֵ
    for (i = 0; i < 4; i++)
    {
        Motor[i].give_current = (int16_t)(Speed_Pid[i].out);
        Steering[i].give_current = (int16_t)(steering_Speed_Pid[i].out);
    }
#endif
}

//����ECD(��Χ��4096)
fp32 motor_ecd_to_relative_ecd(uint16_t angle, uint16_t offset_ecd)
{
    int32_t relative_angle_change = angle - offset_ecd;
    if (relative_angle_change > 4096)
    {
        relative_angle_change -= 8192;
    }
    else if (relative_angle_change < -4096)
    {
        relative_angle_change += 8192;
    }
    return relative_angle_change;
}

const Chassis_Ctrl_Flags_t *get_chassis_flag_control_point(void)
{
    return &Chassis.Flags;
}

const chassis_mode_e *get_chassis_mode_control_point(void)
{
	return &Chassis.Mode;
}

const Chassis_Velocity_t *get_chassis_velocity_control_point(void)
{
	return &Chassis.Velocity;
}

//�Ĵ��������λ
void System_Reset(void)
{
    SCB->AIRCR = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
        (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
        SCB_AIRCR_SYSRESETREQ_Msk);
}

/*------------------------------���¶���ר��------------------------------*/
#if useSteering
//���ݵ���ģʽ���ƶ�����Ϊ
void Chassis_Ctrl::Steering_Mode_Control(void)
{
	if (Mode == CHASSIS_NO_MOVE)
	{
		Steering_Mode = STEERING_STOP;
	}
	else if (Mode == CHASSIS_FOLLOW_YAW)
	{
		Steering_Mode = STEERING_LIMIT;
	}
	else if (Mode == CHASSIS_NO_FOLLOW_YAW)
	{
		Steering_Mode = STEERING_NORMAL;
	}
	else if (Mode == CHASSIS_LITTLE_TOP)
	{
		Steering_Mode = STEERING_LITTLE_TOP;
	}
}
//����ģʽ����
void Chassis_Ctrl::Steering_Behaviour_Control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{
	uint8_t i = 0;
	
	Steering_Mode_Control();
	//��������
	if (ABS(*vx_set) > 0.01f || ABS(*vy_set) > 0.01f || ABS(*wz_set) > 0.01f)
	{
		//���ֽ���
		Vector_to_Wheel_Speed(vx_set, vy_set, wz_set);
	}
	else 
	{
		if (Steering_Mode == STEERING_NORMAL)
		{//���ٶ�����ʱ������ֲ���ת״̬
			Steering_Mode = STEERING_VECTOR_NO_FOLLOW;
		}
		for(i=0;i<4;i++)
		{//����ֹͣ
			Motor[i].speed_set=0;
		}
	}

	switch (Steering_Mode)
	{
	case STEERING_STOP:
		for (i = 0;i < 4;i++){
			Steering[i].data.angle_set = Steering[i].data.angle_set_last;
			Motor[i].speed_set = 0;	
		}
		break;
	case STEERING_FOLLOW_GIMBAL:
		for (i = 0;i < 4;i++){
			Steering[i].data.angle_set = chassis_relative_ECD;
		}
			break;
	case STEERING_FOLLOW_CHASSIS:
		for (i = 0;i < 4;i++){
			Steering[i].data.angle_set = 0;
		}
			break;
	case STEERING_VECTOR_NO_FOLLOW:
		for (i = 0;i < 4;i++){
			Steering[i].data.angle_set = Steering[i].data.angle_set_last;
		}
			break;
	case STEERING_LIMIT:
		for (i = 0;i < 4;i++){
			if (Steering[i].data.angle_set < -2972)//�ǶԳƷ�����Ƕ�
			{
				Steering[i].data.angle_set += 4096;
				Motor[i].speed_set = -Motor[i].speed_set;
			}
			else if (Steering[i].data.angle_set > 1124)
			{
				Steering[i].data.angle_set -= 4096;
				Motor[i].speed_set = -Motor[i].speed_set;
			}
		}
		break;
	case STEERING_LITTLE_TOP:
		if ((vx_set != 0 || vy_set != 0) && wz_set != 0)
		{
			//С����״̬�����ٶ�
			*vx_set *= 0.7f;
			*vy_set *= 0.7f;
			*wz_set *= 0.3f;
		}
		break;
	default:
		break;
	}
}
//��Ȧ���������Ƕȼ���
void Chassis_Ctrl::Steering_Round_Calc(void)
{
    int16_t relative_angle_round=0;	
	for(int i=0;i<4;i++)
	{
		//ʵ�ʽǶ���������
		relative_angle_round = Steering[i].data.angle - Steering[i].data.angle_last;
		if(relative_angle_round>4096) 
		{
			Steering[i].data.angle_round--;
		}
		else if(relative_angle_round<-4096) 
		{
			Steering[i].data.angle_round++;
		}
		Steering[i].angle_real = 
			Steering[i].data.angle_round*8191 + Steering[i].data.angle;
		//�趨�Ƕ���������
		relative_angle_round=Steering[i].data.angle_set-Steering[i].data.angle_set_last;
		if(relative_angle_round>4096) 
		{
			Steering[i].data.angle_set_round--;
		}
		else if(relative_angle_round<-4096) 
		{
			Steering[i].data.angle_set_round++;
		}
		Steering[i].angle_set_real = 
			Steering[i].data.angle_set_round*8191 + Steering[i].data.angle_set;
		//�Ż���ת�ķ�Χ
		if(Steering[i].angle_set_real-Steering[i].angle_real>4096) Steering[i].data.angle_set_round--;
		else if(Steering[i].angle_set_real-Steering[i].angle_real<-4096) Steering[i].data.angle_set_round++;
	}
}
#endif
