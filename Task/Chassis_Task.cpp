#include "Chassis_Task.h"
#include "message_Task.h"
#include "Guard_Task.h"
#include "arm_math.h"

#include "app_motor.h"
#include "dev_can.h"
#include "protocol_dbus.h"

Chassis_Ctrl Chassis;
const Gimbal_Data_t *Gimbal;

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
		Chassis.chassis_behaviour_mode_set();
		if (Chassis.Mode != CHASSIS_NO_MOVE)
		{
			Chassis.Feedback_Update();
			Chassis.Control();
			Chassis.chassis_control_loop();
			
			CAN_Cmd.Chassis.CAN_Chassis->SendData(Chassis.Motor[0].give_current, Chassis.Motor[1].give_current,
				Chassis.Motor[2].give_current, Chassis.Motor[3].give_current);
#if useSteering
			CAN_Cmd.Gimbal.CAN_Gimbal->SendData(Chassis.Steering[0].give_current, Chassis.Steering[1].give_current,
				Chassis.Steering[2].give_current, Chassis.Steering[3].give_current);
#endif
		}
		else
		{
			CAN_Cmd.Chassis.CAN_Chassis->SendData(0, 0, 0, 0);
#if useSteering
			CAN_Cmd.Gimbal.CAN_Gimbal->SendData(0, 0, 0, 0);
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
	Gimbal = get_gimbal_data_point();
	
	for ( uint8_t i = 0; i < 4; i++ )
	{
		Motor[i].chassis_motor_measure = Motor_Ctrl.Chassis.Get_Motor_Measure_Pointer(i);
		PID_Init( &Chassis.Speed_Pid[i], PID_POSITION, Motor_Speed_Pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT );
	}
#if useSteering/* useSteering */
	for ( uint8_t i = 0; i < 4; i++ )
	{
		Steering[i].chassis_motor_measure = Motor_Ctrl.Gimbal.Get_Motor_Measure_Pointer(i);
		PID_Init( &Chassis.steering_Angle_Pid[i], PID_POSITION, m6020_motor_angle_pid, M6020_MOTOR_ANGLE_PID_MAX_OUT, M6020_MOTOR_ANGLE_PID_MAX_IOUT );
		PID_Init( &Chassis.steering_Speed_Pid[i], PID_POSITION, m6020_motor_speed_pid, M6020_MOTOR_SPEED_PID_MAX_OUT, M6020_MOTOR_SPEED_PID_MAX_IOUT );
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
	
	Chassis.Feedback_Update();
}
//���ݸ���
void Chassis_Ctrl::Feedback_Update( void )
{
	uint8_t i = 0;
	for (i = 0; i < 4; i++)
    {
        //���µ���ٶȣ����ٶ����ٶȵ�PID΢��
        Chassis.Motor[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * Chassis.Motor[i].chassis_motor_measure->speed_rpm;
        Chassis.Motor[i].accel = Chassis.Speed_Pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }
#if useMecanum
    //���µ���ǰ���ٶ� x��ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ
    Chassis.Velocity.vx = (-Chassis.Motor[0].speed + Chassis.Motor[1].speed + Chassis.Motor[2].speed - Chassis.Motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    Chassis.Velocity.vy = (-Chassis.Motor[0].speed - Chassis.Motor[1].speed + Chassis.Motor[2].speed + Chassis.Motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    Chassis.Velocity.wz = (-Chassis.Motor[0].speed - Chassis.Motor[1].speed - Chassis.Motor[2].speed - Chassis.Motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
#elif	useSteering
	for (i = 0; i < 4; i++)
    {
        //���µ���ٶȣ����ٶ����ٶȵ�PID΢��
		Chassis.Steering[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * Chassis.Motor[i].chassis_motor_measure->speed_rpm;
		Chassis.Steering[i].accel = Chassis.Speed_Pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
		Chassis.Steering->angle = motor_angle_to_set_change(Chassis.Steering->chassis_motor_measure->ecd, MOTOR_6020_offset[i]);
	}
#endif
}
//������Ϊ״̬����
void Chassis_Ctrl::chassis_behaviour_mode_set(void)
{
	if (switch_is_up(Chassis.RC_Ptr->rc.s[CHANNEL_RIGHT]))
		Chassis.Flags.RC_Flag = false;
	else Chassis.Flags.RC_Flag = true;
	//��������
	if (Chassis.Flags.RC_Flag == true)
	{
		if (read_key_count(&Key.C)==1)
		{
			// key_C.count=0;//��־λ����ȴ���һ�ΰ���
			Chassis.Mode = CHASSIS_NO_MOVE;//���̱��ֲ���
		}
		if (read_key_count(&Key.X)==1)
		{
			if (Chassis.Mode == CHASSIS_NO_MOVE)
			{
				Chassis.Mode = CHASSIS_NO_FOLLOW_YAW;//���̲�������̨			
			}
			else if (Chassis.Mode == CHASSIS_LITTLE_TOP)
			{
				Chassis.Mode=CHASSIS_NO_FOLLOW_YAW;//���̲�������̨
			}
			else if (Chassis.Mode == CHASSIS_NO_FOLLOW_YAW)
			{
				Chassis.Mode=CHASSIS_FOLLOW_YAW;//���̸�����̨
			}			
			else if (Chassis.Mode == CHASSIS_FOLLOW_YAW)
			{
				Chassis.Mode=CHASSIS_NO_FOLLOW_YAW;//���̲�������̨
			}
		}
		if (read_key_count(&Key.G) == 1)
		{//����С����
			if (Chassis.Mode != CHASSIS_LITTLE_TOP)
				Chassis.Mode = CHASSIS_LITTLE_TOP;
			else Chassis.Mode = CHASSIS_NO_FOLLOW_YAW;
		}
		//�ٶ�ѡ��
		if (read_key_count(&Key.shift)==1)
		{//����  SHIFT
			if (Chassis.Velocity.Speed_Gear < 3)
				Chassis.Velocity.Speed_Gear++;
		}
		else if (read_key_count(&Key.ctrl)==1)
		{//����  CTRL
			if (Chassis.Velocity.Speed_Gear > 0)
				Chassis.Velocity.Speed_Gear--;
		}
		//�Ӿ�����
		read_key_single(&Press.R, &Chassis.Flags.Vision_Flag);
		//UI���
		if (read_key_count(&Key.B) == 1)
		{
			UIsend = 10;
			//�������ؿ���
			if ((Chassis.Flags.Vision_Flag == 1)&&(Chassis.Flags.Energy_Flag == 0))
				Chassis.Flags.Energy_Flag = 1;
			else if ((Chassis.Flags.Vision_Flag == 1)&&(Chassis.Flags.Energy_Flag == 1))
				Chassis.Flags.Energy_Flag = 1;
		}
		//Ԥ�⿪��
		if (Chassis.Flags.Vision_Flag == 1)
			read_key_single(&Key.E, &Chassis.Flags.Predict_Flag);
		//�л�ǹ��
		read_key_single(&Key.Q, &Chassis.Flags.Shoot_Direction_Flag);
		//װ������
		read_key_single(&Key.V, &Chassis.Flags.Looding_Flag);
		//Ħ���ֿ���
		read_key_single(&Key.R, &Chassis.Flags.Fric_Flag);
		//�����ֿ���
		if (Chassis.Flags.Fric_Flag)
			read_key_even(&Press.L, &Chassis.Flags.Shoot_Flag);
		else Chassis.Flags.Shoot_Flag = 0;
		//���ٿ���
		read_key_single(&Key.F, &Chassis.Flags.Speed_Up_Flag);
	}
	//ң�ؿ���ģʽ�л�
	if (switch_is_down(Chassis.RC_Ptr->rc.s[CHANNEL_RIGHT]))
	{
		Chassis.Mode = CHASSIS_NO_MOVE;
	}
#if MOVE_OR_SHOOT
	if (switch_is_mid(Chassis.RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_down(Chassis.RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Chassis.Mode = CHASSIS_NO_FOLLOW_YAW;
	}
	else if(switch_is_mid(Chassis.RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_mid(Chassis.RC_Ptr->rc.s[CHANNEL_LEFT])){
		Chassis.Mode = CHASSIS_FOLLOW_YAW;
	}
	else if(switch_is_mid(Chassis.RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_up(Chassis.RC_Ptr->rc.s[CHANNEL_LEFT])){
		Chassis.Mode = CHASSIS_LITTLE_TOP;
	}
#else
	else Chassis.Mode = CHASSIS_NO_FOLLOW_YAW;
	if (switch_is_mid(Chassis.RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_down(Chassis.RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Chassis.Flags.Shoot_Flag=0;
		Chassis.Flags.Fric_Flag=0;
	}
	else if (switch_is_mid(Chassis.RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_mid(Chassis.RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Chassis.Flags.Shoot_Flag = 1;
	}
	else if (switch_is_mid(Chassis.RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_up(Chassis.RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Chassis.Flags.Fric_Flag = 1;
	}
#endif
	else if (switch_is_up(Chassis.RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_up(Chassis.RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		System_Reset();
	}
	else if (switch_is_up(Chassis.RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_mid(Chassis.RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		UIsend = 10;
	}
}

//ң���������ݴ���ɵ��̵�ǰ��vx�ٶȣ�vy�ٶ�
void Chassis_Ctrl::chassis_rc_to_control_vector( fp32 *vx_set, fp32 *vy_set)
{
	if ( vx_set == NULL || vy_set == NULL )
	{
			return;
	}
	//ң����ԭʼͨ��ֵ
	int16_t vx_channel, vy_channel;
  	fp32 vx_set_channel, vy_set_channel;
	
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
		if ( Chassis.RC_Ptr->key.v & CHASSIS_FRONT_KEY||Chassis.RC_Ptr->key.v & CHASSIS_BACK_KEY||
			Chassis.RC_Ptr->key.v & CHASSIS_LEFT_KEY ||Chassis.RC_Ptr->key.v & CHASSIS_RIGHT_KEY )
		{
			if (Chassis.RC_Ptr->key.v & CHASSIS_FRONT_KEY)//������ܸĶ�
			{
				vx_set_channel = Velocity.Speed_Set[Chassis.Flags.Speed_Up_Flag];
			}
			else if (Chassis.RC_Ptr->key.v & CHASSIS_BACK_KEY)
			{
				vx_set_channel = -Velocity.Speed_Set[Chassis.Flags.Speed_Up_Flag];
			}
			else if (Chassis.RC_Ptr->key.v & CHASSIS_LEFT_KEY)
			{
				vy_set_channel = Velocity.Speed_Set[Chassis.Flags.Speed_Up_Flag];
			}
			else if (Chassis.RC_Ptr->key.v & CHASSIS_RIGHT_KEY)
			{
				vy_set_channel = -Velocity.Speed_Set[Chassis.Flags.Speed_Up_Flag];
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
		rc_deadline_limit( Chassis.RC_Ptr->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE  );
		rc_deadline_limit( Chassis.RC_Ptr->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE  );
 
		vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
		vy_set_channel = -( vy_channel * CHASSIS_VY_RC_SEN );
	}
		 
	//һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
	first_order_filter_cali( &Chassis.Filter_vx, vx_set_channel );
	first_order_filter_cali( &Chassis.Filter_vy, vy_set_channel );			
		
	//ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
	if ( vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN )
    {
        Chassis.Filter_vx.out = 0.0f;
    }

    if ( vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN )
    {
        Chassis.Filter_vy.out = 0.0f;
    }
		
    *vx_set = Chassis.Filter_vx.out;
    *vy_set = Chassis.Filter_vy.out;
}

void Chassis_Ctrl::chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set)
{

	fp32 vw_set;

	if (Chassis.Mode == CHASSIS_NO_MOVE)//����״̬
	{
		*vx_set = 0.0f;
		*vy_set = 0.0f;
		*angle_set = 0.0f;
	}
	else if (Chassis.Mode == CHASSIS_FOLLOW_YAW)//������̨
	{
		chassis_rc_to_control_vector(vx_set, vy_set);//��ң��ֵת��Ϊ�����趨��

		Chassis.chassis_relative_angle = *(Chassis.chassis_yaw_relative_angle);//�����̨�Ƕ�
		if (Chassis.chassis_relative_angle > 10)// ���800
		{
			PID_Calc(&Chassis.Follow_Gimbal_Pid, Chassis.chassis_relative_angle, 50, 0);
			vw_set = Chassis.Follow_Gimbal_Pid.out * 0.001f;
		}
		else if (Chassis.chassis_relative_angle < -10)
		{
			PID_Calc(&Chassis.Follow_Gimbal_Pid, Chassis.chassis_relative_angle, -50, 0);
			vw_set = Chassis.Follow_Gimbal_Pid.out * 0.001f;
		}
		else
		{
			vw_set = 0;
		}
		*angle_set = vw_set;
	}
	else if(Chassis.Mode==CHASSIS_NO_FOLLOW_YAW)
	{
		chassis_rc_to_control_vector(vx_set, vy_set);
	}
	else if (Chassis.Mode == CHASSIS_LITTLE_TOP)
	{
		chassis_rc_to_control_vector(vx_set, vy_set);

		*angle_set = Chassis.Velocity.Speed_Set[2];
	}
}
//����
void Chassis_Ctrl::Control(void)
{
   //�����ٶ�
	fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f, relative_angle = 0.0f;
	fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
	chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set);

   //������̨ģʽ
    if (Chassis.Mode == CHASSIS_FOLLOW_YAW)
    {
		Chassis.chassis_relative_angle=*(Chassis.chassis_yaw_relative_angle);//�����̨�Ƕ�
		relative_angle=Chassis.chassis_relative_angle*ECD_TO_PI;
        sin_yaw = arm_sin_f32(-relative_angle);
        cos_yaw = arm_cos_f32(-relative_angle);
        Chassis.Velocity.vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        Chassis.Velocity.vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        Chassis.Velocity.wz_set = angle_set;
    }
	//��������̨ģʽ
    else if (Chassis.Mode == CHASSIS_NO_FOLLOW_YAW)
    {
		//��������̨ģʽ
		Chassis.chassis_relative_angle=*(Chassis.chassis_yaw_relative_angle);//�����̨�Ƕ�
		relative_angle=Chassis.chassis_relative_angle*ECD_TO_PI;
			
        //��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨����
        sin_yaw = arm_sin_f32(-relative_angle);
		cos_yaw = arm_cos_f32(-relative_angle);
		
		if (1)//����ǰ������Ϊ��̨������
		{
			Chassis.Velocity.vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
			Chassis.Velocity.vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
			Chassis.Velocity.wz_set = angle_set;
		}
		else//����ǰ������Ϊ����������
		{		
			Chassis.Velocity.vx_set = vx_set;
			Chassis.Velocity.vy_set = vy_set;
			Chassis.Velocity.wz_set = angle_set;
		}
	}
	else if (Chassis.Mode == CHASSIS_LITTLE_TOP)
	{
		Chassis.chassis_relative_angle=*(Chassis.chassis_yaw_relative_angle);//�����̨�Ƕ�
        //��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
        sin_yaw = arm_sin_f32(-Chassis.chassis_relative_angle);
        cos_yaw = arm_cos_f32(-Chassis.chassis_relative_angle);
        Chassis.Velocity.vx_set = -sin_yaw * vx_set + cos_yaw * vy_set;
		Chassis.Velocity.vy_set = cos_yaw * vx_set + sin_yaw * vy_set;

		Chassis.Velocity.wz_set=angle_set;
        //�ٶ��޷�
        Chassis.Velocity.vx_set = fp32_constrain(Chassis.Velocity.vx_set, Chassis.Velocity.vx_min_speed, Chassis.Velocity.vx_max_speed);
        Chassis.Velocity.vy_set = fp32_constrain(Chassis.Velocity.vy_set, Chassis.Velocity.vy_min_speed, Chassis.Velocity.vy_max_speed);

	}
	//����ģʽ
	else if (Chassis.Mode == CHASSIS_NO_MOVE)
	{
		Chassis.Velocity.vx_set = 0;
		Chassis.Velocity.vy_set = 0;
		Chassis.Velocity.wz_set = 0;
	}
}

#if useMecanum
void Chassis_Ctrl::chassis_vector_to_wheel_speed(fp32 *vx_set,fp32 *vy_set,fp32 *wz_set)
{
	fp32 vx_temp = *vx_set;
	fp32 vy_temp = *vy_set;
	fp32 wz_temp = *wz_set;
	//��ת��ʱ�� ������̨��ǰ��������ǰ������ 0 ��1 ��ת���ٶȱ����� �������� 2,3 ��ת���ٶȱ��
	Chassis.Motor[0].speed_set =  vx_temp - vy_temp +  MOTOR_DISTANCE_TO_CENTER * wz_temp;
	Chassis.Motor[1].speed_set = -vx_temp - vy_temp +  MOTOR_DISTANCE_TO_CENTER * wz_temp;
	Chassis.Motor[2].speed_set =  vx_temp + vy_temp +  MOTOR_DISTANCE_TO_CENTER * wz_temp;
	Chassis.Motor[3].speed_set = -vx_temp + vy_temp +  MOTOR_DISTANCE_TO_CENTER * wz_temp;
}
#elif useSteering
void Chassis_Ctrl::chassis_vector_to_wheel_speed(fp32 *vx_set,fp32 *vy_set,fp32 *wz_set)
{
	uint8_t i = 0;
	fp32 vx_temp = *vx_set;
	fp32 vy_temp = *vy_set;
	fp32 wz_temp = *wz_set;

	fp32 vx_mid[2],vy_mid[2];
	fp32 wheel_speed[4], wheel_angle[4];
	fp32 vz_mid = sin45 * wz_temp;
	
	if (vx_temp != 0 && vy_temp != 0)
	{
		vx_temp *=  0.7f;
		vy_temp *=  0.7f;
	}
	
	vx_mid[0] = vx_temp - vz_mid;
	vx_mid[1] = vx_temp + vz_mid;
	vy_mid[0] = vy_temp - vz_mid;
	vy_mid[1] = vy_temp + vz_mid;

	wheel_speed[0] = sq(vx_mid[0] * vx_mid[0] + vy_mid[0] * vy_mid[0]);
	wheel_speed[1] = sq(vx_mid[1] * vx_mid[1] + vy_mid[0] * vy_mid[0]);
	wheel_speed[2] = sq(vx_mid[1] * vx_mid[1] + vy_mid[1] * vy_mid[1]);
	wheel_speed[3] = sq(vx_mid[0] * vx_mid[0] + vy_mid[1] * vy_mid[1]);
	
	wheel_angle[0] = atan2(vy_mid[0], vx_mid[0]);
	wheel_angle[1] = atan2(vy_mid[1], vx_mid[0]);
	wheel_angle[2] = atan2(vy_mid[1], vx_mid[1]);
	wheel_angle[3] = atan2(vy_mid[0], vx_mid[1]);

	for(i=0;i<4;i++)
	{
		wheel_angle[i] = wheel_angle[i] * PI_TO_ECD;
		wheel_speed[i] = -wheel_speed[i];
		
		Chassis.Motor[i].speed_set = wheel_speed[i];
		Chassis.Steering[i].angle_set = wheel_angle[i];
	}
}
//���ݵ���ģʽ���ƶ�����Ϊ
void Chassis_Ctrl::steering_mode_control_set(void)
{
	if (Chassis.Mode == CHASSIS_NO_MOVE)
	{
		Chassis.Steering_Mode = STEERING_STOP;
	}
	else if (Chassis.Mode == CHASSIS_FOLLOW_YAW)
	{
		Chassis.Steering_Mode = STEERING_LIMIT;
	}
	else if (Chassis.Mode == CHASSIS_NO_FOLLOW_YAW)
	{
		Chassis.Steering_Mode = STEERING_NORMAL;
	}
	else if (Chassis.Mode == CHASSIS_LITTLE_TOP)
	{
		Chassis.Steering_Mode = STEERING_LITTLE_TOP;
	}
}
//����ģʽ����
void Chassis_Ctrl::steering_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{
	uint8_t i = 0;
	
	steering_mode_control_set();

	if ((*vx_set != NULL || *vy_set != NULL || *wz_set != NULL))
	{
		//���ֽ���
		chassis_vector_to_wheel_speed(vx_set, vy_set, wz_set);
	}
	else if (Chassis.Steering_Mode == STEERING_NORMAL)
	{
		Chassis.Steering_Mode = STEERING_VECTOR_NO_FOLLOW;
	}

	switch (Chassis.Steering_Mode)
	{
		case STEERING_STOP:
			for (i = 0;i < 4;i++){
				Chassis.Steering[i].angle_set = Chassis.Steering[i].angle;
				Chassis.Steering[i].speed_set = 0;	
			}
			break;
		case STEERING_FOLLOW_GIMBAL:
			for (i = 0;i < 4;i++){
				Chassis.Steering[i].angle_set = Chassis.chassis_relative_angle;
			}
				break;
		case STEERING_FOLLOW_CHASSIS:
			for (i = 0;i < 4;i++){
				Chassis.Steering[i].angle_set = 0;
			}
				break;
		case STEERING_VECTOR_NO_FOLLOW:
			for (i = 0;i < 4;i++){
				Chassis.Steering[i].angle_set = Chassis.Steering[i].angle_set_last;
			}
				break;
		case STEERING_LIMIT:
			for (i = 0;i < 4;i++){
				if (Chassis.Steering[i].angle_set < -2972)//�ǶԳƷ�����Ƕ�
				{
					Chassis.Steering[i].angle_set = 4096 + Chassis.Steering[i].angle_set;
					Chassis.Steering[i].speed = -Chassis.Steering[i].speed;
				}
				else if (Chassis.Steering[i].angle_set > 1124)
				{
					Chassis.Steering[i].angle_set = -4096 + Chassis.Steering[i].angle_set;
					Chassis.Steering[i].speed = -Chassis.Steering[i].speed;
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
#endif

//���̿��Ƽ���
void Chassis_Ctrl::chassis_control_loop(void)
{
	uint8_t i = 0;
#if useMecanum
	chassis_vector_to_wheel_speed(&Chassis.Velocity.vx_set, &Chassis.Velocity.vy_set, &Chassis.Velocity.wz_set);
    //����pid
    for (i = 0; i < 4; i++)
    {
		PID_Calc(&Chassis.Speed_Pid[i], Chassis.Motor[i].speed, Chassis.Motor[i].speed_set,0);
	}
    //��ֵ����ֵ
    for (i = 0; i < 4; i++)
    {
        Chassis.Motor[i].give_current = (int16_t)(Chassis.Speed_Pid[i].out);
    }
#elif useSteering
	//�����˶��ֽ�
	steering_behaviour_control_set(&Chassis.Velocity.vx_set, &Chassis.Velocity.vy_set, &Chassis.Velocity.wz_set);
	
    //��Ȧ���ĽǶȼ��趨ֵ����
	chassis_round_calc();
    //����pid
    for (i = 0; i < 4; i++)
    {
		PID_Calc(&Chassis.Speed_Pid[i], Chassis.Motor[i].speed, Chassis.Motor[i].speed_set,0);
		PID_Calc(&Chassis.steering_Angle_Pid[i],Chassis.Steering[i].angle_real,Chassis.Steering[i].angle_set_real,0);							
		PID_Calc(&Chassis.steering_Speed_Pid[i],Chassis.Steering[i].chassis_motor_measure->speed_rpm, Chassis.steering_Angle_Pid[i].out,0);
	}
    //��ֵ����ֵ
    for (i = 0; i < 4; i++)
    {
        Chassis.Motor[i].give_current = (int16_t)(Chassis.Speed_Pid[i].out);
        Chassis.Steering[i].give_current = (int16_t)(Chassis.steering_Speed_Pid[i].out);
    }
#endif
}
#if useSteering
//��Ȧ���������Ƕȼ���
void Chassis_Ctrl::chassis_round_calc(void)
{
    int16_t relative_angle_round=0;	
	for(int i=0;i<4;i++)
	{
		//ʵ�ʽǶ���������
		relative_angle_round = Chassis.Steering[i].angle - Chassis.Steering[i].angle_last;
		if(relative_angle_round>4096) 
		{
			Chassis.Steering[i].angle_round--;
		}
		else if(relative_angle_round<-4096) 
		{
			Chassis.Steering[i].angle_round++;
		}
		Chassis.Steering[i].angle_real = 
			Chassis.Steering[i].angle_round*8191 + Chassis.Steering[i].angle;
		//�趨�Ƕ���������
		relative_angle_round=Chassis.Steering[i].angle_set-Chassis.Steering[i].angle_set_last;
		if(relative_angle_round>4096) 
		{
			Chassis.Steering[i].angle_set_round--;
		}
		else if(relative_angle_round<-4096) 
		{
			Chassis.Steering[i].angle_set_round++;
		}
		Chassis.Steering[i].angle_set_real = 
			Chassis.Steering[i].angle_set_round*8191 + Chassis.Steering[i+4].angle_set;
		//�Ż���ת�ķ�Χ
		Chassis.Steering[i].angle_round +=
			(uint16_t)((Chassis.Steering[i].angle_set_real - Chassis.Steering[i].angle_real) / 8192);
	}
}
#endif
//����ECD
fp32 Chassis_Ctrl::motor_angle_to_set_change(uint16_t angle, uint16_t offset_ecd)
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

void Chassis_Ctrl::error_behaviour_control_set(void)
{
	const float *error = 0;
	// if (Guard_ID == usart6)
	{
		chassis_yaw_relative_angle = error;
	}
}

const Chassis_Ctrl_Flags_t *get_chassis_control_point(void)
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