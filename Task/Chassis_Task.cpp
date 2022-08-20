#include "Chassis_Task.h"

Chassis_Ctrl Chassis;
State_Switch Chassis_State_Switch(5);

//�����ٶȻ�pidֵ
const static fp32 Motor_Speed_Pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
//������ת��pidֵ
const static fp32 Chassis_Follow_Gimbal_Pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
const static fp32 chassis_z_order_filter[1] = {CHASSIS_ACCEL_Z_NUM};

static void Chassis_Init( void );
static void Chassis_Feedback_Update( void );

void Chassis_Task( void *pvParameters )
{
	CAN_ALL_Init();//���е�����ƶ���һ������
	Chassis_Init();
	
	while(1)
	{
	  
	}
}

void Chassis_Ctrl::Chassis_Init( void )
{
  RC_Ptr = get_remote_control_point();

#ifdef useMecanum
	
	for ( uint8_t i = 0; i < 4; i++ )
	{
			Motor[i].chassis_motor_measure = Motor_Ctrl.Chassis.Get_Motor_Measure_Pointer(i);
			PID_Init( &Chassis.Speed_Pid[i], PID_POSITION, Motor_Speed_Pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT );
	}
	
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

#endif  /* useMecanum */
	
#ifdef  useSteering
	
#endif  /* useSteering */
	
	Chassis_Feedback_Update();
}

static void Chassis_Feedback_Update( void )
{
    uint8_t i = 0;
    for ( i = 0; i < 4; i++ )
    {
        //���µ���ٶȣ����ٶ����ٶȵ�PID΢��
        Chassis.Motor[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * Chassis.Motor[i].chassis_motor_measure->speed_rpm;
        Chassis.Motor[i].accel = Chassis.Speed_Pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }

    //���µ���ǰ���ٶ� x�� ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ
    Chassis.Velocity.vx = (-Chassis.Motor[0].speed + Chassis.Motor[1].speed + Chassis.Motor[2].speed - Chassis.Motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    Chassis.Velocity.vy = (-Chassis.Motor[0].speed - Chassis.Motor[1].speed + Chassis.Motor[2].speed + Chassis.Motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    Chassis.Velocity.wz = (-Chassis.Motor[0].speed - Chassis.Motor[1].speed - Chassis.Motor[2].speed - Chassis.Motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
}

//ң���������ݴ���ɵ��̵�ǰ��vx�ٶȣ�vy�ٶ�
void Chassis_Ctrl::chassis_rc_to_control_vector( fp32 *vx_set, fp32 *vy_set )
{
	if ( vx_set == NULL || vy_set == NULL )
	{
			return;
	}
	//ң����ԭʼͨ��ֵ
	fp32 vx_set_channel, vy_set_channel;		
	
	
  if( Velocity.Speed_Gear==4 )
//		{   speed_set[0]=2.55;	  speed_set[1//  if(spin_flag)
//	{
//		if(speed_gear==1)		
//		{   speed_set[0]=1.35;	  speed_set[1]=0.75;   }//1.35��0.75
//		else if(speed_gear==2)
//		{   speed_set[0]=1.35;	  speed_set[1]=0.75;   }	
//		else if(speed_gear==3)
//		{   speed_set[0]=1.35;	  speed_set[1]=0.75;   }	
//		else if(speed_gear==4)
//		{   speed_set[0]=1.35;	  speed_set[1]=0.75;   }
//	}
//	else
//	{
//		if(speed_gear==1)
//		{   speed_set[0]=1.1;	    speed_set[1]=0.8;   }//�ҵ�
//		else if(speed_gear==2)
//		{   speed_set[0]=1.95;	  speed_set[1]=1.7;   }	//1
//		else if(speed_gear==3)
//		{   speed_set[0]=2.35;	  speed_set[1]=1.9;   }	//2
//		else if]=2.3;   }	//3
//  }

		
	if( Flags.RC_Flag == false )
	{
		//��WDAS����
		if ( Chassis.RC_Ptr->key.v & CHASSIS_FRONT_KEY||Chassis.RC_Ptr->key.v & CHASSIS_BACK_KEY||
			   Chassis.RC_Ptr->key.v & CHASSIS_LEFT_KEY ||Chassis.RC_Ptr->key.v & CHASSIS_RIGHT_KEY )
		{
			if (Chassis.RC_Ptr->key.v & CHASSIS_FRONT_KEY)
			{
				 vx_set_channel = speed_set[0];
			}
			else if (Chassis.RC_Ptr->key.v & CHASSIS_BACK_KEY)
			{
				 vx_set_channel = -speed_set[0];

			}
			else if (Chassis.RC_Ptr->key.v & CHASSIS_LEFT_KEY)
			{
				 vy_set_channel = speed_set[0];
				
			}
			else if (Chassis.RC_Ptr->key.v & CHASSIS_RIGHT_KEY)
			{
					vy_set_channel = -speed_set[0];
			}			
		}
		else//��WSAD������һֱ��ֹ
		{
			vx_set_channel=0;
			vy_set_channel=0;			
      CAN_Cmd.Chassis.CAN_Chassis->SendData( 0, 0, 0, 0 );
		}
	}
	else
	{
		//��ң���������ݴ������� int16_t yaw_channel,pitch_channel
		rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[3], front_channel, RC_deadband);
		rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[2], left_channel, RC_deadband);
 
		vx_set_channel=front_channel*0.003030303f;
		vy_set_channel=-(left_channel*0.003030303f);
	}
		 
		//һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
		first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
		first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);			
		
	
	//ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
	if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }
		
    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}