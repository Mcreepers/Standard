#include "algorithm_pid.h"

/*
******************************************************************************
  * @attention
  * 
  * ���ô˿�ʱ��ע��:
  * # ���в���ϵͳʱ��ʹ�þ�����ʱ����vTaskDelayUntil
  * # ��ʹ�ò���ϵͳ���ṩһ����׼��ʱ��������Ϊ1ms����1us
  * # ���Ķ�ʱ�����ں������pid.h�� TIMERPERCYCLE
	* # �л�����ģʽ����sys.h�ж��� useFreeRTOS Ϊ Disable��Enable
  * # ʹ��PIDʱ����ִ��pidFunPointerInit
	*
  * <h2><center>&copy; Copyright (c) 2022 - ~, USC	MA Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */

#ifndef useSimplePID

float PID_Position_Calc( PidTypeDef *pid, float set, float ref);
float PID_Delta_Calc( PidTypeDef *pid, float set, float ref );
static float variableIntegralCoefficientCalc( float* maxInterval, float* minInterval, float* thiserror );

static void Membership_Calc( PidTypeDef *pid );
static void Fuzzification( PidTypeDef *pid, float set, float ref );
static void Defuzzification( PidTypeDef *pid );

static PidMode getPIDWorkMode( PidTypeDef *pid );
static PidChangerTypeDef getPIDChangerState( PidTypeDef *pid );

/**
  * @brief          pid����ָ���ʼ������
  * @param[in]      *pid: �ṹ��ָ��
  * @retval         none
  */
void pidFunPointerInit( PidTypeDef *pid )
{
	if ( pid == NULL )
   {
       return;
   }
	pid->vParmaInitFun = PID_Init;
	pid->vChangerInitFun = PIDChangerInit;
	pid->vClearFun = PID_Clear;
	pid->fCalcFun = PID_Calc;
	 
	pid->pmGetFun = getPIDWorkMode;
	pid->pcGetFun = getPIDChangerState;
}

/**
  * @brief          PID��ʼ��
	* @param[in]      pid: PID�ṹ����ָ��
  * @param[in]      PID_Coefficient[3]: PID������������ΪKp��Ki��Kd��������DR_PIDʱ�����һ��������ʹ��
	* @param[in]      max_Iout: ���������
	* @param[in]      max_out: PID������
	* @param[in]      band_I: ���ַ�����ֵ
	* @param[in]      dead_band: pid����
	* @param[in]      filter_D: ΢��һ�׹���ϵͳϵ������ֵԽСЧ��Խ�ã���ϵͳ�����Ȼ��½���
																	 ��֮��Ч������������ߡ�0Ϊ��ʹ�ù���ϵͳ��
	* @param[in]      pvCoefficient: ΢�������˲�ϵ��
	* @param[in]      minInterval: ���ٻ�����������
	* @param[in]      maxInterval: ���ٻ�����������
	* @param[in]      Cycle: pid�������� ��λ:s
  * @retval         none
  */
void PID_Init( PidTypeDef *pid, const float PID_Coefficient[3], float max_Iout, 
	             float max_out, float band_I, float dead_band, float pvCoefficient,
							 float filter_D, float	minInterval, float maxInterval, float Cycle )
{
	if ( pid == NULL || PID_Coefficient == NULL )
    {
        return;
    }
		
		pid->max_Iout = max_Iout;
		pid->max_out = max_out;
		pid->band_I = band_I;
		pid->dead_band = dead_band;
		pid->filter_D = pid->filter_D;
		pid->pvCoefficient = pvCoefficient;
		pid->minInterval = minInterval;
		pid->maxInterval = maxInterval;
		
		#ifndef useFreeRTOS
		pid->pidCycle.sampleTime = Cycle * TIMERPERCYCLE;
		#endif
		
		if( pid->PidChanger.disturbanceRejection == false )
		{
			pid->Kp = PID_Coefficient[0];
			pid->Ki = PID_Coefficient[1] * Cycle;
			pid->Kd = PID_Coefficient[2] / Cycle;
		}
		else if( pid->PidChanger.disturbanceRejection == true )
		{
			pid->Kp = PID_Coefficient[0];
			pid->wc = PID_Coefficient[1];
			
			pid->Kp = 2 * pid->Kp * pid->wc;
			pid->Ki = pid->wc * pid->wc * pid->Kp * Cycle;
			pid->Kd = pid->Kp / Cycle;
		}

}

/**
  * @brief          PID���ܿ��غ���
  * @param[in]      pid: PID�ṹ����ָ��
  * @param[in]      tlBool: ���λ��ֿ���
  * @param[in]      pvBool: ΢�����п���
  * @param[in]      drBool: DR_PID����
  * @retval         none
  */
void PIDChangerInit(PidTypeDef *pid, bool tlBool, bool pvBool, bool fcBool, bool drBool )
{
	if ( pid == NULL )
  {
       return;
  }
	pid->PidChanger.trapezoidalintegral = tlBool;
	pid->PidChanger.processVariable = pvBool;
	pid->PidChanger.fuzzyController = fcBool;
	pid->PidChanger.disturbanceRejection = drBool;
}

/**
  * @brief          PID���㺯��
  * @param[in]      pid: PID�ṹ����ָ��
  * @param[in]      set: �趨ֵ
  * @param[in]      ref: ����ֵ
  * @param[in]      mode: PIDģʽ INITΪ��ʼģʽ��POSITIONΪλ��ʽ��DELTAΪ����ʽ
  * @retval         PID���ֵ
  */
float PID_Calc( PidTypeDef *pid, float set, float ref, PidMode mode )
{
		#ifdef useFreeRTOS
    if ( pid == NULL )
    {
      return 0.0f;
    }
			
			if( mode == POSITION )
			{
				return PID_Position_Calc( pid, set, ref );
			}
			else if ( mode == DELTA )
			{
				return PID_Delta_Calc( pid, set, ref );
			}
			else
			{
				return 0;
			}
		
		#else
		pid->pidCycle.nowTime = millis();
		pid->pidCycle.passTime = pid->pidCycle.nowTime - pid->pidCycle.lastTime;
		if( pid->pidCycle.passTime >= pid->pidCycle.sampleTime )
		{
			if( mode == POSITION )
			{
				return PID_Position_Calc( pid, set, ref );
			}
			else if ( mode == DELTA )
			{
				return PID_Delta_Calc( pid, set, ref );
			}
			else
			{
				pid->pidCycle.lastTime = millis();
				return 0;
			}
		}
		else
		{
			pid->pidCycle.lastTime = millis();
			return 0;
		}
		#endif
}

/**
  * @brief          λ��ʽPID
  * @param[in]      pid: PID�ṹ����ָ��
  * @param[in]      set: �趨ֵ
  * @param[in]      ref: ����ֵ
  * @retval         λ��ʽPID���ֵ
  */
float PID_Position_Calc( PidTypeDef *pid, float set, float ref)
{
    pid->set = set;
		pid->ref = ref;
		
		pid->error[0] = set - ref;
	
		if( pid->PidChanger.fuzzyController == true )
		{	
			Fuzzification( pid, set, ref );
			Defuzzification( pid );
		}
			
		if( ABS( pid->error[0] ) < pid->dead_band ) //PID����
		{
			PID_Clear(pid);
			#ifndef useFreeRTOS
			pid->pidCycle.lastTime = millis();
			#endif
			return 0.0f;
		}
		else
		{
			pid->P_out = pid->Kp * pid->error[0];
		
			if( ABS( pid->error[0] ) < pid->band_I ) //���ַ���
			{
				if( pid->PidChanger.trapezoidalintegral == false )//û�п������λ��֣�ʹ�ÿ����ͻ��ֺͱ��ٻ���
				{
					if( pid->out >  pid->max_out ){
						if( pid->error[0] < 0 ){
							pid->I_out += pid->Ki * pid->error[0]\
														* variableIntegralCoefficientCalc( &pid->maxInterval, &pid->minInterval, &pid->error[0] );
						}
					}
					else{
							pid->I_out += pid->Ki * pid->error[0]\
														* variableIntegralCoefficientCalc( &pid->maxInterval, &pid->minInterval, &pid->error[0] );
					}
				}
				else if( pid->PidChanger.trapezoidalintegral == true )//�������λ��֣�ʹ�ÿ����ͻ��֣����ٻ��ֺ����λ���
				{
					if( pid->out >  pid->max_out ){
						if( pid->error[0] < 0 ){
							pid->I_out += pid->Ki * ( pid->error[0] + pid->error[1] ) / 2\
														* variableIntegralCoefficientCalc( &pid->maxInterval, &pid->minInterval, &pid->error[0] );
						}
					}
					else{
							pid->I_out += pid->Ki * ( pid->error[0] + pid->error[1] ) / 2\
														* variableIntegralCoefficientCalc( &pid->maxInterval, &pid->minInterval, &pid->error[0] );
					}
				}
				else//״̬����
				{
					pid->I_out += pid->Ki * pid->error[0];
				}
			}
			else//���ַ���
			{
				pid->I_out = 0.0f;
			}
			
			if( pid->PidChanger.processVariable == false )//û�п���΢�����У�ʹ�ò���ȫ΢��
			{
				pid->D_out = pid->Kd * ( 1 - pid->filter_D ) * pid->error[0] + pid->filter_D * pid->D_out
									 - pid->Kd * ( 1 - pid->filter_D ) * pid->error[1];
			}
			else if( pid->PidChanger.processVariable == true )//����΢�����У�ʹ��΢������
			{
				pid->D_out = pid->pvCoefficient * pid->Kd / ( pid->pvCoefficient * pid->Kd + pid->Kp ) * pid->D_out\
										 + ( pid->Kd + pid->Kp ) / ( pid->pvCoefficient * pid->Kd + pid->Kp ) * pid->ref\
										 + pid->Kd / ( pid->pvCoefficient * pid->Kd + pid->Kp ) * pid->lastRef;
			}
			else//״̬����
			{
				pid->D_out = pid->Kd * ( pid->error[0] - pid->error[1] );
			}
			
			pid->Dbuf[0] = pid->D_out;
			
			LimitMax(pid->I_out, pid->max_Iout);
			
			pid->out = pid->P_out + pid->I_out + pid->D_out;
			
			pid->lastRef = pid->ref;
			
			pid->error[1] = pid->error[0];
			pid->lastRef = pid->ref;
			
			pid->Dbuf[2] = pid->Dbuf[1];
			pid->Dbuf[1] = pid->Dbuf[0];
			
			if( pid->out >=  pid->max_out ) //����޷�
			{
				#ifndef useFreeRTOS
				pid->pidCycle.lastTime = millis();
				#endif
				return pid->max_out;
			}
			else
			{
				#ifndef useFreeRTOS
				pid->pidCycle.lastTime = millis();
				#endif
				return pid->out;
			}
		}
}

/**
  * @brief          ����ʽPID
  * @param[in]      pid: PID�ṹ����ָ��
  * @param[in]      set: �趨ֵ
  * @param[in]      ref: ����ֵ
  * @retval         ����ʽPID���ֵ
  */
float PID_Delta_Calc( PidTypeDef *pid, float set, float ref )
{
			pid->set = set;
			pid->ref = ref;
			
			pid->error[0] = set - ref;
	
			if( pid->PidChanger.fuzzyController == true )
			{	
				Fuzzification( pid, set, ref );
				Defuzzification( pid );
			}
			
			if( ABS( pid->error[0] ) < pid->dead_band )//PID����
			{
				PID_Clear( pid );
				#ifndef useFreeRTOS
				pid->pidCycle.lastTime = millis();
				#endif
				return 0.0f;
			}
			else
			{
				pid->P_out = pid->Kp * ( pid->error[0] - pid->error[1] );
			
				if( ABS( pid->error[0] ) < pid->band_I )//���ַ���
				{
					if( pid->PidChanger.trapezoidalintegral == false )//û�п������λ��֣�ʹ�ÿ����ͻ��ֺͱ��ٻ���
					{
						if( pid->out >  pid->max_out ){
							if( pid->error[0] < 0 ){
								pid->I_out = pid->Ki * pid->error[0]\
														 * variableIntegralCoefficientCalc( &pid->maxInterval, &pid->minInterval, &pid->error[0] );
							}
						}
						else{
								pid->I_out = pid->Ki * pid->error[0]\
														 * variableIntegralCoefficientCalc( &pid->maxInterval, &pid->minInterval, &pid->error[0] );
						}
					}
					else if( pid->PidChanger.trapezoidalintegral == true )//�������λ��֣�ʹ�ÿ����ͻ��֣����ٻ��ֺ����λ���
					{
						if( pid->out >  pid->max_out ){
							if( pid->error[0] < 0 ){
								pid->I_out += pid->Ki * ( pid->error[0] + pid->error[1] ) / 2\
															* variableIntegralCoefficientCalc( &pid->maxInterval, &pid->minInterval, &pid->error[0] );
							}
						}
						else{
								pid->I_out += pid->Ki * ( pid->error[0] + pid->error[1] ) / 2\
															* variableIntegralCoefficientCalc( &pid->maxInterval, &pid->minInterval, &pid->error[0] );
						}
					}
					else
					{
						pid->I_out = pid->Ki * pid->error[0];
					}
				}
				else
				{
					pid->I_out = 0.0f;
				}
				
				if( pid->PidChanger.processVariable == false )//û�п���΢�����У�ʹ�ò���ȫ΢��
				{
					pid->D_out = pid->Kd * ( 1 - pid->filter_D ) * ( pid->error[0] - 2 * pid->error[1] + pid->error[2] )\
											 + pid->filter_D * ( pid->Dbuf[1] - pid->Dbuf[2]);
				}
				else if( pid->PidChanger.processVariable == true )//����΢�����У�ʹ��΢������
				{
					pid->D_out = pid->pvCoefficient * pid->Kd / ( pid->pvCoefficient * pid->Kd + pid->Kp ) * ( pid->Dbuf[1] - pid->Dbuf[2] )\
											 + ( pid->Kd + pid->Kp ) / ( pid->pvCoefficient * pid->Kd + pid->Kp ) * ( pid->ref - pid->lastRef )\
											 + pid->Kd / ( pid->pvCoefficient * pid->Kd + pid->Kp ) * ( pid->lastRef - pid->perRef );
				}
				else//״̬����
				{
					pid->D_out = pid->Kd * ( pid->error[0] - 2 * pid->error[1] + pid->error[2] );
				}
				
				LimitMax(pid->I_out, pid->max_Iout);
				
				pid->Dbuf[0] = pid->D_out;
				
				pid->out += pid->P_out + pid->I_out + pid->D_out;
				
				pid->error[2] = pid->error[1];
				pid->error[1] = pid->error[0];
				
				pid->perRef = pid->lastRef;
				pid->lastRef = pid->ref;
				
				pid->Dbuf[2] = pid->Dbuf[1];
				pid->Dbuf[1] = pid->Dbuf[0];
				
				if( pid->out >  pid->max_out )//PID����޷�
				{
					#ifndef useFreeRTOS
					pid->pidCycle.lastTime = millis();
					#endif
					return pid->max_out;
				}
				else
				{
					#ifndef useFreeRTOS
					pid->pidCycle.lastTime = millis();
					#endif
					return pid->out;
				}
			}
}

/**
  * @brief          pid������
  * @param[in]      pid: PID�ṹ����ָ��
  * @retval         none
  */
void PID_Clear( PidTypeDef *pid )
{
	  if ( pid == NULL )
		{
			return;
		}
		
    pid->set = pid->ref = pid->lastRef = pid->perRef = 0.0f;
	  pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
		pid->out = pid->P_out = pid->I_out = pid->D_out = 0.0f;
		pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
		
}

/**
  * @brief          ��ȡpid����ģʽ
  * @param[in]      pid: PID�ṹ����ָ��
  * @retval         pid����ģʽ
  */
static PidMode getPIDWorkMode( PidTypeDef *pid )
{
	  if ( pid == NULL )
		{
			return INIT;
		}
		
		return pid->mode;
}

/**
  * @brief          ��ȡpid���ر���
  * @param[in]      pid: PID�ṹ����ָ��
  * @retval         PidChanger�ṹ��
  */
static PidChangerTypeDef getPIDChangerState( PidTypeDef *pid )
{
	  if ( pid == NULL )
		{
			static PidChangerTypeDef PIDChangerNULL;
			
			return PIDChangerNULL;
		}
		
		return pid->PidChanger;
}

/**
  * @brief          ���ٻ���ϵ�����㺯��
  * @param[in]      *maxInterval: ������������
  * @param[in]      *minInterval: ������������
  * @param[in]      *thiserror: 	�������
  * @retval         ���ٻ���ϵ��
  */
static float variableIntegralCoefficientCalc( float* maxInterval, float* minInterval, float* thiserror )
{
		if( maxInterval == NULL ||  minInterval == NULL || thiserror == NULL)
		{
			return 0;
		}
		if( ABS( *thiserror ) <= *minInterval )
		{
			return 1.0;
		}
		else if( ABS( *thiserror ) > *maxInterval )
		{
			return 0;
		}
		else 
		{
			return (( *maxInterval - ABS( *thiserror )) / ( *maxInterval - *minInterval ));
		}
}

/**
  * @brief          ����������
  * @param[in]      pid: pid�ṹ����ָ��
  * @retval         none
  */
static void Defuzzification( PidTypeDef *pid )
{
  pid->fuzzyPID.deltaKp = ( Kp_Rules_Table[pid->fuzzyPID.IndexE[0]][pid->fuzzyPID.IndexER[0]] * pid->fuzzyPID.MembershipE[0] * pid->fuzzyPID.MembershipER[0] ) + 
													( Kp_Rules_Table[pid->fuzzyPID.IndexE[0]][pid->fuzzyPID.IndexER[1]] * pid->fuzzyPID.MembershipE[0] * pid->fuzzyPID.MembershipER[1] ) +
													( Kp_Rules_Table[pid->fuzzyPID.IndexE[1]][pid->fuzzyPID.IndexER[0]] * pid->fuzzyPID.MembershipE[1] * pid->fuzzyPID.MembershipER[0] ) +
													( Kp_Rules_Table[pid->fuzzyPID.IndexE[1]][pid->fuzzyPID.IndexER[1]] * pid->fuzzyPID.MembershipE[1] * pid->fuzzyPID.MembershipER[1] );
	
  pid->fuzzyPID.deltaKi = ( Ki_Rules_Table[pid->fuzzyPID.IndexE[0]][pid->fuzzyPID.IndexER[0]] * pid->fuzzyPID.MembershipE[0] * pid->fuzzyPID.MembershipER[0] ) + 
													( Ki_Rules_Table[pid->fuzzyPID.IndexE[0]][pid->fuzzyPID.IndexER[1]] * pid->fuzzyPID.MembershipE[0] * pid->fuzzyPID.MembershipER[1] ) +
													( Ki_Rules_Table[pid->fuzzyPID.IndexE[1]][pid->fuzzyPID.IndexER[0]] * pid->fuzzyPID.MembershipE[1] * pid->fuzzyPID.MembershipER[0] ) +
													( Ki_Rules_Table[pid->fuzzyPID.IndexE[1]][pid->fuzzyPID.IndexER[1]] * pid->fuzzyPID.MembershipE[1] * pid->fuzzyPID.MembershipER[1] );
	
	pid->fuzzyPID.deltaKd = ( Kd_Rules_Table[pid->fuzzyPID.IndexE[0]][pid->fuzzyPID.IndexER[0]] * pid->fuzzyPID.MembershipE[0] * pid->fuzzyPID.MembershipER[0] ) + 
													( Kd_Rules_Table[pid->fuzzyPID.IndexE[0]][pid->fuzzyPID.IndexER[1]] * pid->fuzzyPID.MembershipE[0] * pid->fuzzyPID.MembershipER[1] ) +
													( Kd_Rules_Table[pid->fuzzyPID.IndexE[1]][pid->fuzzyPID.IndexER[0]] * pid->fuzzyPID.MembershipE[1] * pid->fuzzyPID.MembershipER[0] ) +
													( Kd_Rules_Table[pid->fuzzyPID.IndexE[1]][pid->fuzzyPID.IndexER[1]] * pid->fuzzyPID.MembershipE[1] * pid->fuzzyPID.MembershipER[1] );
	
	pid->Kp = pid->Kp +  pid->fuzzyPID.deltaKp * 0.005f;
	pid->Ki = pid->Ki +  pid->fuzzyPID.deltaKi * 0.000f;
	pid->Kd = pid->Kd +  pid->fuzzyPID.deltaKd * 0.005f;
	
	LimitBand( pid->Kp, 20.0f, -20.0f );
	LimitBand( pid->Kd, 20.0f, -20.0f );
}

/**
  * @brief          ģ��������
  * @param[in]      pid: pid�ṹ����ָ��
  * @param[in]      Set: �趨ֵ
  * @param[in]      Ref: ����ֵ
  * @retval         none
  */
static void Fuzzification( PidTypeDef *pid, float set, float ref )
{
	pid->fuzzyPID.errorRate = pid->error[0] - pid->error[1];
	Membership_Calc( pid );
}

/**
  * @brief          �����ȼ��㺯��
  * @param[in]      pid: pid�ṹ����ָ��
  * @retval         none
  */
static void Membership_Calc( PidTypeDef *pid )
{
	if( pid->error[0] < NB )
	{
		pid->fuzzyPID.IndexE[0] = 0;
		pid->fuzzyPID.IndexE[1] = 0;
		pid->fuzzyPID.MembershipE[0] = 1;
		pid->fuzzyPID.MembershipE[1] = 0;
	}
		else if(( pid->error[0] >= NB ) && ( pid->error[0] < NM ))
	{
		pid->fuzzyPID.IndexE[0] = 0;
		pid->fuzzyPID.IndexE[1] = 1;
		pid->fuzzyPID.MembershipE[0] = ( NM - pid->error[0] )/( NM - NB );
		pid->fuzzyPID.MembershipE[1] = 1 - pid->fuzzyPID.MembershipE[0];
	}
		else if(( pid->error[0] >= NM ) && ( pid->error[0] < NS ))
	{
		pid->fuzzyPID.IndexE[0] = 1;
		pid->fuzzyPID.IndexE[1] = 2;
		pid->fuzzyPID.MembershipE[0] = ( NS - pid->error[0] )/( NS - NM );
		pid->fuzzyPID.MembershipE[1] = 1 - pid->fuzzyPID.MembershipE[0];
	}
		else if(( pid->error[0] >= NS ) && ( pid->error[0] < ZO ))
	{
		pid->fuzzyPID.IndexE[0] = 2;
		pid->fuzzyPID.IndexE[1] = 3;
		pid->fuzzyPID.MembershipE[0] = ( ZO - pid->error[0] )/( ZO - NS );
		pid->fuzzyPID.MembershipE[1] = 1 - pid->fuzzyPID.MembershipE[0];
	}
		else if(( pid->error[0] >= ZO ) && ( pid->error[0] < PS ))
	{
		pid->fuzzyPID.IndexE[0] = 3;
		pid->fuzzyPID.IndexE[1] = 4;
		pid->fuzzyPID.MembershipE[0] = ( PS - pid->error[0] )/( PS - ZO );
		pid->fuzzyPID.MembershipE[1] = 1 - pid->fuzzyPID.MembershipE[0];
	}
		else if(( pid->error[0] >= PS ) && ( pid->error[0] < PM ))
	{
		pid->fuzzyPID.IndexE[0] = 4;
		pid->fuzzyPID.IndexE[1] = 5;
		pid->fuzzyPID.MembershipE[0] = ( PM - pid->error[0] )/( PM - PS );
		pid->fuzzyPID.MembershipE[1] = 1 - pid->fuzzyPID.MembershipE[0];
	}
		else if(( pid->error[0] >= PM ) && ( pid->error[0] < PB ))
	{
		pid->fuzzyPID.IndexE[0] = 5;
		pid->fuzzyPID.IndexE[1] = 6;
		pid->fuzzyPID.MembershipE[0] = ( PB - pid->error[0] )/( PB - PM );
		pid->fuzzyPID.MembershipE[1] = 1 - pid->fuzzyPID.MembershipE[0];
	}
		else if( pid->error[0] >= PB )
	{
		pid->fuzzyPID.IndexE[0] = 6;
		pid->fuzzyPID.IndexE[1] = 6;
		pid->fuzzyPID.MembershipE[0] = 0;
		pid->fuzzyPID.MembershipE[1] = 1;
	}
	
if( pid->fuzzyPID.errorRate < NB )
	{
		pid->fuzzyPID.IndexER[0] = 0;
		pid->fuzzyPID.IndexER[1] = 0;
		pid->fuzzyPID.MembershipER[0] = 1;
		pid->fuzzyPID.MembershipER[1] = 0;
	}
		else if(( pid->fuzzyPID.errorRate >= NB ) && ( pid->fuzzyPID.errorRate < NM ))
	{
		pid->fuzzyPID.IndexER[0] = 0;
		pid->fuzzyPID.IndexER[1] = 1;
		pid->fuzzyPID.MembershipER[0] = ( NM - pid->fuzzyPID.errorRate )/( NM - NB );
		pid->fuzzyPID.MembershipER[1] = 1 - pid->fuzzyPID.MembershipER[0];
	}
		else if(( pid->fuzzyPID.errorRate >= NM ) && ( pid->fuzzyPID.errorRate < NS ))
	{
		pid->fuzzyPID.IndexER[0] = 1;
		pid->fuzzyPID.IndexER[1] = 2;
		pid->fuzzyPID.MembershipER[0] = ( NS - pid->fuzzyPID.errorRate )/( NS - NM );
		pid->fuzzyPID.MembershipER[1] = 1 - pid->fuzzyPID.MembershipER[0];
	}
		else if(( pid->fuzzyPID.errorRate >= NS ) && ( pid->fuzzyPID.errorRate < ZO ))
	{
		pid->fuzzyPID.IndexER[0] = 2;
		pid->fuzzyPID.IndexER[1] = 3;
		pid->fuzzyPID.MembershipER[0] = ( ZO - pid->fuzzyPID.errorRate )/( ZO - NS );
		pid->fuzzyPID.MembershipER[1] = 1 - pid->fuzzyPID.MembershipER[0];
	}
		else if(( pid->fuzzyPID.errorRate >= ZO ) && (pid->fuzzyPID.errorRate < PS ))
	{
		pid->fuzzyPID.IndexER[0] = 3;
		pid->fuzzyPID.IndexER[1] = 4;
		pid->fuzzyPID.MembershipER[0] = ( PS - pid->fuzzyPID.errorRate )/( PS - ZO );
		pid->fuzzyPID.MembershipER[1] = 1 - pid->fuzzyPID.MembershipER[0];
	}
		else if(( pid->fuzzyPID.errorRate >= PS ) && ( pid->fuzzyPID.errorRate < PM ))
	{
		pid->fuzzyPID.IndexER[0] = 4;
		pid->fuzzyPID.IndexER[1] = 5;
		pid->fuzzyPID.MembershipER[0] = ( PM - pid->fuzzyPID.errorRate )/( PM - PS );
		pid->fuzzyPID.MembershipER[1] = 1 - pid->fuzzyPID.MembershipER[0];
	}
		else if(( pid->fuzzyPID.errorRate >= PM ) && ( pid->fuzzyPID.errorRate < PB ))
	{
		pid->fuzzyPID.IndexER[0] = 5;
		pid->fuzzyPID.IndexER[1] = 6;
		pid->fuzzyPID.MembershipER[0] = ( PB - pid->fuzzyPID.errorRate )/( PB - PM );
		pid->fuzzyPID.MembershipER[1] = 1 - pid->fuzzyPID.MembershipER[0];
	}
		else if( pid->fuzzyPID.errorRate >= PB )
	{
		pid->fuzzyPID.IndexER[0] = 6;
		pid->fuzzyPID.IndexER[1] = 6;
		pid->fuzzyPID.MembershipER[0] = 0;
		pid->fuzzyPID.MembershipER[1] = 1;
	}
}

#else

void PID_Init(PidTypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}
//mode : 0��ʾ�����趨ref set ֵ  1����ʾ����errֵ����ͨ��setֵ����
fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set,uint8_t mode)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
		if(mode==0){
			 pid->error[0] = set - ref;
		}else if(mode==1){
			 pid->error[0] = set;
    }
		if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

#endif
