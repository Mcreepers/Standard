#include "protocol_judgement.h"
#include "protocol_crc.h"
#include <stdio.h>
#include "string.h"

//Ӣ��  ����ϵͳ����


/*-----UART8_TX-----PE1-----*/
//DMAͨ����������0 ͨ��5
/*-----UART8_RX-----PE0-----*/
//DMAͨ����������6 ͨ��5
static ext_game_robot_HP_t          game_robot_HP_t;
static ext_game_status_t            game_status;
static ext_game_robot_status_t      game_robot_state_t;
static judge_type                   judgetype;
static ext_power_heat_data_t        power_heat_data_t;
//static ext_referee_warning_t        referee_warning_t;
//static tFrame			                  tframe;
//static ext_game_robot_pos_t         game_robot_position;
static ext_shoot_data_t	  	        shoot_data_t;
static tMsg_head                    judgedatahead;
static ext_game_robot_pos_t	        game_robot_pos_t;
//static ext_client_custom_graphic_single_t  ext_client_custom_graphic_single;
//static draw_data_struct_t		draw_data_struct;
static void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,u16 ndtr);

/* USARTx */
#define    GPIO_AF_USARTx						GPIO_AF_UART8
/* TX */
#define    GPIO_TX                   GPIOE
#define    GPIO_PIN_TX               GPIO_Pin_1
#define    GPIO_PINSOURCE_TX         GPIO_PinSource1
#define    RCC_AHB1PERIPH_GPIO_TX    RCC_AHB1Periph_GPIOE

/* RX */
#define    GPIO_RX                   GPIOE
#define    GPIO_PIN_RX               GPIO_Pin_0
#define    GPIO_PINSOURCE_RX         GPIO_PinSource0
#define    RCC_AHB1PERIPH_GPIO_RX    RCC_AHB1Periph_GPIOE

uint8_t USART8_TX_BUF[TX_BUF_NUM];

//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0xff
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART8_RX_STA=0;       //����״̬���	  

uint8_t rx8_buf[RX_BUF_NUM];

void usart8_init(void)
{
    USART_InitTypeDef uart8;
    GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
		DMA_InitTypeDef   DMA_InitStructure;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1PERIPH_GPIO_TX | RCC_AHB1PERIPH_GPIO_RX,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART8,ENABLE);

    GPIO_PinAFConfig(GPIO_TX,GPIO_PINSOURCE_TX,GPIO_AF_USARTx);
    GPIO_PinAFConfig(GPIO_RX,GPIO_PINSOURCE_RX,GPIO_AF_USARTx); 

    gpio.GPIO_Pin = GPIO_PIN_TX | GPIO_PIN_RX;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE,&gpio);

    uart8.USART_BaudRate = 115200;          // speed 10byte/ms
    uart8.USART_WordLength = USART_WordLength_8b;
    uart8.USART_StopBits = USART_StopBits_1;
    uart8.USART_Parity = USART_Parity_No;
    uart8.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    uart8.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(UART8,&uart8);

		USART_DMACmd(UART8, USART_DMAReq_Rx, ENABLE);
		USART_ClearFlag(UART8, USART_FLAG_IDLE);
		USART_ITConfig(UART8, USART_IT_IDLE, ENABLE);//�򿪿����ж�
   
    USART_Cmd(UART8,ENABLE);//����ʹ��
     
		 
    nvic.NVIC_IRQChannel = UART8_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 8;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&nvic);
	//	MYDMA_Config(DMA1_Stream0,DMA_Channel_5,(u32)&UART8->DR,(u32)USART8_TX_BUF,TX_BUF_NUM);
			 //��Ӧ��DMA����
	  DMA_DeInit(DMA1_Stream6);//����8��rx������DMA1,��6
    DMA_InitStructure.DMA_Channel = DMA_Channel_5;//ͨ��5
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (UART8->DR);//����7�ĵ�ַ
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rx8_buf;//���յ�ַ
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//���赽�ڴ�
		DMA_InitStructure.DMA_BufferSize = RX_BUF_NUM;//�����������С
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ����
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�ڴ��ַ����
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//���ֽ�Ϊ��λ
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//���ֽ�Ϊ��λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//ѭ��ģʽ 
		DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//���ȼ���
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;//FIFOģʽ����
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;////FIFO ��ֵ
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
		DMA_Init(DMA1_Stream6, &DMA_InitStructure);//DMA��ʼ��
		DMA_Cmd(DMA1_Stream6, DISABLE); //Add a disable
    DMA_Cmd(DMA1_Stream6, ENABLE);
}

/*DMA_TX��ʼ��*/
static void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,u16 ndtr)
{ 
 
	DMA_InitTypeDef  DMA_InitStructure;
	
	if((u32)DMA_Streamx>(u32)DMA2)//�õ���ǰstream������DMA2����DMA1
	{
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2ʱ��ʹ�� 
		
	}else 
	{
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1ʱ��ʹ�� 
	}
	DMA_DeInit(DMA_Streamx);
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}//�ȴ�DMA������ 
	
  /* ���� DMA Stream */
  DMA_InitStructure.DMA_Channel = chx;  //ͨ��ѡ��
  DMA_InitStructure.DMA_PeripheralBaseAddr = par;//DMA�����ַ
  DMA_InitStructure.DMA_Memory0BaseAddr = mar;//DMA �洢��0��ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//�洢��������ģʽ
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// ʹ����ͨģʽ 
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//�е����ȼ�
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
  DMA_Init(DMA_Streamx, &DMA_InitStructure);//��ʼ��DMA Stream
	
  //�ж�����
  DMA_ITConfig(DMA_Streamx,DMA_IT_TC,ENABLE);  //����DMA������ɺ�����ж�
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream0_IRQn;//����8 DMA�����жϺ���
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;//��ռ���ȼ�4
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
  NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��nVIC�Ĵ���

  USART_DMACmd(UART8,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���8��DMA���� 
  DMA_Cmd (DMA_Streamx,DISABLE);//�Ȳ�Ҫʹ��DMA�� 
} 
//����һ��DMA����
//DMA_Streamx:DMA������,DMA1_Stream0~7/DMA2_Stream0~7 
//ndtr:���ݴ�����  
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
 
	DMA_Cmd(DMA_Streamx, DISABLE);                      //�ر�DMA���� 
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//ȷ��DMA���Ա�����  
		
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //���ݴ�����  
 
	DMA_Cmd(DMA_Streamx, ENABLE);                      //����DMA���� 
}	
void DMA1_Stream0_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream0,DMA_IT_TCIF0)!= RESET) //���DMA��������ж� 
    {
    DMA_ClearITPendingBit(DMA1_Stream0,DMA_IT_TCIF0); 
		DMA_ClearFlag(DMA1_Stream0,DMA_IT_TCIF0);//���DMA0_Steam0������ɱ�־
		memset(USART8_TX_BUF,0,256);
    }
}
void chassis_to_judgeui(uint16_t txlen){
//	USART_DMACmd(UART8,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���8��DMA����     
//	MYDMA_Enable(DMA1_Stream0,txlen);     //��ʼһ��DMA���䣡
	u16 i=0;
	u16 tx_len=txlen;
	while(txlen--){
		
		usart8_send_char(USART8_TX_BUF[i++]);
	}
	memset(USART8_TX_BUF,0,tx_len);
	
}

static u8 temp;
static u16 UART8_DataLength;
static void referee_data_solve(void);
void UART8_IRQHandler(void)
{  
   if(UART8->SR&(1<<4))//��⵽��·����
	{

		temp = UART8->SR;
		temp = UART8->DR;
		
	DMA_Cmd(DMA1_Stream6, DISABLE ); //��ֹͣDMA����ͣ���� 
	DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);	
	USART_DMACmd(UART8, USART_DMAReq_Rx, DISABLE); //��ֹͣ����8��DMA����
	UART8_DataLength = RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream6);
	referee_data_solve();	
 	DMA_SetCurrDataCounter(DMA1_Stream6, RX_BUF_NUM); //DMAͨ����DMA����Ĵ�С
 	DMA_Cmd(DMA1_Stream6, ENABLE); //ʹ��USART8 TX DMA1 ��ָʾ��ͨ�� 
	USART_DMACmd(UART8, USART_DMAReq_Rx, ENABLE); //ʹ�ܴ���8��DMA����
	DMA_Cmd(DMA1_Stream6, ENABLE); //ʹ��USART8 TX DMA1 ��ָʾ��ͨ�� 
	}
}

//����ϵͳ���
static void get_receiver(hero_robo_data_t *res);
hero_robo_data_t hero_robo_data;
static void referee_data_solve(void){
		static uint16_t start_pos=0,next_start_pos=0;
		memcpy(&judgedatahead.SOF, &rx8_buf[start_pos],FrameHeader_Len);
		/*��У��ͷ֡0xA5 Ȼ��crc8У��֡ͷ ��crc16λУ������*/
		if((judgedatahead.SOF==(uint16_t)JudgeFrameHeader) \
    &&(1==Verify_CRC8_Check_Sum(&rx8_buf[start_pos],FrameHeader_Len)) \
    &&(1==Verify_CRC16_Check_Sum(&rx8_buf[start_pos], judgedatahead.DataLength+FrameHeader_Len+4)))//����λ����+֡ͷ����+�����볤��+У���볤��
		{
			memcpy(&judgetype.rxCmdId, (&rx8_buf[start_pos]+5), sizeof(judgetype.rxCmdId));
			rx8_buf[start_pos]++;//ÿ������һ�ξ���֡ͷ��һ��ֹ�ٴδ�����֡����
			next_start_pos=start_pos+9+judgedatahead.DataLength;//9Ϊ 5λ֡ͷ 2λ���ݳ��� 2У��λ
			switch(judgetype.rxCmdId)
			{
				case CmdID_8://������״̬���ݣ�10Hz���ͣ�
				{
					memcpy(&game_robot_state_t,(&rx8_buf[start_pos]+7),judgedatahead.DataLength);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
					break;
				}
				
				case CmdID_1:
				{
//////////				//+memcpy(&game_status, (rx7_buf+7), 4);
					memcpy(&game_status, (rx8_buf+7), judgedatahead.DataLength);
					break;
				}
				
				case CmdID_3:
				{
//					memcpy(&game_robot_HP_t, (&rx7_buf[start_pos]+7),32);
					memcpy(&game_robot_HP_t, (&rx8_buf[start_pos]+7),judgedatahead.DataLength);
						break;
				}
			
				case CmdID_9://ʵʱ�����������ݣ�50Hz���ڷ��ͣ�
				{
					memcpy(&power_heat_data_t, (rx8_buf+7),judgedatahead.DataLength );
						break;
				}	
			
				case CmdID_14://ʵʱ������ݣ����跢����ͣ�
				{
				memcpy(&shoot_data_t,(&rx8_buf[start_pos]+7),judgedatahead.DataLength);
					break;
				}		
			
				case CmdID_10://��ȡ������λ����Ϣ
				{
					memcpy(&game_robot_pos_t,(&rx8_buf[start_pos]+7),judgedatahead.DataLength);
					break;
				}		
				case CmdID_16:
        {
					memcpy(&judgetype.userinfo,(&rx8_buf[start_pos]+7),judgedatahead.DataLength);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ		
					break;
				}
				default:{
					break;
				}
			}
			start_pos=next_start_pos;
		}
		else
		{
			start_pos=0;			
		}
		get_receiver(&hero_robo_data);
}

static void get_receiver(hero_robo_data_t *res){
	res->hreo_level=game_robot_state_t.robot_level;
	res->hero_HP=game_robot_state_t.remain_HP;
	res->hero_ID=game_robot_state_t.robot_id;
}

const hero_robo_data_t *get_hero_robo_angle_Point(void){
	return &hero_robo_data;
}

void Usart_SendBuff(u8 *buf,u16 len){
	while(len--){
		usart8_send_char(*buf++);
	}
}

//����8����1���ַ� 
//c:Ҫ���͵��ַ�
void usart8_send_char(uint8_t c)
{
	 USART_SendData(UART8,c);  
	while(USART_GetFlagStatus(UART8,USART_FLAG_TXE)==RESET){};
    
} 
