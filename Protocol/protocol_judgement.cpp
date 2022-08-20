#include "protocol_judgement.h"
#include "protocol_crc.h"
#include <stdio.h>
#include "string.h"

//英雄  裁判系统串口


/*-----UART8_TX-----PE1-----*/
//DMA通道：数据流0 通道5
/*-----UART8_RX-----PE0-----*/
//DMA通道：数据流6 通道5
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

//接收状态
//bit15，	接收完成标志
//bit14，	接收到0xff
//bit13~0，	接收到的有效字节数目
u16 USART8_RX_STA=0;       //接收状态标记	  

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
		USART_ITConfig(UART8, USART_IT_IDLE, ENABLE);//打开空闲中断
   
    USART_Cmd(UART8,ENABLE);//串口使能
     
		 
    nvic.NVIC_IRQChannel = UART8_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 8;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&nvic);
	//	MYDMA_Config(DMA1_Stream0,DMA_Channel_5,(u32)&UART8->DR,(u32)USART8_TX_BUF,TX_BUF_NUM);
			 //相应的DMA配置
	  DMA_DeInit(DMA1_Stream6);//串口8的rx接收是DMA1,流6
    DMA_InitStructure.DMA_Channel = DMA_Channel_5;//通道5
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (UART8->DR);//串口7的地址
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rx8_buf;//接收地址
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到内存
		DMA_InitStructure.DMA_BufferSize = RX_BUF_NUM;//接收区缓存大小
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不增
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址自增
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//以字节为单位
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//以字节为单位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//循环模式 
		DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//优先级高
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;//FIFO模式禁用
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;////FIFO 阈值
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
		DMA_Init(DMA1_Stream6, &DMA_InitStructure);//DMA初始化
		DMA_Cmd(DMA1_Stream6, DISABLE); //Add a disable
    DMA_Cmd(DMA1_Stream6, ENABLE);
}

/*DMA_TX初始化*/
static void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,u16 ndtr)
{ 
 
	DMA_InitTypeDef  DMA_InitStructure;
	
	if((u32)DMA_Streamx>(u32)DMA2)//得到当前stream是属于DMA2还是DMA1
	{
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能 
		
	}else 
	{
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 
	}
	DMA_DeInit(DMA_Streamx);
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}//等待DMA可配置 
	
  /* 配置 DMA Stream */
  DMA_InitStructure.DMA_Channel = chx;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = par;//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = mar;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器到外设模式
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA_Streamx, &DMA_InitStructure);//初始化DMA Stream
	
  //中断配置
  DMA_ITConfig(DMA_Streamx,DMA_IT_TC,ENABLE);  //配置DMA发送完成后产生中断
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream0_IRQn;//串口8 DMA发送中断函数
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;//抢占优先级4
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化nVIC寄存器

  USART_DMACmd(UART8,USART_DMAReq_Tx,ENABLE);  //使能串口8的DMA发送 
  DMA_Cmd (DMA_Streamx,DISABLE);//先不要使能DMA！ 
} 
//开启一次DMA传输
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7 
//ndtr:数据传输量  
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
 
	DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输 
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//确保DMA可以被设置  
		
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量  
 
	DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输 
}	
void DMA1_Stream0_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream0,DMA_IT_TCIF0)!= RESET) //检查DMA传输完成中断 
    {
    DMA_ClearITPendingBit(DMA1_Stream0,DMA_IT_TCIF0); 
		DMA_ClearFlag(DMA1_Stream0,DMA_IT_TCIF0);//清除DMA0_Steam0传输完成标志
		memset(USART8_TX_BUF,0,256);
    }
}
void chassis_to_judgeui(uint16_t txlen){
//	USART_DMACmd(UART8,USART_DMAReq_Tx,ENABLE);  //使能串口8的DMA发送     
//	MYDMA_Enable(DMA1_Stream0,txlen);     //开始一次DMA传输！
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
   if(UART8->SR&(1<<4))//检测到线路空闲
	{

		temp = UART8->SR;
		temp = UART8->DR;
		
	DMA_Cmd(DMA1_Stream6, DISABLE ); //先停止DMA，暂停接收 
	DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);	
	USART_DMACmd(UART8, USART_DMAReq_Rx, DISABLE); //先停止串口8的DMA接收
	UART8_DataLength = RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream6);
	referee_data_solve();	
 	DMA_SetCurrDataCounter(DMA1_Stream6, RX_BUF_NUM); //DMA通道的DMA缓存的大小
 	DMA_Cmd(DMA1_Stream6, ENABLE); //使能USART8 TX DMA1 所指示的通道 
	USART_DMACmd(UART8, USART_DMAReq_Rx, ENABLE); //使能串口8的DMA接收
	DMA_Cmd(DMA1_Stream6, ENABLE); //使能USART8 TX DMA1 所指示的通道 
	}
}

//裁判系统相关
static void get_receiver(hero_robo_data_t *res);
hero_robo_data_t hero_robo_data;
static void referee_data_solve(void){
		static uint16_t start_pos=0,next_start_pos=0;
		memcpy(&judgedatahead.SOF, &rx8_buf[start_pos],FrameHeader_Len);
		/*先校验头帧0xA5 然后crc8校验帧头 再crc16位校验整包*/
		if((judgedatahead.SOF==(uint16_t)JudgeFrameHeader) \
    &&(1==Verify_CRC8_Check_Sum(&rx8_buf[start_pos],FrameHeader_Len)) \
    &&(1==Verify_CRC16_Check_Sum(&rx8_buf[start_pos], judgedatahead.DataLength+FrameHeader_Len+4)))//数据位长度+帧头长度+命令码长度+校验码长度
		{
			memcpy(&judgetype.rxCmdId, (&rx8_buf[start_pos]+5), sizeof(judgetype.rxCmdId));
			rx8_buf[start_pos]++;//每处理完一次就在帧头加一防止再次处理这帧数据
			next_start_pos=start_pos+9+judgedatahead.DataLength;//9为 5位帧头 2位数据长度 2校验位
			switch(judgetype.rxCmdId)
			{
				case CmdID_8://机器人状态数据，10Hz发送；
				{
					memcpy(&game_robot_state_t,(&rx8_buf[start_pos]+7),judgedatahead.DataLength);//把数组中的数据复制到对应的结构体中去
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
			
				case CmdID_9://实时功率热量数据，50Hz周期发送；
				{
					memcpy(&power_heat_data_t, (rx8_buf+7),judgedatahead.DataLength );
						break;
				}	
			
				case CmdID_14://实时射击数据，弹丸发射后发送；
				{
				memcpy(&shoot_data_t,(&rx8_buf[start_pos]+7),judgedatahead.DataLength);
					break;
				}		
			
				case CmdID_10://读取机器人位置信息
				{
					memcpy(&game_robot_pos_t,(&rx8_buf[start_pos]+7),judgedatahead.DataLength);
					break;
				}		
				case CmdID_16:
        {
					memcpy(&judgetype.userinfo,(&rx8_buf[start_pos]+7),judgedatahead.DataLength);//把数组中的数据复制到对应的结构体中去		
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

//串口8发送1个字符 
//c:要发送的字符
void usart8_send_char(uint8_t c)
{
	 USART_SendData(UART8,c);  
	while(USART_GetFlagStatus(UART8,USART_FLAG_TXE)==RESET){};
    
} 
