#include "protocol_judgement.h"
#include "protocol_crc.h"
#include <stdio.h>
#include "string.h"
#include "device.h"
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
static void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx, u32 chx, u32 par, u32 mar, u16 ndtr);

uint8_t USART8_TX_BUF[TX_BUF_NUM];

//接收状态
//bit15，	接收完成标志
//bit14，	接收到0xff
//bit13~0，	接收到的有效字节数目
u16 USART8_RX_STA = 0;       //接收状态标记	  

uint8_t rx7_buf[RX_BUF_NUM];
uint8_t	tx7_buf[TX_BUF_NUM];

void usart7_DMA_init(void)
{
	usart7_TxDMA.dmaInit(PTM_CR_SGSG_DS, (uint32_t *)(UART7->DR), (uint32_t *)rx7_buf, RX_BUF_NUM,
		DMA_PeripheralInc_Disable, DMA_MemoryInc_Enable, DMA_PeripheralDataSize_Byte,
		DMA_MemoryDataSize_Byte, DMA_Priority_VeryHigh,
		DMA_FIFOThreshold_1QuarterFull);
	usart7_RxDMA.dmaInit(MTP_NM_SGSG_DS, (uint32_t *)(UART7->DR), (uint32_t *)tx7_buf, TX_BUF_NUM,
		DMA_PeripheralInc_Disable, DMA_MemoryInc_Enable, DMA_PeripheralDataSize_Byte,
		DMA_MemoryDataSize_Byte, DMA_Priority_High,
		DMA_FIFOThreshold_Full);
	usart7_TxDMA.InterruptConfig(referee_data_solve);
}

//开启一次DMA传输
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7 
//ndtr:数据传输量  
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx, u16 ndtr)
{

	DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输 

	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE) {}	//确保DMA可以被设置  

	DMA_SetCurrDataCounter(DMA_Streamx, ndtr);          //数据传输量  

	DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输 
}

void chassis_to_judgeui(uint16_t txlen)
{
	USART_DMACmd(UART7, USART_DMAReq_Tx, ENABLE);  //使能串口8的DMA发送     
	MYDMA_Enable(DMA1_Stream1, txlen);     //开始一次DMA传输！
	// u16 i = 0;
	// u16 tx_len=txlen;
	// while(txlen--){
	// 	usart8_send_char(USART8_TX_BUF[i++]);
	// }
	memset(tx7_buf, 0, txlen);
}

static u8 temp;
static u16 UART8_DataLength;
static void referee_data_solve(void);
void UART8_IRQHandler(void)
{
	if (UART8->SR & (1 << 4))//检测到线路空闲
	{

		temp = UART8->SR;
		temp = UART8->DR;

		DMA_Cmd(DMA1_Stream6, DISABLE); //先停止DMA，暂停接收 
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
static void referee_data_solve(void)
{
	static uint16_t start_pos = 0, next_start_pos = 0;
	memcpy(&judgedatahead.SOF, &rx7_buf[start_pos], FrameHeader_Len);
	/*先校验头帧0xA5 然后crc8校验帧头 再crc16位校验整包*/
	if ((judgedatahead.SOF == (uint16_t)JudgeFrameHeader) \
		&& (1 == Verify_CRC8_Check_Sum(&rx7_buf[start_pos], FrameHeader_Len)) \
		&& (1 == Verify_CRC16_Check_Sum(&rx7_buf[start_pos], judgedatahead.DataLength + FrameHeader_Len + 4)))//数据位长度+帧头长度+命令码长度+校验码长度
	{
		memcpy(&judgetype.rxCmdId, (&rx7_buf[start_pos] + 5), sizeof(judgetype.rxCmdId));
		rx7_buf[start_pos]++;//每处理完一次就在帧头加一防止再次处理这帧数据
		next_start_pos = start_pos + 9 + judgedatahead.DataLength;//9为 5位帧头 2位数据长度 2校验位
		switch (judgetype.rxCmdId)
		{
		case CmdID_8://机器人状态数据，10Hz发送；
			{
				memcpy(&game_robot_state_t, (&rx7_buf[start_pos] + 7), judgedatahead.DataLength);//把数组中的数据复制到对应的结构体中去
				break;
			}

		case CmdID_1:
			{
				memcpy(&game_status, (rx7_buf + 7), judgedatahead.DataLength);
				break;
			}

		case CmdID_3:
			{
				memcpy(&game_robot_HP_t, (&rx7_buf[start_pos] + 7), judgedatahead.DataLength);
				break;
			}

		case CmdID_9://实时功率热量数据，50Hz周期发送；
			{
				memcpy(&power_heat_data_t, (rx7_buf + 7), judgedatahead.DataLength);
				break;
			}

		case CmdID_14://实时射击数据，弹丸发射后发送；
			{
				memcpy(&shoot_data_t, (&rx7_buf[start_pos] + 7), judgedatahead.DataLength);
				break;
			}

		case CmdID_10://读取机器人位置信息
			{
				memcpy(&game_robot_pos_t, (&rx7_buf[start_pos] + 7), judgedatahead.DataLength);
				break;
			}
		case CmdID_16:
			{
				memcpy(&judgetype.userinfo, (&rx7_buf[start_pos] + 7), judgedatahead.DataLength);//把数组中的数据复制到对应的结构体中去		
				break;
			}
		default:{
				break;
			}
		}
		start_pos = next_start_pos;
	}
	else
	{
		start_pos = 0;
	}
	get_receiver(&hero_robo_data);
}

static void get_receiver(hero_robo_data_t *res)
{
	res->hreo_level = game_robot_state_t.robot_level;
	res->hero_HP = game_robot_state_t.remain_HP;
	res->hero_ID = game_robot_state_t.robot_id;
}

const hero_robo_data_t *get_hero_robo_angle_Point(void)
{
	return &hero_robo_data;
}

void Usart_SendBuff(u8 *buf, u16 len)
{
	while (len--)
	{
		usart7_send_char(*buf++);
	}
}

//串口8发送1个字符 
//c:要发送的字符
void usart7_send_char(uint8_t c)
{
	USART_SendData(UART7, c);
	while (USART_GetFlagStatus(UART7, USART_FLAG_TXE) == RESET) {};

}
