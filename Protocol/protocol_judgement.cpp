#include "protocol_judgement.h"
#include "protocol_crc.h"
#include <stdio.h>
#include "string.h"
#include "device.h"
//裁判系统串口

//static tFrame			                  tframe;
static tMsg_head                    judgedatahead;
//static ext_client_custom_graphic_single_t  ext_client_custom_graphic_single;
//static draw_data_struct_t		draw_data_struct;
static judge_type_t                   judge_type;


uint8_t rx7_buf[RX_BUF_NUM];
uint8_t	TX7_buf[TX_BUF_NUM];

static void referee_data_solve(void);
uint8_t temp;
uint16_t UART7_DataLength;

void uart7_dma_get(void)
{
//	已在串口空闲中断中被清除
//	if (UART7->SR & (1 << 4))//检测到线路空闲
//	{
//		//软件序列清除IDLE标志位
//		temp = UART7->DR;
//		temp = UART7->SR;


		DMA_Cmd(DMA1_Stream3, DISABLE); //先停止DMA，暂停接收 
		DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
		USART_DMACmd(UART7, USART_DMAReq_Rx, DISABLE); //先停止串口7的DMA接收		
		UART7_DataLength = RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream3);

		referee_data_solve();

		DMA_SetCurrDataCounter(DMA1_Stream3, RX_BUF_NUM); //DMA通道的DMA缓存的大小
		DMA_Cmd(DMA1_Stream3, ENABLE); //使能USART7 TX DMA1 所指示的通道 
		USART_DMACmd(UART7, USART_DMAReq_Rx, ENABLE); //使能串口7的DMA接收
		DMA_Cmd(DMA1_Stream3, ENABLE);

//	}

}

void usart7_DMA_init(void)
{
	usart7_RxDMA.dmaInit(PTM_CR_SGSG_DS, (uint32_t *)&(UART7->DR), (uint32_t *)rx7_buf, RX_BUF_NUM,
		DMA_PeripheralInc_Disable, DMA_MemoryInc_Enable, DMA_PeripheralDataSize_Byte,
		DMA_MemoryDataSize_Byte, DMA_Priority_VeryHigh,
		DMA_FIFOThreshold_1QuarterFull);
	usart7_TxDMA.dmaInit(MTP_NM_SGSG_DS, (uint32_t *)&(UART7->DR), (uint32_t *)TX7_buf, TX_BUF_NUM,
		DMA_PeripheralInc_Disable, DMA_MemoryInc_Enable, DMA_PeripheralDataSize_Byte,
		DMA_MemoryDataSize_Byte, DMA_Priority_High,
		DMA_FIFOThreshold_Full);
//	usart7_RxDMA.InterruptConfig(referee_data_solve);
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
	// USART_DMACmd(UART7, USART_DMAReq_Tx, ENABLE);  //使能串口8的DMA发送
	// MYDMA_Enable(DMA1_Stream1, txlen);     //开始一次DMA传输！
	u16 i = 0;
	while(txlen--){
		usart7_send_char(TX7_buf[i++]);
	}
	memset(TX7_buf, 0, txlen);
}

//裁判系统相关
static void referee_data_solve(void)
{
	static uint16_t start_pos = 0, next_start_pos = 0;
	memcpy(&judgedatahead.SOF, &rx7_buf[start_pos], FrameHeader_Len);
	/*先校验头帧0xA5 然后crc8校验帧头 再crc16位校验整包*/
	if ((judgedatahead.SOF == (uint16_t)JudgeFrameHeader) \
		&& (1 == Verify_CRC8_Check_Sum(&rx7_buf[start_pos], FrameHeader_Len)) \
		&& (1 == Verify_CRC16_Check_Sum(&rx7_buf[start_pos], judgedatahead.DataLength + FrameHeader_Len + 4)))//数据位长度+帧头长度+命令码长度+校验码长度
	{
		memcpy(&judge_type.rxCmdId, (&rx7_buf[start_pos] + 5), sizeof(judge_type.rxCmdId));
		rx7_buf[start_pos]++;//每处理完一次就在帧头加一防止再次处理这帧数据
		next_start_pos = start_pos + 9 + judgedatahead.DataLength;//9为 5位帧头 2位数据长度 2校验位
		switch (judge_type.rxCmdId)
		{
		case CmdID_8://机器人状态数据，10Hz发送；
			{
				memcpy(&judge_type.game_robot_state, (&rx7_buf[start_pos] + 7), judgedatahead.DataLength);//把数组中的数据复制到对应的结构体中去
				break;
			}

		case CmdID_1:
			{
				memcpy(&judge_type.game_status, (rx7_buf + 7), judgedatahead.DataLength);
				break;
			}

		case CmdID_3:
			{
				memcpy(&judge_type.game_robot_HP, (&rx7_buf[start_pos] + 7), judgedatahead.DataLength);
				break;
			}

		case CmdID_9://实时功率热量数据，50Hz周期发送；
			{
				memcpy(&judge_type.power_heat_data, (rx7_buf + 7), judgedatahead.DataLength);
				break;
			}

		case CmdID_14://实时射击数据，弹丸发射后发送；
			{
				memcpy(&judge_type.shoot_data, (&rx7_buf[start_pos] + 7), judgedatahead.DataLength);
				break;
			}

		case CmdID_10://读取机器人位置信息
			{
				memcpy(&judge_type.game_robot_pos, (&rx7_buf[start_pos] + 7), judgedatahead.DataLength);
				break;
			}
		case CmdID_16:
			{
				memcpy(&judge_type.userinfo, (&rx7_buf[start_pos] + 7), judgedatahead.DataLength);//把数组中的数据复制到对应的结构体中去		
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
}

const judge_type_t *get_robo_data_Point(void)
{
	return &judge_type;
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
