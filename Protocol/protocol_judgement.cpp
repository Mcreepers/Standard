#include "protocol_judgement.h"
#include "protocol_crc.h"
#include <stdio.h>
#include "string.h"
#include "device.h"
//����ϵͳ����

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
//	���ڴ��ڿ����ж��б����
//	if (UART7->SR & (1 << 4))//��⵽��·����
//	{
//		//����������IDLE��־λ
//		temp = UART7->DR;
//		temp = UART7->SR;


		DMA_Cmd(DMA1_Stream3, DISABLE); //��ֹͣDMA����ͣ���� 
		DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
		USART_DMACmd(UART7, USART_DMAReq_Rx, DISABLE); //��ֹͣ����7��DMA����		
		UART7_DataLength = RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream3);

		referee_data_solve();

		DMA_SetCurrDataCounter(DMA1_Stream3, RX_BUF_NUM); //DMAͨ����DMA����Ĵ�С
		DMA_Cmd(DMA1_Stream3, ENABLE); //ʹ��USART7 TX DMA1 ��ָʾ��ͨ�� 
		USART_DMACmd(UART7, USART_DMAReq_Rx, ENABLE); //ʹ�ܴ���7��DMA����
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

//����һ��DMA����
//DMA_Streamx:DMA������,DMA1_Stream0~7/DMA2_Stream0~7 
//ndtr:���ݴ�����  
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx, u16 ndtr)
{

	DMA_Cmd(DMA_Streamx, DISABLE);                      //�ر�DMA���� 

	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE) {}	//ȷ��DMA���Ա�����  

	DMA_SetCurrDataCounter(DMA_Streamx, ndtr);          //���ݴ�����  

	DMA_Cmd(DMA_Streamx, ENABLE);                      //����DMA���� 
}

void chassis_to_judgeui(uint16_t txlen)
{
	// USART_DMACmd(UART7, USART_DMAReq_Tx, ENABLE);  //ʹ�ܴ���8��DMA����
	// MYDMA_Enable(DMA1_Stream1, txlen);     //��ʼһ��DMA���䣡
	u16 i = 0;
	while(txlen--){
		usart7_send_char(TX7_buf[i++]);
	}
	memset(TX7_buf, 0, txlen);
}

//����ϵͳ���
static void referee_data_solve(void)
{
	static uint16_t start_pos = 0, next_start_pos = 0;
	memcpy(&judgedatahead.SOF, &rx7_buf[start_pos], FrameHeader_Len);
	/*��У��ͷ֡0xA5 Ȼ��crc8У��֡ͷ ��crc16λУ������*/
	if ((judgedatahead.SOF == (uint16_t)JudgeFrameHeader) \
		&& (1 == Verify_CRC8_Check_Sum(&rx7_buf[start_pos], FrameHeader_Len)) \
		&& (1 == Verify_CRC16_Check_Sum(&rx7_buf[start_pos], judgedatahead.DataLength + FrameHeader_Len + 4)))//����λ����+֡ͷ����+�����볤��+У���볤��
	{
		memcpy(&judge_type.rxCmdId, (&rx7_buf[start_pos] + 5), sizeof(judge_type.rxCmdId));
		rx7_buf[start_pos]++;//ÿ������һ�ξ���֡ͷ��һ��ֹ�ٴδ�����֡����
		next_start_pos = start_pos + 9 + judgedatahead.DataLength;//9Ϊ 5λ֡ͷ 2λ���ݳ��� 2У��λ
		switch (judge_type.rxCmdId)
		{
		case CmdID_8://������״̬���ݣ�10Hz���ͣ�
			{
				memcpy(&judge_type.game_robot_state, (&rx7_buf[start_pos] + 7), judgedatahead.DataLength);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
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

		case CmdID_9://ʵʱ�����������ݣ�50Hz���ڷ��ͣ�
			{
				memcpy(&judge_type.power_heat_data, (rx7_buf + 7), judgedatahead.DataLength);
				break;
			}

		case CmdID_14://ʵʱ������ݣ����跢����ͣ�
			{
				memcpy(&judge_type.shoot_data, (&rx7_buf[start_pos] + 7), judgedatahead.DataLength);
				break;
			}

		case CmdID_10://��ȡ������λ����Ϣ
			{
				memcpy(&judge_type.game_robot_pos, (&rx7_buf[start_pos] + 7), judgedatahead.DataLength);
				break;
			}
		case CmdID_16:
			{
				memcpy(&judge_type.userinfo, (&rx7_buf[start_pos] + 7), judgedatahead.DataLength);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ		
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


//����8����1���ַ� 
//c:Ҫ���͵��ַ�
void usart7_send_char(uint8_t c)
{
	USART_SendData(UART7, c);
	while (USART_GetFlagStatus(UART7, USART_FLAG_TXE) == RESET) {};

}
