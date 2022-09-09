/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/
#include "stm32f4xx_it.h"
#include <stdio.h>

struct exceptionInfo_t
{
	uint32_t exc_return;
	uint32_t r4;
	uint32_t r5;
	uint32_t r6;
	uint32_t r7;
	uint32_t r8;
	uint32_t r9;
	uint32_t r10;
	uint32_t r11;
	uint32_t r0;
	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r12;
	uint32_t lr;
	uint32_t pc;
	uint32_t psr;
};

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

int getStackTopAddr( void )
{
	return *( volatile int *)0x00000000;
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Exception( struct exceptionInfo_t *exceptionInfo )
{
	uint32_t *Stack_Ptr = ( uint32_t * )( exceptionInfo + 1 );
	int i;
	
	printf( "psr: 0x%08x\r\n", exceptionInfo->psr );
	printf( "r00: 0x%08x\r\n", exceptionInfo->r0 );
	printf( "r01: 0x%08x\r\n", exceptionInfo->r1 );
	printf( "r02: 0x%08x\r\n", exceptionInfo->r2 );
	printf( "r03: 0x%08x\r\n", exceptionInfo->r3 );
	printf( "r04: 0x%08x\r\n", exceptionInfo->r4 );
	printf( "r05: 0x%08x\r\n", exceptionInfo->r5 );
	printf( "r06: 0x%08x\r\n", exceptionInfo->r6 );
	printf( "r07: 0x%08x\r\n", exceptionInfo->r7 );
	printf( "r08: 0x%08x\r\n", exceptionInfo->r8 );
	printf( "r09: 0x%08x\r\n", exceptionInfo->r9 );
	printf( "r10: 0x%08x\r\n", exceptionInfo->r10 );
	printf( "r11: 0x%08x\r\n", exceptionInfo->r11 );
	printf( "r12: 0x%08x\r\n", exceptionInfo->r12 );
	printf( " lr: 0x%08x\r\n", exceptionInfo->lr );
	printf( " pc: 0x%08x\r\n", exceptionInfo->pc );
	
	printf("stacks: \r\n");
	for( i = 0; i < ( getStackTopAddr() - ( uint32_t )Stack_Ptr ) + 12; )
	{
		printf( "0x%08x ", *Stack_Ptr );
		Stack_Ptr++;
		i++;
		if( i % 16 == 0 )
			printf( "\r\n" );
	}
	printf( "\r\n" );
	
	while(1){}
}

//void HardFault_Handler(void)
//{
//  /* Go to infinite loop when Hard Fault exception occurs */
//  while (1)
//  {
//  }
//}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
__weak void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
__weak void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
__weak void SysTick_Handler(void)
{
   
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
