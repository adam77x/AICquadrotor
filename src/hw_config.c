/* Library includes. */
#include "hw_config.h"
#include "FreeRTOSConfig.h"
#include "testp.h"

/*-----------------------------------------------------------*/
///////////////// test //////////////////////////



 
// void Uart_Init(void){
// 	
// 	GPIO_InitTypeDef GPIO_InitStructuret;
//  GPIO_InitStructuret.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
// GPIO_InitStructuret.GPIO_Mode = GPIO_Mode_AF;
// GPIO_InitStructuret.GPIO_OType = GPIO_OType_PP;
// GPIO_InitStructuret.GPIO_PuPd = GPIO_PuPd_NOPULL;
// GPIO_InitStructuret.GPIO_Speed = GPIO_Speed_50MHz;
// GPIO_Init(GPIOC, &GPIO_InitStructuret);
// //Connect USART pins to AF
// GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
// GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
// 	
// 	
// 	//Enable GPIO Clocks For USART2
// RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
// //Enable Clocks for USART2
// RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
// 	
// 	
// 	USART_InitTypeDef USART_InitStructuret;


//     /******** USART ?????? ********/
//     USART_InitStructuret.USART_BaudRate = 9600; // ?? USART ?? (?????) ? 9600
//     USART_InitStructuret.USART_WordLength = USART_WordLength_8b; // ?? USART ???????? 8
//     USART_InitStructuret.USART_StopBits = USART_StopBits_1; // ?? USART ????? 1
//     USART_InitStructuret.USART_Parity = USART_Parity_No; // ????????
//     USART_InitStructuret.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ???????
//     USART_InitStructuret.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // ?? USART ??? Rx (??) ? Tx (??)
//    
// 								//Configuring And Enabling USART2	
// 		USART_Init(USART3, &USART_InitStructuret); // ???? USART ??,????UART3
// 	
// 	  //Enable USART Interrupt ----
// 	  NVIC_InitTypeDef NVIC_InitStructuret;

// 		USART_ClearFlag(USART3, USART_FLAG_TC);
// 		/* Enable transmit and receive interrupts for the USART2. */
// 		USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
// 		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);  

// 		NVIC_InitStructuret.NVIC_IRQChannel = USART2_IRQn;
// 		NVIC_InitStructuret.NVIC_IRQChannelPreemptionPriority = configMAX_SYSCALL_INTERRUPT_PRIORITY + 0x10;
// 		NVIC_InitStructuret.NVIC_IRQChannelSubPriority = 0;
// 		NVIC_InitStructuret.NVIC_IRQChannelCmd = ENABLE;
// 		NVIC_Init(&NVIC_InitStructuret);

//       USART_Cmd(USART3, ENABLE);


// 	
// 	
// } 




///////////////// test /////////////
void prvSetupHardware( void )
{
/* Set the Vector Table base address at 0x08000000 */
NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );
NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
/* Configure LED IOs as output push-pull */
/* Initialize LEDs on STM32F4_Discovery board */
//prvLED_Config(GPIO);
/* Configure User button pin (PA0) as external interrupt -> modes switching */
STM_EVAL_PBInit(BUTTON_USER,BUTTON_MODE_EXTI);
/* Configuration of Timer4 to control LEDs based on MEMS data */
//prvTIM4_Config();
/* Configure LIS302 in order to produce data used for TIM4 reconfiguration and LED control */
RCC_Configuration();
GPIO_Configuration();
TIM_Configuration();
	USART_Configuration();
prvMEMS_Config();
	Uart_Init();
	

}


// void prvLED_Config(char state)
// {
// GPIO_InitTypeDef GPIO_InitStructure;
// /* GPIOD Periph clock enable */
// RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
// /* Configure PD12, PD13, PD14 and PD15 in output push-pull mode */
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
// GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
// GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
// if(state==GPIO)
// {
// /* standard output pin */
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
// GPIO_Init(GPIOD, &GPIO_InitStructure);
// }
// else
// {
// /*-------------------------- GPIO Configuration ----------------------------*/
// /* GPIOD Configuration: Pins 12, 13, 14 and 15 in output push-pull - alternative mode */
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
// GPIO_Init(GPIOD, &GPIO_InitStructure);
// /* Connect TIM4 pins to AF2 */
// GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
// GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
// GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
// GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
// }
// }
/**
* @brief Configures the different system clocks.
* @param None
* @retval None
*/




void RCC_Configuration(void)
{
RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD , ENABLE );
RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE );
//Enable GPIO Clocks For USART2
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//Enable Clocks for USART2
RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
}


/**
* @brief configure the PD12~15 to Timers
* @param None
* @retval None
*/


void GPIO_Configuration(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
//GPIO Configuration for TIM4
GPIO_StructInit(&GPIO_InitStructure); // Reset init structure
	
GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
// // Setup Blue & Green LED on STM32-Discovery Board to use PWM.
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
// //PD12->LED3 PD13->LED4 PD14->LED5 PD15->LED6
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // Alt Function - Push Pull
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
GPIO_Init( GPIOD, &GPIO_InitStructure );
/*----------------------------------------------------------------------*/
//GPIO Configuration for USART - PA2, PA3
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOA, &GPIO_InitStructure);
//Connect USART pins to AF
GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
}


/**
* @brief configure the TIM4 for PWM mode
* @param None
* @retval None
*/


void TIM_Configuration(void)
{
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
TIM_OCInitTypeDef TIM_OCInitStruct;
// Let PWM frequency equal 400Hz.
// Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 0.1Hz resolution.
// Solving for prescaler gives 240.
TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
TIM_TimeBaseInitStruct.TIM_Period = 2500 - 1;//6720 - 1;//3360 - 1;
TIM_TimeBaseInitStruct.TIM_Prescaler = 84 - 1;//40 - 1;//500 - 1;
TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseInit( TIM4, &TIM_TimeBaseInitStruct );
	
TIM_OCStructInit( &TIM_OCInitStruct );
TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	
	
// Initial duty cycle equals 0%. Value can range from zero to 65535.
//TIM_Pulse = TIM4_CCR1 register (16 bits)
TIM_OCInitStruct.TIM_Pulse = 0; //(0=Always Off, 65535=Always On)
	
TIM_OC1Init( TIM4, &TIM_OCInitStruct ); // Channel 1 LED
TIM_OC2Init( TIM4, &TIM_OCInitStruct ); // Channel 2 LED
TIM_OC3Init( TIM4, &TIM_OCInitStruct ); // Channel 3 LED
TIM_OC4Init( TIM4, &TIM_OCInitStruct ); // Channel 4 LED

TIM_Cmd( TIM4, ENABLE );

}


/**
* @brief configure the USART
* @param None
* @retval None
*/



void USART_Configuration(void){
    /******** ?? USART?GPIO ??? ********/
    USART_InitTypeDef USART_InitStructure;


    /******** USART ?????? ********/
    USART_InitStructure.USART_BaudRate = 9600; // ?? USART ?? (?????) ? 9600
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // ?? USART ???????? 8
    USART_InitStructure.USART_StopBits = USART_StopBits_1; // ?? USART ????? 1
    USART_InitStructure.USART_Parity = USART_Parity_No; // ????????
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ???????
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // ?? USART ??? Rx (??) ? Tx (??)
   
								//Configuring And Enabling USART2	
		USART_Init(USART2, &USART_InitStructure); // ???? USART ??,????UART3
	
	  //Enable USART Interrupt ----
	  NVIC_InitTypeDef NVIC_InitStructure;

		USART_ClearFlag(USART2, USART_FLAG_TC);
		/* Enable transmit and receive interrupts for the USART2. */
		USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);  

		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configMAX_SYSCALL_INTERRUPT_PRIORITY + 0x10;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

      USART_Cmd(USART2, ENABLE);


}

/*------------below is original code------------*/
void prvMEMS_Config(void)
{
uint8_t ctrl = 0;
LIS3DSH_InitTypeDef LIS3DSH_InitStruct;
LIS3DSH_InitStruct.SM1_Hysteresis=0x00;
LIS3DSH_InitStruct.SM1_Pin=LIS3DSH_SM1_INT_TO_PIN_INT1;
LIS3DSH_InitStruct.SM1_Enable=LIS3DSH_SM1_DISABLE;
LIS3DSH_InitStruct.SM2_Hysteresis=0x00;
LIS3DSH_InitStruct.SM2_Pin=LIS3DSH_SM2_INT_TO_PIN_INT1;
LIS3DSH_InitStruct.SM2_Enable=LIS3DSH_SM2_DISABLE;
LIS3DSH_InitStruct.CR3_Dren=LIS3DSH_CR3_DREN_TO_INT1_DISABLE;
LIS3DSH_InitStruct.CR3_Iea=LIS3DSH_CR3_IEA_ACTIVE_LOW;
LIS3DSH_InitStruct.CR3_Iel=LIS3DSH_CR3_IEL_LATCHED;
LIS3DSH_InitStruct.CR3_Int2En=LIS3DSH_CR3_INT2_DISABLED;
LIS3DSH_InitStruct.CR3_Int1En=LIS3DSH_CR3_INT1_DISABLED;
LIS3DSH_InitStruct.CR3_Vfilt=LIS3DSH_CR3_VFILT_DISABLED;
LIS3DSH_InitStruct.CR3_Strt=LIS3DSH_CR3_NO_SOFT_RESET;
LIS3DSH_InitStruct.CR4_Odr=LIS3DSH_CR4_ODR_100HZ;
LIS3DSH_InitStruct.CR4_Bdu=LIS3DSH_CR4_BDU_ENABLED;
LIS3DSH_InitStruct.CR4_Zen=LIS3DSH_CR4_Z_AXIS_ENABLED;
LIS3DSH_InitStruct.CR4_Yen=LIS3DSH_CR4_Y_AXIS_ENABLED;
LIS3DSH_InitStruct.CR4_Xen=LIS3DSH_CR4_X_AXIS_ENABLED;
LIS3DSH_InitStruct.CR5_Bw=LIS3DSH_CR5_BW_50HZ;
LIS3DSH_InitStruct.CR5_Fscale=LIS3DSH_CR5_FSCALE_2G;
LIS3DSH_InitStruct.CR5_St=LIS3DSH_CR5_ST_DISABLE;
LIS3DSH_InitStruct.CR5_Sim=LIS3DSH_CR5_MODE_4_WIRE_INTERFACE;
LIS3DSH_InitStruct.CR6_Boot=LIS3DSH_CR6_FORCE_REBOOT_DISABLE;
LIS3DSH_InitStruct.CR6_FifoEn=LIS3DSH_CR6_FIFO_DISABLED;
LIS3DSH_InitStruct.CR6_WtmEn=LIS3DSH_CR6_WTM_DISABLED;
LIS3DSH_InitStruct.CR6_AddInc=LIS3DSH_CR6_ADDINC_DISABLED;
LIS3DSH_InitStruct.CR6_P1Empty=LIS3DSH_CR6_FIFO_EMPTY_TO_INT1_DISABLED;
LIS3DSH_InitStruct.CR6_P1Wtm=LIS3DSH_CR6_FIFO_WTM_TO_INT1_DISABLED;
LIS3DSH_InitStruct.CR6_P1OverRun=LIS3DSH_CR6_FIFO_OVERRUN_TO_INT1_DISABLED;
LIS3DSH_InitStruct.CR6_P2Boot=LIS3DSH_CR6_BOOT_TO_INT2_DISABLED;
LIS3DSH_Init(&LIS3DSH_InitStruct);	
}


/**
* @brief MEMS accelerometer management of the timeout situation.
* @param None.
* @retval None.
*/


	
uint32_t LIS3DSH_TIMEOUT_UserCallback(void)
{
/* MEMS Accelerometer Timeout error has occured */
while (1)
{
}
}
// void prvTIM4_Config(void)
// {
// uint16_t PrescalerValue = 0;
// TIM_OCInitTypeDef TIM_OCInitStructure;
// TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
// /* --------------------------- System Clocks Configuration -----------------*/
// /* TIM4 clock enable */
// RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
// /* Compute the prescaler value */
// PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 2000) - 1;
// /* Time base configuration */
// TIM_TimeBaseStructure.TIM_Period = TIM_ARR;
// TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
// TIM_TimeBaseStructure.TIM_ClockDivision = 0;
// TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
// TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
// /* Enable TIM4 Preload register on ARR */
// TIM_ARRPreloadConfig(TIM4, ENABLE);
// /* TIM PWM1 Mode configuration: Channel */
// TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
// TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
// TIM_OCInitStructure.TIM_Pulse = TIM_CCR;
// TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
// /* Output Compare PWM1 Mode configuration: Channel1 */
// TIM_OC1Init(TIM4, &TIM_OCInitStructure);
// TIM_CCxCmd(TIM4, TIM_Channel_1, DISABLE);
// TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
// /* Output Compare PWM1 Mode configuration: Channel2 */
// TIM_OC2Init(TIM4, &TIM_OCInitStructure);
// TIM_CCxCmd(TIM4, TIM_Channel_2, DISABLE);
// TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
// /* Output Compare PWM1 Mode configuration: Channel3 */
// TIM_OC3Init(TIM4, &TIM_OCInitStructure);
// TIM_CCxCmd(TIM4, TIM_Channel_3, DISABLE);
// TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
// /* Output Compare PWM1 Mode configuration: Channel4 */
// TIM_OC4Init(TIM4, &TIM_OCInitStructure);
// TIM_CCxCmd(TIM4, TIM_Channel_4, DISABLE);
// TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
// }

////////////////////////// test //////////////////////////////////////////////////

// void USART3_IRQHandler(){
//     if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET){
//         // ???? A ??,??? ok ??
//         if((char)USART_ReceiveData(USART3) == '1')
//         {
//             My_Usart3_Printf("ok \n");           
//         }
// 				        if((char)USART_ReceiveData(USART3) == '2')
//         {
//             My_Usart3_Printf("ok \n");           
//         }
// 				        if((char)USART_ReceiveData(USART3) == '3')
//         {
//             My_Usart3_Printf("ok \n");           
//         }
// 				        if((char)USART_ReceiveData(USART3) == '4')
//         {
//             My_Usart3_Printf("ok \n");           
//         }
// 				
//     }
// }



//////////////////////////// test //////////////////////





/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

