#include "stm32f4xx.h"
  
void My_Usart3_Printf(char *string){
    while(*string){
        /* ????? USART3 */
        USART_SendData(USART3, (unsigned short int) *string++);
 
        /* ???????? */
        while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
    }
}
 
void Uart_Init(){
    /******** ?? USART?GPIO ??? ********/
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
 
    /******** ?? GPIOC?USART3 ? RCC ?? ********/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
 
    /******** ? PC10?PC11 ??? USART3 ********/
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
 
    /******** ?? PC10 ? Tx ??  ********/
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // ???????
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // ??????
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // ?????
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // ??? 10 ?
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // ?? GPIO ??? 50 MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure); // ???? GPIO ??,???? GPIOC
 
    /******** ?? PC11 ? Rx ??  ********/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // ?????
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; // ??? 11 ?
    GPIO_Init(GPIOC, &GPIO_InitStructure); // ???? GPIO ??,???? GPIOC
 
    /******** USART ?????? ********/
    USART_InitStructure.USART_BaudRate = 9600; // ?? USART ?? (?????) ? 9600
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // ?? USART ???????? 8
    USART_InitStructure.USART_StopBits = USART_StopBits_1; // ?? USART ????? 1
    USART_InitStructure.USART_Parity = USART_Parity_No; // ????????
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ???????
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  // ?? USART ??? Rx (??) ? Tx (??)
    USART_Init(USART3, &USART_InitStructure); // ???? USART ??,????UART3
 
    /******** ?? USART3 ********/
    USART_Cmd(USART3, ENABLE);
 
    /*??&??????*/
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    /* ?? USART3 ?? */
    NVIC_EnableIRQ(USART3_IRQn);
}
 

 
// ???? (??)
// void USART3_IRQHandler(){
//     if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET){
//         // ???? A ??,??? ok ??
//         if((char)USART_ReceiveData(USART3) == 'A')
//         {
//             My_Usart3_Printf("ok \n");           
//         }
//     }
// }
