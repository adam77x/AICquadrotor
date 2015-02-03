#include "stm32f4xx.h"
/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "queue.h"

extern xSemaphoreHandle serial_tx_wait_sem;
extern xQueueHandle xQueueUARTRecvie;
extern xTimerHandle xTimerNoSignal;

/* Queue structure used for passing characters. */

typedef struct {
char ch;
} serial_ch_msg;

/* IRQ handler to handle USART2 interruptss (both transmit and receive
* interrupts). */

void USART2_IRQHandler()
{
	
static signed portBASE_TYPE xHigherPriorityTaskWoken;
serial_ch_msg rx_msg;

	
/* If this interrupt is for a transmit... */
if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
/* "give" the serial_tx_wait_sem semaphore to notfiy processes
* that the buffer has a spot free for the next byte.
*/

xSemaphoreGiveFromISR(serial_tx_wait_sem, &xHigherPriorityTaskWoken);
	
/* Diables the transmit interrupt. */
USART_ITConfig(USART2, USART_IT_TXE, DISABLE);

/* If this interrupt is for a receive... */

}

else if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
	
		/* Receive the byte from the buffer. */
		rx_msg.ch = USART_ReceiveData(USART2);
		xTimerReset(xTimerNoSignal, 10); 
	
	  //My_Usart3_Printf(&rx_msg);
		/* Queue the received byte. */
	
   My_Usart3_Printf(" IRQ \n");
	
	 xQueueSendToBackFromISR(xQueueUARTRecvie, &rx_msg, &xHigherPriorityTaskWoken);
  My_Usart3_Printf(" 222 \n");
// 			if(!xQueueSendToFrontFromISR(xQueueUARTRecvie, &rx_msg, &xHigherPriorityTaskWoken)) 
// 				{

// 						//if(xQueueSendToBackFromISR(xQueueUARTRecvie, &rx_msg, &xHigherPriorityTaskWoken)!= pdPASS ) {
// 					/* If there was an error queueing the received byte,
// 					* freeze. */
// 				My_Usart3_Printf(" before \n");
// 					while(1);
// 	 
//  						}
						
  My_Usart3_Printf(" after \n");

}

else {
	
/* Only transmit and receive interrupts should be enabled.
* If this is another type of interrupt, freeze.
*/
		
while(1);
}

if (xHigherPriorityTaskWoken) {
taskYIELD();
	
}
}

void send_byte(char ch)
{
/* Wait until the RS232 port can receive another byte (this semaphore
* is "given" by the RS232 port interrupt when the buffer has room for
* another byte.
*/
while (!xSemaphoreTake(serial_tx_wait_sem, portMAX_DELAY));
/* Send the byte and enable the transmit interrupt (it is disabled by
* the interrupt).
*/
USART_SendData(USART2, ch);
USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
	
}

void send_str(char *str){
while( *str != '\0')
send_byte(*str++);
}
