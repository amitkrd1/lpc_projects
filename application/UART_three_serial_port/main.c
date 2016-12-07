#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
//#include "gps.h"
//#include <time.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>


#define USE_INTEGER_CLOCK
STATIC RINGBUFF_T txring, rxring;
//STATIC RINGBUFF_T txring1, rxring1;               ///Ringbuffer variables
#define UART_RB_SIZE 128


#if defined(BOARD_NXP_LPCXPRESSO_1549)
#define LPC_USART       LPC_USART0
#define LPC_IRQNUM      UART0_IRQn
#define LPC_UARTHNDLR   UART0_IRQHandler

#else
#error "No UART setup defined"
#endif
#define NUM_FIELDS
#define UART_TEST_DEFAULT_BAUDRATE 9600
 char stringbuffer[1024];

static uint8_t rxbuff[UART_RB_SIZE], txbuff[UART_RB_SIZE];
static uint8_t rxbuff1[UART_RB_SIZE],txbuff1[UART_RB_SIZE];
static uint8_t rxbuff2[UART_RB_SIZE],txbuff2[UART_RB_SIZE];


const char inst1[] = "LPC15xx UART1 example using ring buffers\r\n";
const char inst2[] = "Press a key to echo it back or ESC to quit\r\n";
const char inst3[]="UART0 testing TASK-4  \r\n";
const char inst4[]="Its working\r\n";
const char inst5[]="UART2 testing TASK-5  \r\n";
const char inst6[]="its sucessful\r\n";




void LPC_UARTHNDLR(void)
{
	/* Want to handle any errors? Do it here. */

	/* Use default ring buffer handler. Override this with your own
	   code if you need more capability. */
	Chip_UART_IRQRBHandler(LPC_USART, &rxring, &txring);
}


void NVIC_config(void)
{
	
	//NVIC_EnableIRQ(UART0_IRQn);
	//NVIC_EnableIRQ(UART1_IRQn);
	
	//Chip_UART_IntEnable(LPC_USART1, UART_INTEN_RXRDY);
	//Chip_UART_IntDisable(LPC_USART1, UART_INTEN_TXRDY);
	
	NVIC_EnableIRQ(UART0_IRQn );
	NVIC_EnableIRQ(UART1_IRQn );                       //IRQ number for UART0 UART1, UART2 are 21,22,23 respectively
	NVIC_EnableIRQ(UART2_IRQn );
	NVIC_SetPriority(21,0);                 	//0 highest interrupt priority, 7 lowest interrupt priority
	NVIC_SetPriority(22,7); 
	NVIC_SetPriority(23,2);
	
}
static void pinmux(void)
{
	
 Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 13, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
 Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 18, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
 Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 22, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
 Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 23, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 6, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
 Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 7, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	
	Chip_SWM_MovablePortPinAssign(SWM_UART1_RXD_I, 0, 13);
  Chip_SWM_MovablePortPinAssign(SWM_UART1_TXD_O, 0, 18);
	Chip_SWM_MovablePortPinAssign(SWM_UART0_RXD_I, 0, 22);
  Chip_SWM_MovablePortPinAssign(SWM_UART0_TXD_O, 0, 23);
	Chip_SWM_MovablePortPinAssign(SWM_UART2_RXD_I, 0, 6);
  Chip_SWM_MovablePortPinAssign(SWM_UART2_TXD_O, 0, 7);
	
}

static void uart_init(void)
{
	Chip_UART_Init(LPC_USART0);
	Chip_UART_Init(LPC_USART1);
	Chip_UART_Init(LPC_USART2);
	
	Chip_UART_ConfigData(LPC_USART1, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
	Chip_UART_SetBaud(LPC_USART1, UART_TEST_DEFAULT_BAUDRATE);
	Chip_UART_Enable(LPC_USART1);
	Chip_UART_TXEnable(LPC_USART1);
	
	
	Chip_UART_ConfigData(LPC_USART0, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
	Chip_UART_SetBaud(LPC_USART0, UART_TEST_DEFAULT_BAUDRATE);
	Chip_UART_Enable(LPC_USART0);
	Chip_UART_TXEnable(LPC_USART0);
	
		Chip_UART_ConfigData(LPC_USART2, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
	 Chip_UART_SetBaud(LPC_USART2, UART_TEST_DEFAULT_BAUDRATE);
	 Chip_UART_Enable(LPC_USART2);
	 Chip_UART_TXEnable(LPC_USART2);
  
	//Chip_UART_IntEnable(LPC_USART1, UART_INTEN_RXRDY);
	//Chip_UART_IntDisable(LPC_USART1, UART_INTEN_TXRDY);
	
	//NVIC_EnableIRQ(UART0_IRQn);
	//NVIC_EnableIRQ(UART1_IRQn);
}


static void prvSetupHardware(void)
{

	SystemCoreClockUpdate();
	Board_Init();
	/* Initial LED0 state is off */
	Board_LED_Set(0,false);
}

static void vLEDTask1(void *pvParameters) {
	bool LedState = false;

	while (1) {
		Board_LED_Set(2, LedState);
		LedState = (bool) !LedState;
  //printf("hello world \n");
		// About a 3Hz on/off toggle rate 
		vTaskDelay(configTICK_RATE_HZ /6);
	}
}

static void vLEDTask2(void *pvParameters) {
	bool LedState = false;

	while (1) {
		Board_LED_Set(1, LedState);
		LedState = (bool) !LedState;
		
		//printf("Hello World !!..\n\r");

		/* About a 7Hz on/off toggle rate */
		vTaskDelay(configTICK_RATE_HZ / 14);
	}
}

static void UART1_task3(void *pvParameters)
{
	
	

	 RingBuffer_Init(&rxring, rxbuff1, 1, UART_RB_SIZE);
	 RingBuffer_Init(&txring, txbuff1, 1, UART_RB_SIZE);

	  
	   while(1)
		 {
				//Chip_UART_ReadRB(LPC_USART,&rxbuff,stringbuffer,50);
				Chip_UART_SendBlocking(LPC_USART1, inst1, sizeof(inst1) - 1);
			
			 
				Chip_UART_SendBlocking(LPC_USART1, inst2, sizeof(inst2) - 1);
		 }
}
static void UART0_Task4(void *pvParameters)
{
	uint8_t key;
	int bytes;
	
	  RingBuffer_Init(&rxring, rxbuff, 1, UART_RB_SIZE);
	  RingBuffer_Init(&txring, txbuff, 1, UART_RB_SIZE);
	  Chip_UART_TXIntHandlerRB(LPC_USART0,&txring);
		//Chip_UART_IntEnable(LPC_USART0, UART_INTEN_RXRDY);
	  //Chip_UART_IntDisable(LPC_USART0, UART_INTEN_TXRDY);
		
	 
	
			while(1)
			{
			//Chip_UART_ReadBlocking(LPC_USART0,stringbuffer,100);
			Chip_UART_SendBlocking(LPC_USART0, inst3, 100);
			//Chip_UART_ReadRB(LPC_USART0,&rxring,stringbuffer,50);
       //vTaskDelay(configTICK_RATE_HZ);
			//Chip_UART_SendRB(LPC_USART0,&txring,inst3, sizeof(inst3)-1);
				//Chip_UART_SendBlocking(LPC_USART0, inst4, sizeof(inst4) - 1);
	   
}
			}

static void UART2_task5(void *pvParameters)
{
	 RingBuffer_Init(&rxring, rxbuff2, 1, UART_RB_SIZE);
	 RingBuffer_Init(&txring, txbuff2, 1, UART_RB_SIZE);

	  
	   while(1)
		 {
				//Chip_UART_ReadRB(LPC_USART,&rxbuff,stringbuffer,50);
				Chip_UART_SendBlocking(LPC_USART2, inst5, sizeof(inst5) - 1);
			  //vTaskDelay(configTICK_RATE_HZ);
			 
				//Chip_UART_SendBlocking(LPC_USART2, inst6, sizeof(inst6) - 1);
		 }
}


 int main(void)
 {	
	
	 prvSetupHardware();
	 pinmux();
	 uart_init();
	 NVIC_config();
	 
	 xTaskCreate(vLEDTask1, "vTaskLed1",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
				(TaskHandle_t *) NULL);
	 
	 xTaskCreate(vLEDTask2, "vTaskLed2",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
				(TaskHandle_t *) NULL);
	 
	xTaskCreate(UART1_task3, "UART1Test",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
				(TaskHandle_t *) NULL);
  
	 xTaskCreate(UART0_Task4, "UARTTest",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
				(TaskHandle_t *) NULL); 
	 
	 xTaskCreate(UART2_task5, "UARTTest",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
				(TaskHandle_t *) NULL); 

	 vTaskStartScheduler();
	return 1;		
//the code should never reach here
}
