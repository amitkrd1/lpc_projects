#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>


#define USE_INTEGER_CLOCK


#if defined(BOARD_NXP_LPCXPRESSO_1549)
#define LPC_USART       LPC_USART0
#define LPC_IRQNUM      UART0_IRQn
#define LPC_UARTHNDLR   UART0_IRQHandler

#else
#error "No UART setup defined"
#endif

#define UART_TEST_DEFAULT_BAUDRATE 9600



uint32_t clkrate;
uint32_t systemclkrate;
uint32_t pllinclkrate;
uint32_t plloutclkrate;
uint32_t systickrate;
uint32_t pllfreq;


const char *ptr;


//extern char* pFields[NUM_FIELDS];
//extern char* pField;



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
	
		//LedState = (bool) !LedState;
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


	
static void clkfreq_task(void *pvParameters)
{
	
	
	while(1)
	{
	clkrate=Chip_Clock_GetMainClockRate();
		systemclkrate=Chip_Clock_GetSystemClockRate();
	 	pllinclkrate=	Chip_Clock_GetSystemPLLInClockRate();
		plloutclkrate=Chip_Clock_GetSystemPLLOutClockRate();
		//Chip_Clock_GetSysTickClockRate();
		systickrate=Chip_Clock_GetSysTickClockRate();
	
		  //printf("%d \r\n",systickrate);
	  printf("%d \t %d \r \n",pllinclkrate,plloutclkrate);

}

}
 int main(void)
 {	
	
	 prvSetupHardware();
	 pinmux();
	 uart_init();
	 NVIC_config();
	 //clock_setup();
	
	 
	 xTaskCreate(vLEDTask1, "vTaskLed1",
				configMINIMAL_STACK_SIZE, NULL, (0 ),
				(TaskHandle_t *) NULL);
	 
	 xTaskCreate(vLEDTask2, "vTaskLed2",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
				(TaskHandle_t *) NULL);
	 

	 xTaskCreate(clkfreq_task, "clkfreqTest",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
				(TaskHandle_t *) NULL); 

	 vTaskStartScheduler();
	return 1;		
//the code should never reach here
}
