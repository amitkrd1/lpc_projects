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


const char inst1[] = "LPC15xx UART1 example using ring buffers\r\n";
const char inst2[] = "Press a key to echo it back or ESC to quit\r\n";
const char inst3[]="UART0 testing TASK-4  \r\n";
const char inst4[]="Its working\r\n";
const char inst5[]="UART2 testing TASK-5  \r\n";
const char inst6[]="its sucessful\r\n";


uint32_t clkrate;
uint32_t systemclkrate;
uint32_t pllinclkrate;
uint32_t plloutclkrate;
uint32_t systickrate;
uint32_t pllfreq;




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

void timer_init()
{
	Chip_SCT_Init(LPC_SCT0);                								               // enable the SCT0 clock

	Chip_SCT_Config(LPC_SCT0, 	SCT_CONFIG_16BIT_COUNTER 	|
                              SCT_CONFIG_AUTOLIMIT_L 		);				       // two 16 bit timers, auto limit

	Chip_SCT_SetControl(LPC_SCT0, SCT_CTRL_PRE_L(119));					           // PRE_L[12:5] = 120-1 (SCT clock = 12MHz/120 = 100 KHz)

	Chip_SCT_SetMatchCount(LPC_SCT0, SCT_MATCH_0, (100000/10)-1);						        // match 0 @ 10 Hz = 100 msec
	Chip_SCT_SetMatchReload(LPC_SCT0, SCT_MATCH_0, (100000/10)-1);

	Chip_SCT_EventState(LPC_SCT0, SCT_EVENT_0 , ENABLE_ALL_STATES);			  // event 0 happens in all state
	Chip_SCT_EventControl(LPC_SCT0, SCT_EVENT_0 , SCT_COMBMODE_MATCH);		// match 0 condition only

	Chip_SCT_SetOutput(LPC_SCT0, SCT_OUTPUT_0 , SCT_EVT_0);					      // event 0 will set   SCT0_OUT0
	Chip_SCT_ClearOutput(LPC_SCT0, SCT_OUTPUT_0, SCT_EVT_0);				      // event 0 will clear SCT0_OUT0

	Chip_SCT_SetConflictResolution(LPC_SCT0, 0 ,SCT_RES_TOGGLE_OUTPUT);		// output 0 toggles on conflict

	Chip_SCT_ClearControl(LPC_SCT0,SCT_CTRL_HALT_L); 
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
		clkrate=Chip_Clock_GetMainClockRate();
		systemclkrate=Chip_Clock_GetSystemClockRate();
	 	pllinclkrate=	Chip_Clock_GetSystemPLLInClockRate();
		plloutclkrate=Chip_Clock_GetSystemPLLOutClockRate();
		//Chip_Clock_GetSysTickClockRate();
		systickrate=Chip_Clock_GetSysTickClockRate();
	
		  printf("%d \r\n",systickrate);
	  //printf("%d \t %d \n",pllinclkrate,plloutclkrate);

		
		//printf("Hello World !!..\n\r");

		/* About a 7Hz on/off toggle rate */
		vTaskDelay(configTICK_RATE_HZ / 14);
	}
}






			
static void GPIO_task(void *pvParameters)
{
Chip_GPIO_Init(LPC_GPIO);

Chip_GPIO_WritePortBit(LPC_GPIO,0,6,1);
	

	vTaskDelay(configTICK_RATE_HZ);
	
	
Chip_GPIO_WritePortBit(LPC_GPIO,0,6,0);
	vTaskDelay(configTICK_RATE_HZ);
}

static void timer_task(void *pvParameters)
{
Chip_Clock_SetSysClockDiv(6);
Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);							        //enable SWM clk
Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT0_O , 1, 1); 				    // P0_25 (red LED) is SCT0_OUT0

timer_init();
while (1)                                              					    // loop forever
	{
   __WFI();
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
	 
	// xTaskCreate(vLEDTask2, "vTaskLed2",
			//	configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
				//(TaskHandle_t *) NULL);
	
	 xTaskCreate(timer_task, "timertask",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
				(TaskHandle_t *) NULL); 

	 vTaskStartScheduler();
	return 1;		
//the code should never reach here
}
