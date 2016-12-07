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

#define match_green_OFF     (5000000)
#define match_green_ON      (4000000)
#define match_red_OFF       (9000000)
#define match_red_ON        (1000000)
#define delay               (10000000)
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


const char *ptr;


//extern char* pFields[NUM_FIELDS];
//extern char* pField;


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
	Chip_SCT_Init(LPC_SCT0);													// enable the SCT0 clock

	Chip_SCT_Config(LPC_SCT0, SCT_CONFIG_32BIT_COUNTER |
                            SCT_CONFIG_AUTOLIMIT_L );							// unified timer and auto limit


	Chip_SCT_SetMatchReload(LPC_SCT0, SCT_MATCH_0, delay);						      // match_cycle
	Chip_SCT_SetMatchReload(LPC_SCT0, SCT_MATCH_1, match_green_OFF);			  // match_green_OFF
	Chip_SCT_SetMatchReload(LPC_SCT0, SCT_MATCH_2, match_green_ON);				  // match_green_ON
	Chip_SCT_SetMatchReload(LPC_SCT0, SCT_MATCH_3, match_red_OFF);				  // match_red_OFF
	Chip_SCT_SetMatchReload(LPC_SCT0, SCT_MATCH_4, match_red_ON);				    // match_red_ON

	Chip_SCT_EventState(LPC_SCT0, SCT_EVENT_0, ENABLE_STATE0);					    // event 0 happens in state 0
	Chip_SCT_EventControl(LPC_SCT0, SCT_EVENT_0 , (CHIP_SCT_EVENTCTRL_T) (	SCT_EVECTRL_MATCH0|			// related to match_cycle
                                                                          SCT_IOCOND_LOW	  |			// IN_0 low
                                                                          SCT_COMBMODE_AND  |			// match AND IO condition
                                                                          SCT_STATELD_1	    |			// STATEV is loaded into state
                                                                          SCT_STATEEV_1	    ));		// new state is 1


	Chip_SCT_EventState(LPC_SCT0, SCT_EVENT_1, ENABLE_STATE0); 					    // event 1 happens in state 0
	Chip_SCT_EventControl(LPC_SCT0, SCT_EVENT_1 , (CHIP_SCT_EVENTCTRL_T) (  SCT_EVECTRL_MATCH3 |
                                                                          SCT_COMBMODE_MATCH ));  		// match_red_OFF only condition


	Chip_SCT_EventState(LPC_SCT0, SCT_EVENT_2, ENABLE_STATE0); 					    // event 2 happens in state 0
	Chip_SCT_EventControl(LPC_SCT0, SCT_EVENT_2,  (CHIP_SCT_EVENTCTRL_T) (  SCT_EVECTRL_MATCH4	|
                                                                          SCT_COMBMODE_MATCH	));			// match_red_ON only condition

	Chip_SCT_EventState(LPC_SCT0, SCT_EVENT_3, ENABLE_STATE1); 					    // event 3 happens in state 1
	Chip_SCT_EventControl(LPC_SCT0, SCT_EVENT_3, (CHIP_SCT_EVENTCTRL_T) (   SCT_EVECTRL_MATCH0	|			  // related to match_cycle
                                                                          SCT_IOCOND_HIGH	    |			  // IN_0 high
                                                                          SCT_COMBMODE_AND	  |			  // match AND IO condition
                                                                          SCT_STATELD_1		    |			  // STATEV is loaded into state
                                                                          SCT_STATEEV_0		    ));			// new state is 0


	Chip_SCT_EventState(LPC_SCT0, SCT_EVENT_4, ENABLE_STATE1); 					    // event 4 happens in state 1
	Chip_SCT_EventControl(LPC_SCT0, SCT_EVENT_4, (CHIP_SCT_EVENTCTRL_T) (   SCT_EVECTRL_MATCH2	 |
                                                                          SCT_COMBMODE_MATCH   ));			// match_green_ON only condition

	Chip_SCT_EventState(LPC_SCT0, SCT_EVENT_5, ENABLE_STATE1); 					    // event 5 happens in state 1
	Chip_SCT_EventControl(LPC_SCT0, SCT_EVENT_5, (CHIP_SCT_EVENTCTRL_T) (   SCT_EVECTRL_MATCH1	 |			// match_green_OFF only condition
                                                                          SCT_COMBMODE_MATCH	 ));

	Chip_SCT_SetOutput(LPC_SCT0, SCT_OUTPUT_0,  (CHIP_SCT_EVENT_T)  (    SCT_EVT_0 |
                                                                       SCT_EVT_3 |
                                                                       SCT_EVT_5 )); 					      // event 0, 3 and 5 set OUT0 (green LED)

	Chip_SCT_ClearOutput(LPC_SCT0, SCT_OUTPUT_0, SCT_EVT_4);					      // event 4 clear OUT0 (green LED)

	Chip_SCT_SetOutput(LPC_SCT0, SCT_OUTPUT_1, (CHIP_SCT_EVENT_T)  (  SCT_EVT_0 |
                                                                    SCT_EVT_1 |
                                                                    SCT_EVT_3 ));						      // event 0, 1 and 3 set OUT1 (red LED)

	Chip_SCT_ClearOutput(LPC_SCT0, SCT_OUTPUT_1, SCT_EVT_2);					// event 2 clear OUT1 (red LED)

	Chip_SCT_Output(LPC_SCT0, 0x0F );     										// default set OUT0 and OUT1

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
	
		  //printf("%d \r\n",systickrate);
	  printf("%d \t %d \n",pllinclkrate,plloutclkrate);

		
		//printf("Hello World !!..\n\r");

		/* About a 7Hz on/off toggle rate */
		vTaskDelay(configTICK_RATE_HZ / 14);
	}
}


	
static void timer_task(void *pvParameters)
{
SystemCoreClockUpdate();
	Board_Init();

	Chip_Clock_SetSysClockDiv(6);


	Chip_Clock_EnablePeriphClock((CHIP_SYSCTL_CLOCK_T) ( SYSCTL_CLOCK_SWM 	  | 		// enable SWM clock
                                                             SYSCTL_CLOCK_GPIO0   |		// enable GPIO port 0 clock
                                                             SYSCTL_CLOCK_GPIO1	  | 	        // enable GPIO port 1 clock
                                                             SYSCTL_CLOCK_MUX 	  |		// enable MUX clock
                                                             SYSCTL_CLOCK_IOCON	));             //enable IOCON clock


	Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT0_O , 0, 3);			//SCT0_OUT0 = P0.3 = green LED
	Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT1_O , 0, 25);			//SCT0_OUT1 = P0.25  = red   LED

    Chip_INMUX_SelectSCT0Src(0, SCT0_INMUX_PIO0_17);				// SCT0_IN2  = P0.17 = SW1

   timer_init();                                          			// Initialize SCT0

    while (1)                                              			// loop forever
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
	 
	 xTaskCreate(vLEDTask2, "vTaskLed2",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
				(TaskHandle_t *) NULL);
	 
//	xTaskCreate(UART1_task3, "UART1Test",
//				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
//				(TaskHandle_t *) NULL);
  
//	 xTaskCreate(UART0_Task4, "UARTTest",
//				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
//				(TaskHandle_t *) NULL); 
//	 
	 xTaskCreate(timer_task, "TimerTest",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
				(TaskHandle_t *) NULL); 

	 vTaskStartScheduler();
	return 1;		
//the code should never reach here
}
