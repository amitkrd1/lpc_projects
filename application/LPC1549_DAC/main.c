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

/* DAC internal timer reload value to generate interrupts at 2KHz or 0.5ms*/
#define DAC_TIM_RELOAD ((uint32_t) 0x8C9F)
/* Increments for DAC input at every interrupt to generate a saw tooth wave of 100Hz*/
#define DAC_IN_UPDATE  ((uint32_t) 1)

static volatile uint32_t dac_input;


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



//void NVIC_config(void)
//{
//	
//	//NVIC_EnableIRQ(UART0_IRQn);
//	//NVIC_EnableIRQ(UART1_IRQn);
//	
//	//Chip_UART_IntEnable(LPC_USART1, UART_INTEN_RXRDY);
//	//Chip_UART_IntDisable(LPC_USART1, UART_INTEN_TXRDY);
//	
//	NVIC_EnableIRQ(UART0_IRQn );
//	NVIC_EnableIRQ(UART1_IRQn );                       //IRQ number for UART0 UART1, UART2 are 21,22,23 respectively
//	NVIC_EnableIRQ(UART2_IRQn );
//	NVIC_SetPriority(21,0);                 	//0 highest interrupt priority, 7 lowest interrupt priority
//	NVIC_SetPriority(22,7); 
//	NVIC_SetPriority(23,2);
//	
//}

/**
 * @brief	DAC interrupt handler sub-routine
 * @return	Nothing
 */
void DAC_IRQHandler(void)
{
	
	int voltage_value;
	if (Chip_DAC_GetIntStatus(LPC_DAC)) {
		/* Update DAC Input value*/
		dac_input += DAC_IN_UPDATE;
		printf("%u \n\r",	dac_input);
	//Chip_UART_SendBlocking(LPC_USART0,(const void *)dac_input, sizeof(dac_input) - 1);
		/* Boundary check for max DAC input*/
		if (dac_input > 4095) {
			/* Cycle DAC values */
			dac_input = 0;
		}
		/* Updating DAC Value register will clear interrupt */
		Chip_DAC_UpdateValue(LPC_DAC, dac_input);
		
	}
}



static void pinmux(void)
{
	
 Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 13, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
 Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 18, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
 Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 22, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
 Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 23, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 6, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
 Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 7, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 12, (IOCON_MODE_INACT | IOCON_ADMODE_EN));

	/* Assign DAC_OUT to P0.12 via SWM (fixed pin) */
	Chip_SWM_EnableFixedPin(SWM_FIXED_DAC_OUT);
	
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

		/* About a 7Hz on/off toggle rate */
		vTaskDelay(configTICK_RATE_HZ / 14);
	}
}


static void DAC_task(void *pvParameters)
{
	 /* initialize the DAC */
	Chip_DAC_Init(LPC_DAC);
	/* Setup board specific DAC pin muxing */
//	Init_DAC_PinMux();
	/* Initialize DAC input to 0 */
	dac_input = 0;

	/* Set up DAC internal timer to trigger interrupts every 0.5ms/2KHz */
	Chip_DAC_SetReqInterval(LPC_DAC, 2000);
	Chip_DAC_EnableIntTimer(LPC_DAC);
	/* Disable double buffering */
	Chip_DAC_EnableDoubleBuffer(LPC_DAC);

	/* Set trigger source as Internal Timer */
	Chip_DAC_SetTrgSrcInternal(LPC_DAC);
	/* Start DAC with zero voltage */
	Chip_DAC_UpdateValue(LPC_DAC, dac_input);
	/* Enable the Interrupt for DAC */
	NVIC_EnableIRQ(DAC_IRQ);

	while (1) {
		/* Enter low power mode until DAC interrupt */
		
		__WFI();
	}

}


 int main(void)
 {	
	
	 prvSetupHardware();
	 pinmux();
	 uart_init();
//	 NVIC_config();
	 
	 xTaskCreate(vLEDTask1, "vTaskLed1",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
				(TaskHandle_t *) NULL);
	 
	 xTaskCreate(vLEDTask2, "vTaskLed2",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
				(TaskHandle_t *) NULL);
	 

	 xTaskCreate(DAC_task, "DACTest",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
				(TaskHandle_t *) NULL); 

	 vTaskStartScheduler();
	return 1;		
//the code should never reach here
}
