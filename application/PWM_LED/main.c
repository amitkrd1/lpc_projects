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

#define SCT_PWM            LPC_SCT0 /* Use SCT0 for PWM */
#define SCT_PWM_PIN_OUT    1        /* COUT1 Generate square wave */
#define SCT_PWM_PIN_LED    0        /* COUT0 [index 2] Controls LED */
#define SCT_PWM_OUT        1        /* Index of OUT PWM */
#define SCT_PWM_LED        2        /* Index of LED PWM */
#define SCT_PWM_RATE   10000        /* PWM frequency 10 KHz */


/* Systick timer tick rate, to change duty cycle */
#define TICKRATE_HZ     1000        /* 1 ms Tick rate */

#define LED_STEP_CNT      20        /* Change LED duty cycle every 20ms */
#define OUT_STEP_CNT    1000        /* Change duty cycle every 1 second */
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




static void pinmux(void)
{
	
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	
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
	
	Chip_SWM_MovablePinAssign(SWM_SCT0_OUT1_O, 29);
	//Chip_SWM_MovablePinAssign(SWM_SCT0_OUT0_O, 9);
	Chip_SWM_MovablePinAssign(SWM_SCT0_OUT0_O, 25);
	
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
	
	uint32_t cnt1 = 0, cnt2 = 0;
int led_dp = 0, led_step = 1, out_dp = 0;
	
	Chip_SCTPWM_Init(SCT_PWM);
	Chip_SCTPWM_SetRate(SCT_PWM, SCT_PWM_RATE);

	/* Setup Board specific output pin */
	//app_setup_pin();

	/* Use SCT0_OUT1 pin */
	Chip_SCTPWM_SetOutPin(SCT_PWM, SCT_PWM_OUT, SCT_PWM_PIN_OUT);
	Chip_SCTPWM_SetOutPin(SCT_PWM, SCT_PWM_LED, SCT_PWM_PIN_LED);

	/* Start with 0% duty cycle */
	Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_OUT, Chip_SCTPWM_GetTicksPerCycle(SCT_PWM)/2);
	Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_LED, 0);
	Chip_SCTPWM_Start(SCT_PWM);


	/* Enable SysTick Timer */
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);
	
	
	while (1) {
		cnt1 ++;
		cnt2 ++;
		if (cnt1 >= OUT_STEP_CNT) {
			out_dp += 10;
			if (out_dp > 100) {
				out_dp = 0;
			}

			/* Increase dutycycle by 10% every second */
			Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_OUT,
				Chip_SCTPWM_PercentageToTicks(SCT_PWM, out_dp));
			cnt1 = 0;
		}

		if (cnt2 >= LED_STEP_CNT) {
			led_dp += led_step;
			if (led_dp < 0) {
				led_dp = 0;
				led_step = 1;
			}
			if (led_dp > 200) {
				led_dp = 200;
				led_step = -1;
			}

			/* Increment or Decrement Dutycycle by 0.5% every 10ms */
			Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_LED,
				Chip_SCTPWM_PercentageToTicks(SCT_PWM, led_dp)/2);
			cnt2 = 0;
		}
		__WFI();
	}

}


 int main(void)
 {	
	
	 prvSetupHardware();
	 pinmux();
	 uart_init();
//	 NVIC_config();
	 
//	 xTaskCreate(vLEDTask1, "vTaskLed1",
//				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
//				(TaskHandle_t *) NULL);
//	 
//	 xTaskCreate(vLEDTask2, "vTaskLed2",
//				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
//				(TaskHandle_t *) NULL);
	 

	 xTaskCreate(DAC_task, "DACTest",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
				(TaskHandle_t *) NULL); 

	 vTaskStartScheduler();
	return 1;		
//the code should never reach here
}
