/*
 * @brief Blinky example using timers and sysTick
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "board.h"
#include "FreeRTOS.h"
#include "task.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
#define SPI_RX_BUFFER_SIZE  16

static uint16_t rx_buff[SPI_RX_BUFFER_SIZE];
/*****************************************************************************
 * Private functions
 ****************************************************************************/
/* Initializes pin muxing for SPI interface - note that SystemInit() may
   already setup your pin muxing at system startup */
static void Init_SPI_PinMux(void)
{
#if (defined(BOARD_NXP_LPCXPRESSO_1549))
	/* Enable the clock to the Switch Matrix */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	/*
	 * Initialize SPI0 pins connect
	 * SCK0: PINASSIGN3[15:8]: Select P0.0
	 * MOSI0: PINASSIGN3[23:16]: Select P0.16
	 * MISO0: PINASSIGN3[31:24] : Select P0.10
	 * SSEL0: PINASSIGN4[7:0]: Select P0.9
	 */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 14, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 12, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 28, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 27, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));

	Chip_SWM_MovablePinAssign(SWM_SPI0_SCK_IO, 14);	/* P0.0 */
	Chip_SWM_MovablePinAssign(SWM_SPI0_MOSI_IO, 12);/* P0.16 */
	Chip_SWM_MovablePinAssign(SWM_SPI0_MISO_IO, 28);/* P0.10 */
	Chip_SWM_MovablePinAssign(SWM_SPI0_SSELSN_0_IO, 27);	/* P0.9 */

	/* Disable the clock to the Switch Matrix to save power */
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
#else
	/* Configure your own SPI pin muxing here if needed */
#warning "No SPI pin muxing defined"
#endif
}

/* Turn on LED to indicate an error */
static void errorSPI(void)
{
	Board_LED_Set(0, true);
	while (1) {}
}


static void pinmux(void)
{
  Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 6, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 7, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	
	Chip_SWM_MovablePortPinAssign(SWM_UART0_RXD_I, 0, 6);
  Chip_SWM_MovablePortPinAssign(SWM_UART0_TXD_O, 0, 7);


}

static void uart_init(void)
{
	
	Chip_UART_Init(LPC_USART0);
	
	Chip_UART_ConfigData(LPC_USART0, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
	Chip_UART_SetBaud(LPC_USART0, 9600);
	Chip_UART_Enable(LPC_USART0);
	Chip_UART_TXEnable(LPC_USART0);	
	
}

static void setupSpiSlave()
{
	SPI_CFG_T spiCfg;
SPI_DELAY_CONFIG_T spiDelayCfg;
	/* Initialize SPI Block */
	Chip_SPI_Init(LPC_SPI0);
	/* Set SPI Config register */
	 spiCfg.ClkDiv = 0;	/* Set Clock divider to zero */
	spiCfg.Mode = SPI_MODE_SLAVE;	/* Enable slave Mode */
	spiCfg.ClockMode = SPI_CLOCK_MODE0;	/* Enable Mode 0 */
	spiCfg.DataOrder = SPI_DATA_MSB_FIRST;	/* Transmit MSB first */
	/* Slave select polarity is active low */
	spiCfg.SSELPol = (SPI_CFG_SPOL0_LO| SPI_CFG_SPOL1_LO | SPI_CFG_SPOL2_LO | SPI_CFG_SPOL3_LO);
	Chip_SPI_SetConfig(LPC_SPI0, &spiCfg);
	/* Set Delay register */
  spiDelayCfg.PreDelay=2;
  	spiDelayCfg.PostDelay = 2;
   spiDelayCfg.FrameDelay = 2;
  spiDelayCfg.TransferDelay = 2;
 	Chip_SPI_DelayConfig(LPC_SPI0, &spiDelayCfg);
	/* Enable Loopback mode for this example */
	//Chip_SPI_EnableLoopBack(LPC_SPI0);
	/* Enable SPI0 */
	Chip_SPI_Enable(LPC_SPI0);
}



static void ReadSpiMssg(uint16_t *xferPtr, uint32_t xferSize)
{
//	uint8_t i;
	
	uint16_t data;
//	uint32_t SPIMsg;
static SPI_DATA_SETUP_T XferSetup;
	/* Setup Transfer structure, this data should be retained for the entire transmission */
	XferSetup.pTx = NULL;	/* Transmit Buffer */
	XferSetup.pRx = xferPtr;/* Receive Buffer */
	XferSetup.DataSize = xferSize;	/* Data size in bits */
	XferSetup.Length = sizeof(xferPtr);	/* Total frame length */
	/* Assert only SSEL0 */
	XferSetup.ssel = SPI_TXCTL_ASSERT_SSEL0 | SPI_TXCTL_DEASSERT_SSEL1 | SPI_TXCTL_DEASSERT_SSEL2 |
					 SPI_TXCTL_DEASSERT_SSEL3;
	XferSetup.TxCnt = 0;
	XferSetup.RxCnt = 0;
	
	
	Chip_SPI_RWFrames_Blocking(LPC_SPI0, &XferSetup);
	
}

/* Sets up system hardware */
static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Board_Init();

	/* Initial LED0 state is off */
	Board_LED_Set(0, false);
}

/* LED1 toggle thread */
static void vLEDTask1(void *pvParameters) {
	bool LedState = false;

	while (1) {
		Board_LED_Set(0, LedState);
		LedState = (bool) !LedState;

		/* About a 3Hz on/off toggle rate */
		vTaskDelay(configTICK_RATE_HZ / 6);
	}
}

/* LED2 toggle thread */
static void vLEDTask2(void *pvParameters) {
	bool LedState = false;

	while (1) {
		Board_LED_Set(1, LedState);
		LedState = (bool) !LedState;

		/* About a 7Hz on/off toggle rate */
		vTaskDelay(configTICK_RATE_HZ / 14);
	}
}

/* UART (or output) thread */
static void vUARTTask(void *pvParameters) {
	int tickCnt = 0;

	while (1) {
		DEBUGOUT("Tick: %d \r\n", tickCnt);
		tickCnt++;

		/* About a 1s delay here */
		vTaskDelay(configTICK_RATE_HZ);
	}
}

static void vSPISlaveTask(void *pvParameters)
{
	
	uint8_t i;
	/* Generic Initialization */
	//SystemCoreClockUpdate();
	//Board_Init();
 // pinmux();
	//uart_init();
	/* Clear activity LED */
	Board_LED_Set(0, false);

	/* Setup SPI pin muxing */
	Init_SPI_PinMux();

	/* Allocate SPI handle, setup rate, and initialize clocking */
	setupSpiSlave();

	/* Loop forever */
	while (1) {
		/* Read simple message over SPI */
		ReadSpiMssg(rx_buff, SPI_RX_BUFFER_SIZE);
	for (i = 0; i < 4; i++) {
			DEBUGOUT("SPI Read Data %d is %x\r\n", i, rx_buff[i]);
		}

		/* Toggle LED to show activity. */
		Board_LED_Toggle(0);
	}

//	/* Code never reaches here. Only used to satisfy standard main() */
//	return 0;
	
	
	
}


/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	main routine for FreeRTOS blinky example
 * @return	Nothing, function should not exit
 */
int main(void)
{
	prvSetupHardware();
  pinmux();
	uart_init();
	/* LED1 toggle thread */
//	xTaskCreate(vLEDTask1, "vTaskLed1",
//				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
//				(TaskHandle_t *) NULL);

	/* LED2 toggle thread */
//	xTaskCreate(vLEDTask2, "vTaskLed2",
//				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
//				(TaskHandle_t *) NULL);

	/* UART output thread, simply counts seconds */
//	xTaskCreate(vUARTTask, "vTaskUart",
//				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
//				(TaskHandle_t *) NULL);
//	
	xTaskCreate(vSPISlaveTask, "vSPISlaveTask",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}
