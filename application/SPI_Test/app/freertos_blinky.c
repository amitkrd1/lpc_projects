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
//#include "LPC15xx.h"

#include "sd_card.h"





/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Macro definitions for single/multiple sector read/write */
#define S_SECTOR_INDEX 103
#define S_FILL_VALUE 0xAA

#define M_SECTOR_INDEX  203
#define M_SECTOR_NUM    2
#define M_FILL_VALUE    0x55

/* The number of bytes to display in terminal */
#define DISPLAY_SIZE    32
/* data buffer */
uint8_t *buf = (uint8_t *)0x2007C000; // 16KB

volatile uint32_t Timer = 0;






/*****************************************************************************
 * Private functions
 ****************************************************************************/


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
	Chip_UART_SetBaud(LPC_USART1, 115200);
	Chip_UART_Enable(LPC_USART1);
	Chip_UART_TXEnable(LPC_USART1);
	
	
	Chip_UART_ConfigData(LPC_USART0, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
	Chip_UART_SetBaud(LPC_USART0, 115200);
	Chip_UART_Enable(LPC_USART0);
	Chip_UART_TXEnable(LPC_USART0);
	
		Chip_UART_ConfigData(LPC_USART2, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
	 Chip_UART_SetBaud(LPC_USART2, 115200);
	 Chip_UART_Enable(LPC_USART2);
	 Chip_UART_TXEnable(LPC_USART2);
  
	
} 
 
 
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
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 16, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 12, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 28, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 27, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));

	Chip_SWM_MovablePinAssign(SWM_SPI0_SCK_IO, 16);	/* P0.0 */
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





/*----------------------------------------------------------------------------
  Timer IRQ: Executed periodically
 *----------------------------------------------------------------------------*/
void SCT0_Init(void)
{

	Chip_SCT_Init(LPC_SCT0);			                   						// enable the SCT2 clock

	Chip_SCT_Config(LPC_SCT0, 	SCT_CONFIG_32BIT_COUNTER  |						// unified timers,
                              SCT_CONFIG_AUTOLIMIT_L    );						// auto limit

	Chip_SCT_SetMatchCount(LPC_SCT0, SCT_MATCH_0,	(SystemCoreClock/1000) - 1);	//match 0 @ 100 Hz = 10 msec
	Chip_SCT_SetMatchReload(LPC_SCT0, SCT_MATCH_0, (SystemCoreClock/1000) - 1);


	Chip_SCT_EventState(LPC_SCT0, SCT_EVENT_0, ENABLE_ALL_STATES);	 			// event 0 happens in all states
	Chip_SCT_EventControl(LPC_SCT0, SCT_EVENT_0, SCT_COMBMODE_MATCH);			// match 0 only condition

	Chip_SCT_EnableEventInt(LPC_SCT0,SCT_EVT_0);								// event 0 generates an interrupt

    NVIC_EnableIRQ(SCT0_IRQn);                             						// enable SCT0 interrupt

    Chip_SCT_ClearControl(LPC_SCT0,SCT_CTRL_HALT_L);							// start timer

}

void SCT0_IRQHandler(void)										// SCT0 Interrupt Handler (10 msec)
{
	 static uint32_t ticks;

	
	
	if (ticks++ >=10)
    {
        disk_timerproc();
        ticks = 0;
    }

    Timer++;
	}

/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
  Test R/W of single sector
 *----------------------------------------------------------------------------*/
void SingleSector_RW_Test()
{
    uint32_t i;

    printf("\n>Single sector read/write test ...\n\n");

    printf("Read sector #%d:\n", S_SECTOR_INDEX);
    if (SD_ReadSector(S_SECTOR_INDEX, buf, 1) == SD_FALSE)
    {
        printf("Failed to read sector %d.\n",  S_SECTOR_INDEX);
        while (1);
    }
    printf("(Only display the first %d bytes to avoid too many content in terminal).\n", DISPLAY_SIZE); 
    for (i=0;i<DISPLAY_SIZE;)
    {        
        printf("%2x ", buf[i++]);
        if ((i%16)==0) printf("\n");
    }

    printf("Fill sector #%d with 0x%x.\n", S_SECTOR_INDEX, S_FILL_VALUE);
    for (i=0;i<512;i++) buf[i] = S_FILL_VALUE;
    if (SD_WriteSector(S_SECTOR_INDEX, buf, 1) == SD_FALSE)
    {
        printf("Failed to write sector %d.\n", S_SECTOR_INDEX);
        while (1);
    }

    printf("Read sector #%d\n", S_SECTOR_INDEX);
    if (SD_ReadSector(S_SECTOR_INDEX, buf, 1) == SD_FALSE)
    {
        printf("Failed to read sector %d.\n",  S_SECTOR_INDEX);
        while (1);
    }
    printf("(Only display the first %d bytes to avoid too many content in terminal).\n", DISPLAY_SIZE);
    for (i=0;i<DISPLAY_SIZE;)
    {        
        printf("%2x ", buf[i++]);
        if ((i%16)==0) printf("\n");
    }
}

/*----------------------------------------------------------------------------
  Test R/W of multiple sectors
 *----------------------------------------------------------------------------*/
void MultiSector_RW_Test ()
{
    uint32_t i;

    printf("\n>Multiple sector read/write test ...\n\n");

    printf("Read %d sectors from #%d:\n", M_SECTOR_NUM, M_SECTOR_INDEX);
    if (SD_ReadSector(M_SECTOR_INDEX, buf, M_SECTOR_NUM) == SD_FALSE)
    {
        printf("Failed to read sectors from %d.\n",  M_SECTOR_INDEX);
        while (1);
    }
    printf("(Only display the first %d bytes to avoid too many content in terminal).\n", DISPLAY_SIZE);
    for (i=0;i<DISPLAY_SIZE;)
    {        
        printf("%2x ", buf[i++]);
        if ((i%16)==0) printf("\n");
    }

    printf("Fill %d sectors from #%d with 0x%x.\n", M_SECTOR_NUM, M_SECTOR_INDEX, M_FILL_VALUE);
    for (i=0;i<512*M_SECTOR_NUM;i++) buf[i] = M_FILL_VALUE;
    if (SD_WriteSector(M_SECTOR_INDEX, buf, M_SECTOR_NUM) == SD_FALSE)
    {
        printf("Failed to write sectors from %d.\n", M_SECTOR_INDEX);
        while (1);
    }

    printf("Read %d sectors from #%d:\n", M_SECTOR_NUM, M_SECTOR_INDEX);
    if (SD_ReadSector(M_SECTOR_INDEX, buf, M_SECTOR_NUM) == SD_FALSE)
    {
        printf("Failed to read sectors from %d.\n",  M_SECTOR_INDEX);
        while (1);
    }
    printf("Only display the first %d bytes to avoid too many content in terminal.\n", DISPLAY_SIZE);
    for (i=0;i<DISPLAY_SIZE;)
    {        
        printf("%2x ", buf[i++]);
        if ((i%16)==0) printf("\n");
    }   
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
uint32_t clkratediv;

	while (1) {
		Board_LED_Set(0, LedState);
		LedState = (bool) !LedState;
		
clkratediv=Chip_SPI_CalClkRateDivider(LPC_SPI0, 400000);
		
   printf("%d \r\n",clkratediv);
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


	
static void SD_cardTask(void *pvParameters)
{
	 uint32_t i;
	uart_init();
	Init_SPI_PinMux();
	


    SysTick_Config(SystemCoreClock/1000 - 1);  /* Generate interrupt each 1 ms      */

    //SER_init(1);                          /* UART#1 Initialization              */

    printf("\nAccess SDC/MMC via SPI on NXP LPC1700. "__DATE__" "__TIME__"\n\n");

    if (SD_Init () == SD_FALSE)
    {
        printf("Failed to init the card, pls check the card.\n");
        while (1);
    }

    if (SD_ReadConfiguration () == SD_FALSE)
    {
        printf("Failed to read card CID or CSD.\n");
        while (1);
    }

    printf("Card init OK.\n\n");
    printf("Card type: ");
    switch (CardType)
    {
        case CARDTYPE_MMC:
            printf("MMC\n");
            break;
        case CARDTYPE_SDV1:
            printf("Version 1.x Standard Capacity SD card.\n");
            break;
        case CARDTYPE_SDV2_SC:
            printf("Version 2.0 or later Standard Capacity SD card.\n");
            break;
        case CARDTYPE_SDV2_HC:
            printf("Version 2.0 or later High/eXtended Capacity SD card.\n");
            break;
        default:
            break;            
    }
    printf("Sector size: %d bytes\n", CardConfig.sectorsize);
    printf("Sector count: %d\n", CardConfig.sectorcnt);
    printf("Block size: %d sectors\n", CardConfig.blocksize);
    printf("Card capacity: %d MByte\n\n", (((CardConfig.sectorcnt >> 10) * CardConfig.sectorsize)) >> 10);
    printf("OCR(hex): ");
    for (i=0;i<4;i++) printf("%02x ", CardConfig.ocr[i]);
    printf("\n");
    printf("CID(hex): ");
    for (i=0;i<16;i++) printf("%02x ", CardConfig.cid[i]);
    printf("\n");
    printf("CSD(hex): ");
    for (i=0;i<16;i++) printf("%02x ", CardConfig.csd[i]);
    printf("\n");


    /* Test read/write of single sector */
    SingleSector_RW_Test ();

    /* Test read/write of multiple sectors */
    MultiSector_RW_Test ();


    /* Read speed test */
    printf("\n>Read speed test ...\n");

    i = 16;
    printf("\nReading %d sectors (%d bytes) ...", i, i*512);
    Timer = 0;
    if (SD_ReadSector(100, buf, i) == SD_FALSE)
    {
        printf("Failed.\n");
        while (1);
    }
    printf(" at speed of %d kB/sec.\n", Timer ? ((i*512) / Timer) : 0);

    i = 32;
    printf("Reading %d sectors (%d bytes) ...", i, i*512);
    Timer = 0;
    if (SD_ReadSector(100, buf, i) == SD_FALSE)
    {
        printf("Failed.\n");
        while (1);
    }
    printf(" at speed of %d kB/sec.\n", Timer ? ((i*512) / Timer) : 0);

    /* Write speed test */
    printf("\n>Write speed test ...\n");
    i = 16;
    printf("\nWriting %d sectors (%d bytes) ...", i, i*512);
    Timer = 0;
    if (SD_WriteSector(100, buf, i) == SD_FALSE)
    {
        printf("Failed.\n");
        while (1);
    }
    printf(" at speed of %d kB/sec.\n", Timer ? ((i*512) / Timer) : 0);

    i = 32;
    printf("Writing %d sectors (%d bytes) ...", i, i*512);
    Timer = 0;
    if (SD_WriteSector(100, buf, i) == SD_FALSE)
    {
        printf("Failed.\n");
        while (1);
    }
    printf(" at speed of %d kB/sec.\n", Timer ? ((i*512) / Timer) : 0);

    printf ("\nTest complete successfully.\n\n");

    while (1); 
	
	
	
	
}



/**
 * @brief	main routine for FreeRTOS blinky example
 * @return	Nothing, function should not exit
 */
int main(void)
{
	
	
	prvSetupHardware();
	pinmux();
	
	

	/* LED1 toggle thread */
	xTaskCreate(vLEDTask1, "vTaskLed1",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

	/* LED2 toggle thread */
	xTaskCreate(vLEDTask2, "vTaskLed2",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

//	/* UART output thread, simply counts seconds */
//	xTaskCreate(vUARTTask, "vTaskUart",
//				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
//				(TaskHandle_t *) NULL);
//	
	
	xTaskCreate(SD_cardTask, "vSDCARDTask",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);


	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}
