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
#include "sd_spi.h"
#include "delay.h"
#include "uart.h"






/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
SD_CardInfo cardinfo;


extern uint8_t response[];

// sprintf buffer
char buffer[128];

// SPI read test buffer
uint8_t bf[512];
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
	Chip_SWM_MovablePortPinAssign(SWM_UART0_RXD_I, 0, 6);
  Chip_SWM_MovablePortPinAssign(SWM_UART0_TXD_O, 0, 7);
	Chip_SWM_MovablePortPinAssign(SWM_UART2_RXD_I, 0, 22);
  Chip_SWM_MovablePortPinAssign(SWM_UART2_TXD_O, 0, 23);
	
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
 





/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/



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
		
  // printf("%d \r\n",clkratediv);
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
		//DEBUGOUT("Tick: %d \r\n", tickCnt);
		tickCnt++;

		/* About a 1s delay here */
		vTaskDelay(configTICK_RATE_HZ);
	}
}


	
static void SD_cardTask(void *pvParameters)
{
	
	ErrorCode_t ret = LPC_OK;
//  USB_INTERFACE_DESCRIPTOR* pIntfDesc;
  
  SD_ERROR sderr;
  
  uint32_t t1,t2,i;
  
  /* Initialize board and chip */
  Board_Init();
  
  SystemCoreClockUpdate();
  DWT_Init();
  
  Board_LED_Set(0, false);
  Board_LED_Set(1, false);
  Board_LED_Set(2, false);
  
 init_uart(115200);
    while(1){
			
			
  sderr=init_sd_spi(&cardinfo);
  
  switch(sderr) {
    case SD_OK:
      sprintf(buffer,"\r\n----->Card initialization OK\r\n");
    break;
    case ERROR_GO_IDLE_STATE_TIMEOUT:
      sprintf(buffer,"Error: ERROR_GO_IDLE_STATE_TIMEOUT\r\n");
    break;
    case ERROR_GO_IDLE_STATE_RESPONSE:
      sprintf(buffer,"Error: ERROR_GO_IDLE_STATE_RESPONSE:%d\r\n",response[0]);
    break;
    case ERROR_SEND_IF_COND_TIMEOUT:
      sprintf(buffer,"Error: ERROR_SEND_IF_COND_TIMEOUT\r\n");
    break;
    case ERROR_SEND_IF_COND_RESPONSE:
      sprintf(buffer,"Error: ERROR_SEND_IF_COND_RESPONSE:%02x%02x%02x%02x%02x\r\n",response[0],response[1],response[2],response[3],response[4]);
    break;
    case ERROR_READ_OCR_TIMEOUT:
      sprintf(buffer,"Error: ERROR_READ_OCR_TIMEOUT\r\n");
    break;
    case ERROR_READ_OCR_RESPONSE:
      sprintf(buffer,"Error: ERROR_READ_OCR_RESPONSE:%02x%02x%02x\r\n",response[0],response[1],response[2]);
    break;
    case ERROR_APP_CMD_TIMEOUT:
      sprintf(buffer,"Error: ERROR_APP_CMD_TIMEOUT\r\n");
    break;
    case ERROR_SD_SEND_OP_COND_TIMEOUT:
      sprintf(buffer,"Error: ERROR_SD_SEND_OP_COND_TIMEOUT\r\n");
    break;
    case ERROR_INIT_TIMEOUT:
      sprintf(buffer,"Error: ERROR_INIT_TIMEOUT\r\n");
    break;
    case ERROR_SEND_CID_TIMEOUT:
      sprintf(buffer,"Error: ERROR_SEND_CID_TIMEOUT\r\n");
    break;
    case ERROR_SEND_CID_TOKEN_TIMEOUT:
      sprintf(buffer,"Error: ERROR_SEND_CID_TOKEN_TIMEOUT\r\n");
    break;
    case ERROR_SEND_CSD_TIMEOUT:
      sprintf(buffer,"Error: ERROR_SEND_CSD_TIMEOUT\r\n");
    break;
    case ERROR_SEND_CSD_TOKEN_TIMEOUT:
      sprintf(buffer,"Error: ERROR_SEND_CSD_TOKEN_TIMEOUT\r\n");
    break;
  }
  putLineUART(buffer);
  
  if(sderr!=SD_OK) {
    Board_LED_Set(2, true);
    while(1){};
  }
  
  putLineUART("\r\n----------CARD INFO------------\r\n");
  
  switch(cardinfo.CardType){
    case SD_CARD_STD_CAPACITY_V1_1:sprintf(buffer,"Card Type: SD Card v1.1\r\n");break;
    case SD_CARD_STD_CAPACITY_V2_0:sprintf(buffer,"Card Type: SD Card v2.0\r\n");break;
    case SD_CARD_HIGH_CAPACITY:sprintf(buffer,"Card Type: SDHC Card\r\n");break;
    case MULTIMEDIA_CARD:sprintf(buffer,"Card Type: MMC Card\r\n");break;
  }
  putLineUART(buffer);
  
  sprintf(buffer,"ManufacturerID:%d",cardinfo.SD_cid.ManufacturerID);
  putLineUART(buffer);
  
  sprintf(buffer,"\r\nOEM_AppliID:%c%c",cardinfo.SD_cid.OEM_AppliID>>8,cardinfo.SD_cid.OEM_AppliID&0x00FF);
  putLineUART(buffer);  
  
  sprintf(buffer,"\r\nProdName:%c%c%c%c%c",cardinfo.SD_cid.ProdName1>>24,(cardinfo.SD_cid.ProdName1&0x00FF0000)>>16,
       (cardinfo.SD_cid.ProdName1&0x0000FF00)>>8,cardinfo.SD_cid.ProdName1&0x000000FF,cardinfo.SD_cid.ProdName2);  
  putLineUART(buffer);
  if(cardinfo.CardType==MULTIMEDIA_CARD) {
    sprintf(buffer,"%c",cardinfo.SD_cid.Reserved1);  
    putLineUART(buffer);
  }
  
  sprintf(buffer,"\r\nProdRev:%d.%d",cardinfo.SD_cid.ProdRev>>4,cardinfo.SD_cid.ProdRev&0x0F);
  putLineUART(buffer);
  
  sprintf(buffer,"\r\nProdSN:0x%08X",cardinfo.SD_cid.ProdSN);
  putLineUART(buffer);
  
  if(cardinfo.CardType==MULTIMEDIA_CARD) {
    sprintf(buffer,"\r\nManufactDate:%04d-%02d",(cardinfo.SD_cid.ManufactDate>>4)+1997,cardinfo.SD_cid.ManufactDate&0x000F);
  }
  else {
    sprintf(buffer,"\r\nManufactDate:%04d-%02d",(cardinfo.SD_cid.ManufactDate>>4)+2000,cardinfo.SD_cid.ManufactDate&0x000F);
  }
  putLineUART(buffer);
  
  sprintf(buffer,"\r\nCapacity:%lld MB",cardinfo.CardCapacity/1048576);
  putLineUART(buffer);
  
  sprintf(buffer,"\r\nBlock Size:%d bytes",cardinfo.CardBlockSize);
  putLineUART(buffer);
  
  sprintf(buffer,"\r\nCSDStruct:%d",cardinfo.SD_csd.CSDStruct);
  putLineUART(buffer);
  sprintf(buffer,"\r\nSysSpecVersion:%d",cardinfo.SD_csd.SysSpecVersion);
  putLineUART(buffer);
  sprintf(buffer,"\r\nTAAC:0x%x(",cardinfo.SD_csd.TAAC);
  putLineUART(buffer);
  switch(cardinfo.SD_csd.TAAC>>3){
    case 0:sprintf(buffer,"reserved");break;
    case 1:sprintf(buffer,"1.0");break;
    case 2:sprintf(buffer,"1.2");break;
    case 3:sprintf(buffer,"1.3");break;
    case 4:sprintf(buffer,"1.5");break;
    case 5:sprintf(buffer,"2.0");break;
    case 6:sprintf(buffer,"2.5");break;
    case 7:sprintf(buffer,"3.0");break;
    case 8:sprintf(buffer,"3.5");break;
    case 9:sprintf(buffer,"4.0");break;
    case 10:sprintf(buffer,"4.5");break;
    case 11:sprintf(buffer,"5.0");break;
    case 12:sprintf(buffer,"5.5");break;
    case 13:sprintf(buffer,"6.0");break;
    case 14:sprintf(buffer,"7.0");break;
    case 15:sprintf(buffer,"8.0");break;
  }
  putLineUART(buffer);
  switch(cardinfo.SD_csd.TAAC&0x07){
    case 0:sprintf(buffer," x 1ns)");break;
    case 1:sprintf(buffer," x 10ns)");break;
    case 2:sprintf(buffer," x 100ns)");break;
    case 3:sprintf(buffer," x 1us)");break;
    case 4:sprintf(buffer," x 10us)");break;
    case 5:sprintf(buffer," x 100us)");break;
    case 6:sprintf(buffer," x 1ms)");break;
    case 7:sprintf(buffer," x 10ms)");break;
  }
  putLineUART(buffer);
  sprintf(buffer,"\r\nNSAC:%d",cardinfo.SD_csd.NSAC);
  putLineUART(buffer);
  sprintf(buffer,"\r\nMaxBusClkFrec:0x%x(",cardinfo.SD_csd.MaxBusClkFrec);
  putLineUART(buffer);
  switch((cardinfo.SD_csd.MaxBusClkFrec&0x78)>>3) {
    case 0:sprintf(buffer,"reserved");break;
    case 1:sprintf(buffer,"1.0");break;
    case 2:sprintf(buffer,"1.2");break;
    case 3:sprintf(buffer,"1.3");break;
    case 4:sprintf(buffer,"1.5");break;
    case 5:sprintf(buffer,"2.0");break;
    case 6:sprintf(buffer,"2.5");break;
    case 7:sprintf(buffer,"3.0");break;
    case 8:sprintf(buffer,"3.5");break;
    case 9:sprintf(buffer,"4.0");break;
    case 10:sprintf(buffer,"4.5");break;
    case 11:sprintf(buffer,"5.0");break;
    case 12:sprintf(buffer,"5.5");break;
    case 13:sprintf(buffer,"6.0");break;
    case 14:sprintf(buffer,"7.0");break;
    case 15:sprintf(buffer,"8.0");break;
  }
  putLineUART(buffer);
  switch(cardinfo.SD_csd.MaxBusClkFrec&0x03){
    case 0:sprintf(buffer," x 100kbit/s)");break;
    case 1:sprintf(buffer," x 1Mbit/s");break;
    case 2:sprintf(buffer," x 10Mbit/s)");break;
    case 3:sprintf(buffer," x 100Mbit/s)");break;
    default:sprintf(buffer," reserved)");break;
  }
  putLineUART(buffer);
  sprintf(buffer,"\r\nCardComdClasses:0x%x",cardinfo.SD_csd.CardComdClasses);
  putLineUART(buffer);
  sprintf(buffer,"\r\nRdBlockLen:%d(%db)",cardinfo.SD_csd.RdBlockLen,1<<cardinfo.SD_csd.RdBlockLen);
  putLineUART(buffer);
  sprintf(buffer,"\r\nPartBlockRead:%d(",cardinfo.SD_csd.PartBlockRead);
  putLineUART(buffer);
  if(cardinfo.SD_csd.PartBlockRead==0) {
    sprintf(buffer,"no)");    
  }
  else {
    sprintf(buffer,"yes)");
  }
  putLineUART(buffer);
  sprintf(buffer,"\r\nWrBlockMisalign:%d(",cardinfo.SD_csd.WrBlockMisalign);
  putLineUART(buffer);
  if(cardinfo.SD_csd.WrBlockMisalign==0) {
    sprintf(buffer,"no)");    
  }
  else {
    sprintf(buffer,"yes)");
  }
  putLineUART(buffer);
  sprintf(buffer,"\r\nRdBlockMisalign:%d(",cardinfo.SD_csd.RdBlockMisalign);
  putLineUART(buffer);
  if(cardinfo.SD_csd.RdBlockMisalign==0) {
    sprintf(buffer,"no)");    
  }
  else {
    sprintf(buffer,"yes)");
  }
  putLineUART(buffer);
  sprintf(buffer,"\r\nDSRImpl:%d(",cardinfo.SD_csd.DSRImpl);
  putLineUART(buffer);
  if(cardinfo.SD_csd.DSRImpl==0) {
    sprintf(buffer,"no)");    
  }
  else {
    sprintf(buffer,"yes)");
  }
  putLineUART(buffer);
  
  if(cardinfo.CardType!=SD_CARD_HIGH_CAPACITY) {
    sprintf(buffer,"\r\nMaxRdCurrentVDDMin:%d(",cardinfo.SD_csd.MaxRdCurrentVDDMin);
    putLineUART(buffer);
    switch(cardinfo.SD_csd.MaxRdCurrentVDDMin){
      case 0:sprintf(buffer,"0.5mA)");break;
      case 1:sprintf(buffer,"1mA)");break;
      case 2:sprintf(buffer,"5mA)");break;
      case 3:sprintf(buffer,"10mA)");break;
      case 4:sprintf(buffer,"25mA)");break;
      case 5:sprintf(buffer,"35mA)");break;
      case 6:sprintf(buffer,"60mA)");break;
      case 7:sprintf(buffer,"100mA)");break;      
    }
    putLineUART(buffer);
    sprintf(buffer,"\r\nMaxRdCurrentVDDMax:%d(",cardinfo.SD_csd.MaxRdCurrentVDDMax);
    putLineUART(buffer);
    switch(cardinfo.SD_csd.MaxRdCurrentVDDMax){
      case 0:sprintf(buffer,"1mA)");break;
      case 1:sprintf(buffer,"5mA)");break;
      case 2:sprintf(buffer,"10mA)");break;
      case 3:sprintf(buffer,"25mA)");break;
      case 4:sprintf(buffer,"35mA)");break;
      case 5:sprintf(buffer,"45mA)");break;
      case 6:sprintf(buffer,"80mA)");break;
      case 7:sprintf(buffer,"200mA)");break;      
    }
    putLineUART(buffer);
    sprintf(buffer,"\r\nMaxWrCurrentVDDMin:%d(",cardinfo.SD_csd.MaxWrCurrentVDDMin);
    putLineUART(buffer);
    switch(cardinfo.SD_csd.MaxWrCurrentVDDMin){
      case 0:sprintf(buffer,"0.5mA)");break;
      case 1:sprintf(buffer,"1mA)");break;
      case 2:sprintf(buffer,"5mA)");break;
      case 3:sprintf(buffer,"10mA)");break;
      case 4:sprintf(buffer,"25mA)");break;
      case 5:sprintf(buffer,"35mA)");break;
      case 6:sprintf(buffer,"60mA)");break;
      case 7:sprintf(buffer,"100mA)");break;      
    }
    putLineUART(buffer);
    sprintf(buffer,"\r\nMaxWrCurrentVDDMax:%d(",cardinfo.SD_csd.MaxWrCurrentVDDMax);
    putLineUART(buffer);
    switch(cardinfo.SD_csd.MaxWrCurrentVDDMax){
      case 0:sprintf(buffer,"1mA)");break;
      case 1:sprintf(buffer,"5mA)");break;
      case 2:sprintf(buffer,"10mA)");break;
      case 3:sprintf(buffer,"25mA)");break;
      case 4:sprintf(buffer,"35mA)");break;
      case 5:sprintf(buffer,"45mA)");break;
      case 6:sprintf(buffer,"80mA)");break;
      case 7:sprintf(buffer,"200mA)");break;      
    }
    putLineUART(buffer);
  }
  sprintf(buffer,"\r\nEraseGrSize:%d",cardinfo.SD_csd.EraseGrSize);
  putLineUART(buffer);
  sprintf(buffer,"\r\nEraseGrMul:%d",cardinfo.SD_csd.EraseGrMul);
  putLineUART(buffer);
  sprintf(buffer,"\r\nWrProtectGrSize:%d",cardinfo.SD_csd.WrProtectGrSize);
  putLineUART(buffer);
  sprintf(buffer,"\r\nWrProtectGrEnable:%d(",cardinfo.SD_csd.WrProtectGrEnable);
  putLineUART(buffer);
  if(cardinfo.SD_csd.WrProtectGrEnable==0) {
    sprintf(buffer,"no)");    
  }
  else {
    sprintf(buffer,"yes)");
  }
  putLineUART(buffer);  
  sprintf(buffer,"\r\nWrSpeedFact:%d(x%d)",cardinfo.SD_csd.WrSpeedFact,1<<cardinfo.SD_csd.WrSpeedFact);
  putLineUART(buffer);
  sprintf(buffer,"\r\nMaxWrBlockLen:%d(%db)",cardinfo.SD_csd.MaxWrBlockLen,1<<cardinfo.SD_csd.MaxWrBlockLen);
  putLineUART(buffer);
  sprintf(buffer,"\r\nWriteBlockPaPartial:%d(",cardinfo.SD_csd.WriteBlockPaPartial);
  putLineUART(buffer);
  if(cardinfo.SD_csd.WriteBlockPaPartial==0) {
    sprintf(buffer,"no)");    
  }
  else {
    sprintf(buffer,"yes)");
  }
  putLineUART(buffer);
  sprintf(buffer,"\r\nContentProtectAppli:%d",cardinfo.SD_csd.ContentProtectAppli);
  putLineUART(buffer);
  sprintf(buffer,"\r\nFileFormatGroup:%d",cardinfo.SD_csd.FileFormatGroup);
  putLineUART(buffer);
  sprintf(buffer,"\r\nCopyFlag:%d(",cardinfo.SD_csd.CopyFlag);
  putLineUART(buffer);
  if(cardinfo.SD_csd.CopyFlag==0) {
    sprintf(buffer,"original)");    
  }
  else {
    sprintf(buffer,"copied)");
  }
  putLineUART(buffer);
  sprintf(buffer,"\r\nPermWrProtect:%d(",cardinfo.SD_csd.PermWrProtect);
  putLineUART(buffer);
  if(cardinfo.SD_csd.PermWrProtect==0) {
    sprintf(buffer,"no)");    
  }
  else {
    sprintf(buffer,"yes)");
  }
  putLineUART(buffer);
  sprintf(buffer,"\r\nTempWrProtect:%d(",cardinfo.SD_csd.TempWrProtect);
  putLineUART(buffer);
  if(cardinfo.SD_csd.TempWrProtect==0) {
    sprintf(buffer,"no)");    
  }
  else {
    sprintf(buffer,"yes)");
  }
  putLineUART(buffer);
  sprintf(buffer,"\r\nFileFormat:%d",cardinfo.SD_csd.FileFormat);
  putLineUART(buffer);
  
  putLineUART("\r\n\r\n------SPI READ SPEED TEST----------");
  
  t1=DWT_Get();
  for(i=0;i<1000;i+=1) {
    sd_read_block(i,bf);
  }
  t2=DWT_Get();
  
  sprintf(buffer,"\r\nRead speed - Total:%d bytes, Time:%d ms, Avg:%d kbytes/sec\r\n",
     SD_BLOCKSIZE*1000,(t2-t1)/(SystemCoreClock/1000),((SD_BLOCKSIZE*1000))/((t2-t1)/(SystemCoreClock/1000)));
  putLineUART(buffer);
	
}
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

	/* UART output thread, simply counts seconds */
	xTaskCreate(vUARTTask, "vTaskUart",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);
	
	
	xTaskCreate(SD_cardTask, "vSDCARDTask",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);


	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}
