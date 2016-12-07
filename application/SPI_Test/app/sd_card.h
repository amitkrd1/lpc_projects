/**************************************************************************//**
 * @file     sd.h
 * @brief    Header file for sd.c
 * @version  1.0
 * @date     18. Nov. 2010
 *
 * @note
 * Copyright (C) 2010 NXP Semiconductors(NXP), ChaN. All rights reserved.
 *
 * General SD driver (SD_xxxx()): NXP
 * SD card initilization flow and some code snippets are inspired from ChaN.
 *
 ******************************************************************************/
 
#include "stdint.h" 
#include "board.h"


#ifndef __SD_CARD_H
#define __SD_CARD_H


/* type defintion */
typedef unsigned char    SD_BOOL;
#define SD_TRUE     1
#define SD_FALSE    0

#ifndef NULL
 #ifdef __cplusplus              // EC++
  #define NULL          0
 #else
  #define NULL          ((void *) 0)
 #endif
#endif

#define SPI_CLOCKRATE_LOW   (uint32_t) (250)   /* 100MHz / 250 = 400kHz */
#define SPI_CLOCKRATE_HIGH  (uint32_t) (4)     /* 100MHz / 4 = 25MHz */
	
/* Memory card type definitions */
#define CARDTYPE_UNKNOWN        0
#define CARDTYPE_MMC            1   /* MMC */
#define CARDTYPE_SDV1           2   /* V1.x Standard Capacity SD card */
#define CARDTYPE_SDV2_SC        3   /* V2.0 or later Standard Capacity SD card */
#define CARDTYPE_SDV2_HC        4   /* V2.0 or later High/eXtended Capacity SD card */

/* SD/MMC card configuration */
typedef struct tagCARDCONFIG
{
    uint32_t sectorsize;    /* size (in byte) of each sector, fixed to 512bytes */
    uint32_t sectorcnt;     /* total sector number */  
    uint32_t blocksize;     /* erase block size in unit of sector */     
	uint8_t  ocr[4];		/* OCR */
	uint8_t  cid[16];		/* CID */
	uint8_t  csd[16];		/* CSD */
} CARDCONFIG;

/* Public variables */
extern uint8_t CardType;
extern CARDCONFIG CardConfig;


/* Public functions */
SD_BOOL     SD_Init (void);
SD_BOOL     SD_ReadSector (uint32_t sect, uint8_t *buf, uint32_t cnt);
SD_BOOL     SD_WriteSector (uint32_t sect, const uint8_t *buf, uint32_t cnt);
SD_BOOL     SD_ReadConfiguration (void);
void        disk_timerproc (void);

static void SPI_CS_High (void)
  {
    /* SSEL is GPIO, set to high.  */
    //LPC_GPIO0->FIOPIN |= (1 << 16);
	  Chip_GPIO_Init(LPC_GPIO);
	//Chip_GPIO_WritePortBit(LPC_GPIO,0,27,true);
		Chip_GPIO_SetPinDIROutput(LPC_GPIO,0,27);

	
 }
	
 static uint32_t SPI_RecvByte()
 {
	 Chip_SPI_SendMidFrame(LPC_SPI0,0xFF);
 }
	 
	 

#endif // __SD_CARD_H

/* --------------------------------- End Of File ------------------------------ */


 


