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
 #include "bmp180.h"





	/*****************************************************************************
	* Private types/enumerations/variables
	****************************************************************************/




	/*****************************************************************************
	* Public types/enumerations/variables
	****************************************************************************/

	#define I2C_CLK_DIVIDER     (180)

	/* 100KHz I2C bit-rate - going too fast may prevent the salev from responding
	 in time */
	#define I2C_BITRATE         (100000)
	/* Standard I2C mode */
	#define I2C_MODE    (0)
	#define TICKRATE_HZ         (10)
	static volatile int state;
	#define BMP180_1_16     ((float) 0.0625)
	#define BMP180_1_256    ((float) 0.00390625)
	#define BMP180_1_2048   ((float) 0.00048828125)
	#define BMP180_1_4096   ((float) 0.000244140625)
	#define BMP180_1_8192   ((float) 0.0001220703125)
	#define BMP180_1_32768  ((float) 0.000030517578125)
	#define BMP180_1_65536  ((float) 0.0000152587890625)
	#define BMP180_1_101325 ((float) 0.00000986923266726)

	#define BMP_SLV_ADD  (0x77)


	static I2CM_XFER_T  i2cmXferRec;
	
	

	/* EEPROM values */
	int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
	uint16_t AC4, AC5, AC6, UT;
	/* OK */
	int32_t X1, X2, X3, B3, B5, B6, T, p;
	uint32_t B4, B7, UP;
	uint8_t lib_initialized = 0;
	


	/*****************************************************************************
	* Private functions
	****************************************************************************/


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

	static void Init_I2C_PinMux(void)
	{
	#if defined(BOARD_NXP_LPCXPRESSO_1549)
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 22, IOCON_DIGMODE_EN | I2C_MODE);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 23, IOCON_DIGMODE_EN | I2C_MODE);
	Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SCL);
	Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SDA);
	#else
	/* Configure your own I2C pin muxing here if needed */
	#error "No I2C Pin Muxing defined for this example"
	#endif
	}

	//void SysTick_Handler(void)
	//{
	//	static int ticks = 0;

	//	ticks++;
	//	if (ticks > TICKRATE_HZ) {
	//		ticks = 0;
	//		state = 1 - state;
	//	}
	//}


	static void SetupXferRecAndExecute(uint8_t devAddr,
									 uint8_t *txBuffPtr,
									 uint16_t txSize,
									 void *rxBuffPtr,
									 uint16_t rxSize)
	{
	/* Setup I2C transfer record */
	i2cmXferRec.slaveAddr = devAddr;
	i2cmXferRec.status = 0;
	i2cmXferRec.txSz = txSize;
	i2cmXferRec.rxSz = rxSize;
	i2cmXferRec.txBuff = txBuffPtr;
	i2cmXferRec.rxBuff = rxBuffPtr;

	Chip_I2CM_XferBlocking(LPC_I2C0, &i2cmXferRec);
	}


	void BMPInit(void) {

		
		uint8_t slaveadd;
	uint8_t data[22];
	uint8_t BMP180TempRegisterAddress = 0;
		uint8_t i = 0;
  bool slavecheck;
	/* Read LM75 temperature sensor */
	SetupXferRecAndExecute(

		/* The LM75 I2C bus address */
		BMP_SLV_ADD, 

		/* Transmit one byte, the LM75 temp register address */
		&BMP180TempRegisterAddress, 1, 

		/* Receive back two bytes, the contents of the temperature register */
		data, 22);

		AC1 = (int16_t)(data[i] << 8 | data[i + 1]); i += 2;
	AC2 = (int16_t)(data[i] << 8 | data[i + 1]); i += 2;
	AC3 = (int16_t)(data[i] << 8 | data[i + 1]); i += 2;
	AC4 = (uint16_t)(data[i] << 8 | data[i + 1]); i += 2;
	AC5 = (uint16_t)(data[i] << 8 | data[i + 1]); i += 2;
	AC6 = (uint16_t)(data[i] << 8 | data[i + 1]); i += 2;
	B1 = (int16_t)(data[i] << 8 | data[i + 1]); i += 2;
	B2 = (int16_t)(data[i] << 8 | data[i + 1]); i += 2;
	MB = (int16_t)(data[i] << 8 | data[i + 1]); i += 2;
	MC = (int16_t)(data[i] << 8 | data[i + 1]); i += 2;
	MD = (int16_t)(data[i] << 8 | data[i + 1]);
// slavecheck=Chip_I2CS_IsSlaveSelected(LPC_I2C0);
// printf("%d \r\n",slavecheck);
	/* Test for valid operation */
	if (i2cmXferRec.status == I2CM_STATUS_OK) 
		/* Output temperature. */
		{
			
		printf("Calibration CO: %d, %d,%d,%d  \r\n",(uint8_t )AC1,AC2,AC3,MD);

	}

	}

	void ReadTemp(TM_BMP180_t* BMP180_Data)
		
	{
		int i=0;
		int temp;
		uint8_t tempdata[2];
		uint8_t value;
		
		uint8_t BMP180_REGISTER_CONTROL=0xF4;
		uint8_t BMP180_REGISTER_RESULT=0xF6;
		
			Chip_I2CM_WriteByte(LPC_I2C0,0xF4);
	    Chip_I2CM_WriteByte(LPC_I2C0,0x2E);

	BMP180_Data->Delay = BMP180_TEMPERATURE_DELAY;
		
		SetupXferRecAndExecute(

		/* The BMP180 I2C bus address */
		BMP_SLV_ADD, 

		/* Transmit one byte, the BM180 temp register address */
		&BMP180_REGISTER_RESULT, 1, 

		/* Receive back two bytes, the contents of the temperature register */
		tempdata, 2);

		/* Get uncompensated temperature */
	UT =tempdata[0] << 8 | tempdata[1];
		printf(" UnT: %d \r \n",UT);

	/* Calculate true temperature */
	//	X1 = (UT - AC6) * AC5 * 0x8000;
	X1 = (UT - AC6) * AC5 * BMP180_1_32768;
	X2 = MC * 2048 / (X1 + MD);
	B5 = X1 + X2;

	/* Get temperature in degrees */
	temp = (B5 + 8) / ((float)160);
	printf("AT: %d \r\n",temp);
	}
  
void readPressure(TM_BMP180_t* BMP180_Data, TM_BMP180_Oversampling_t Oversampling)
	{
		uint8_t command;
	  uint8_t pressureValue[3];
		uint8_t BMP180_REGISTER_RESULT=0xF6;
		
		
		
		switch (Oversampling) {
		case TM_BMP180_Oversampling_UltraLowPower :
			command = BMP180_COMMAND_PRESSURE_0;
			BMP180_Data->Delay = BMP180_PRESSURE_0_DELAY;
			break;
		case TM_BMP180_Oversampling_Standard:
			command = BMP180_COMMAND_PRESSURE_1;
			BMP180_Data->Delay = BMP180_PRESSURE_1_DELAY;
			break;
		case TM_BMP180_Oversampling_HighResolution:
			command = BMP180_COMMAND_PRESSURE_2;
			BMP180_Data->Delay = BMP180_PRESSURE_2_DELAY;
			break;
		case TM_BMP180_Oversampling_UltraHighResolution:
			command = BMP180_COMMAND_PRESSURE_3;
			BMP180_Data->Delay = BMP180_PRESSURE_3_DELAY;
			break;
		default:
			command = BMP180_COMMAND_PRESSURE_0;
			BMP180_Data->Delay = BMP180_PRESSURE_0_DELAY;
			break;
	}
    Chip_I2CM_WriteByte(LPC_I2C0,0xF4);
	  Chip_I2CM_WriteByte(LPC_I2C0,0x34);
		
	SetupXferRecAndExecute(

		/* The BMP180 I2C bus address */
		BMP_SLV_ADD, 

		/* Transmit two byte, the BM180 data register address */
		&BMP180_REGISTER_RESULT, 2, 

		/* Receive back two bytes, the contents of the pressure value */
		pressureValue, 3);
	BMP180_Data->Oversampling = Oversampling;
	/* Get uncompensated pressure */
	UP = (pressureValue[0] << 16 | pressureValue[1] << 8 | pressureValue[2]) >> (8 - (uint8_t)BMP180_Data->Oversampling);
	/* Calculate true pressure */
	
	printf(" UP: %d \r\n",UP);
	B6 = B5 - 4000;
	X1 = (B2 * (B6 * B6 * BMP180_1_4096)) * BMP180_1_2048;
	X2 = AC2 * B6 * BMP180_1_2048;
	X3 = X1 + X2;
	B3 = (((AC1 * 4 + X3) << (uint8_t)BMP180_Data->Oversampling) + 2) * 0.25;
	X1 = AC3 * B6 * BMP180_1_8192;
	X2 = (B1 * (B6 * B6 * BMP180_1_4096)) * BMP180_1_65536;
	X3 = ((X1 + X2) + 2) * 0.25;
	B4 = AC4 * (uint32_t)(X3 + 32768) * BMP180_1_32768;
	B7 = ((uint32_t)UP - B3) * (50000 >> (uint8_t)BMP180_Data->Oversampling);
	if (B7 < 0x80000000) {
		p = (B7 * 2) / B4;
	} else {
		p = (B7 / B4) * 2;
	}
	X1 = ((float)p * BMP180_1_256) * ((float)p * BMP180_1_256);
	X1 = (X1 * 3038) * BMP180_1_65536;
	X2 = (-7357 * p) * BMP180_1_65536;
	p = p + (X1 + X2 + 3791) * BMP180_1_16;
	
	//printf("B7: %d \r\n",B7);
	printf("%d \r\n",p);
	
	/* Save pressure */
	BMP180_Data->Pressure = p;
	
	/* Calculate altitude */
	BMP180_Data->Altitude = (float)44330.0 * (float)((float)1.0 - (float)pow((float)p * BMP180_1_101325, 0.19029495));
	
	
	//printf("Altitude %f \r\n",BMP180_Data->Altitude);
	
	
}
	
	static void setupI2CMaster(void)
	{
	/* Enable I2C clock and reset I2C peripheral */
	Chip_I2C_Init(LPC_I2C0);

	/* Setup clock rate for I2C */
	Chip_I2C_SetClockDiv(LPC_I2C0, I2C_CLK_DIVIDER);

	/* Setup I2CM transfer rate */
	Chip_I2CM_SetBusSpeed(LPC_I2C0, I2C_BITRATE);

	/* Enable I2C master interface */
	Chip_I2CM_Enable(LPC_I2C0);
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

	static void i2c_task(void *pvParameters)
	{
 TM_BMP180_t BMP180_Data;

	//	int lastState = 0;
	//	state = 0;

	/* Generic Initialization */
	SystemCoreClockUpdate();
	Board_Init();
	 pinmux();
	uart_init();

	/* Clear activity LED */
	Board_LED_Set(0, false);

	/* Setup I2C pin muxing */
	Init_I2C_PinMux();

	/* Allocate I2C handle, setup I2C rate, and initialize I2C
		 clocking */
	setupI2CMaster();

	/* Disable the interrupt for the I2C */
	NVIC_DisableIRQ(I2C0_IRQn);

	/* Enable SysTick Timer */
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);

	/* Loop forever, toggle LED on board */
	while (1) {
	 BMPInit();
	 ReadTemp(&BMP180_Data);
		readPressure(&BMP180_Data, TM_BMP180_Oversampling_UltraHighResolution);
		//vTaskDelay(configTICK_RATE_HZ);
		/* Sleep until a state change occurs in SysTick */
			__WFI();
		}
	


		/* Read Motor Control board's I2C temperature sensor and output result */
		
		



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


	xTaskCreate(i2c_task, "vi2cTask",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);
	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
	}
