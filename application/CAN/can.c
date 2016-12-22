/*
 * can.c
 *
 *  Created on: 20.12.2016
 *      Author:
 */

#include "board.h"
#include "can.h"

/* Memory for CAN API */
#define MAX_CAN_PARAM_SIZE          512
static uint32_t gCANapiMem[MAX_CAN_PARAM_SIZE];

/* CAN handle */
static CAN_HANDLE_T pCanHandle;

/* CAN initialization structure / message structure */
static CAN_CFG gCANConfig;
CAN_MSG1_OBJ msg1_obj,msg2_obj;			//2 message objects
CAN_MSG1_OBJ rec_obj;					//received message object

volatile uint8_t message_received; 		//message received flag
static volatile uint8_t tx_busy;		//busy flag
uint16_t act_kbaud;						//actual kbaudrate for busoff re-initialize CAN

/* Callback function prototypes */
static void CAN_rx(uint8_t msg_obj_num);

static void CAN_tx(uint8_t msg_obj_num);

static void CAN_error(uint32_t error_info);

/* Publish CAN Callback Functions */
CAN_CALLBACKS callbacks = {
	CAN_rx,
	CAN_tx,
	CAN_error
};

/*****************************************************************************
 * Private functions
 ****************************************************************************/
static void CAN_rx(uint8_t msgNumber)
{
 Board_LED_Toggle(1);					//toggle LED 1
	/* Determine which CAN message has been received */
	rec_obj.msgobj = msgNumber;

	/* Now load up the rec_obj structure with the CAN message */
	LPC_CAND_API->hwCAN_MsgReceive(pCanHandle, (CAN_MSG_OBJ*)&rec_obj);
#ifndef LOOPBACK_MODE					//don't transmit in LOOPBACK mode
	if (msgNumber == 1) {
		/* Simply transmit CAN frame (echo) with with ID +0x100 via buffer 4 */
		rec_obj.msgobj = 4;
		rec_obj.mode_id += 0x100;
		LPC_CAND_API->hwCAN_MsgTransmit(pCanHandle, (CAN_MSG_OBJ*)&rec_obj);
	}
#endif
 message_received =1;					//set message received flag
}

static void CAN_tx(uint8_t msg_obj)
{
 Board_LED_Toggle(2);					//toggle LED 2
 tx_busy = 0;							//rest busy flag
}

CAN_ERROR_t can_err;

static void CAN_error(uint32_t error_info)
{
 if(error_info & CAN_ERROR_PASS)		//passive error
 {
  can_err.Pass_Cnt++;
 }										//end passive error
 if(error_info & CAN_ERROR_WARN)		//warning error
 {
  can_err.WARN_Cnt++;
 }										//end warning error
 if(error_info & CAN_ERROR_BOFF)		//busoff error
 {
  can_err.BOff_Cnt++;
 }										//end busoff error
 if(error_info & CAN_ERROR_STUF)		//stuf error
 {
  can_err.STUF_Cnt++;
 }										//end stuf error
 if(error_info & CAN_ERROR_FORM)		//form error
 {
  can_err.FORM_Cnt++;
 }										//end form error
 if(error_info & CAN_ERROR_ACK)			//ack error
 {
  can_err.ACK_Cnt++;
 }										//end ack error
 if(error_info & CAN_ERROR_BIT1)		//bit1 error
 {
  can_err.BIT1_Cnt++;
	 
	 
	 
	 
	 
	 
 }										//end bit1 error
 if(error_info & CAN_ERROR_BIT0)		//bit0 error
 {
  can_err.BIT0_Cnt++;
 }										//end bit0 error
 if(error_info & CAN_ERROR_CRC)			//crc error
 {
  can_err.CRC_Cnt++;
 }										//end crc error
}

void CAN_IRQHandler(void)
{
 LPC_CAND_API->hwCAN_Isr(pCanHandle);
}

void can_speed(uint16_t kbaud)
{
 uint8_t selection;
 switch(kbaud)
 {
  case  100://Initialize CAN Controller 72MHz-100kbps
	 	    selection = 0;
	 	    break;
  case  125://Initialize CAN Controller 72MHz-125kbps
	  	    selection = 1;
	 	    break;
  case  250://Initialize CAN Controller 72MHz-250kbps
  	    	selection = 2;
	  	    break;
  case  500://Initialize CAN Controller 72MHz-500kbps
   	    	selection = 3;
	 	    break;
  case 1000://Initialize CAN Controller 72MHz-1000kbps
  	    	selection = 4;
	 	    break;
  default:  //Initialize CAN Controller 72MHz-100kbps
	        selection = 0;
	  	    break;
 }
//set selected speed
 gCANConfig.clkdiv = c_speed[selection].div;
 gCANConfig.btr    = c_speed[selection].btr;
 //store actual kbaud
 act_kbaud = kbaud;
}


void can_init(uint16_t kbaud)
{
 uint32_t maxParamSize;
 uint32_t status;
 CAN_API_INIT_PARAM_T myCANConfig = {0,LPC_C_CAN0_BASE,&gCANConfig,&callbacks,NULL,NULL};
 /* Enable clocking for CAN and reset the controller */
 Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_CAN);
 Chip_SYSCTL_PeriphReset(RESET_CAN);
#ifdef LQFP48
//CAN signal muxing LQFP48
 Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 18, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
 Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 13, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
 Chip_SWM_MovablePortPinAssign(SWM_CAN_TD1_O , 0, 18);
 Chip_SWM_MovablePortPinAssign(SWM_CAN_RD1_I,  0, 13);
#endif
#ifdef LQFP64
 /* Assign the pins rx 0[11] and tx 0[31] @ LQFP64 */
 Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 11, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
 Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 31, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
 Chip_SWM_MovablePinAssign(SWM_CAN_RD1_I, 11);	/* Rx P0.11 */
 Chip_SWM_MovablePinAssign(SWM_CAN_TD1_O, 31);	/* Tx P0.31 */
#endif

 myCANConfig.mem_base = (uint32_t) &gCANapiMem[0];
 gCANConfig.clkdiv = 0;		/* Target divisor = 1, clkdiv = (target-1) i.e 0 */

 can_speed(kbaud);						//set speed in kbps
 gCANConfig.isr_ena = 1;				//enable interrupts

 /* Validate that we reserved enough memory */
 maxParamSize = LPC_CAND_API->hwCAN_GetMemSize(&myCANConfig);
 if (maxParamSize > MAX_CAN_PARAM_SIZE / 4) { while ( 1 ) {} }

 /* Initialize the ROM with specific configuration */
 status = LPC_CAND_API->hwCAN_Init(&pCanHandle, &myCANConfig);
 if (status != CAN_ERROR_NONE) { while (1) { __WFI();} }

 //change CAN DAR / LOOPBACK if necessary
 LPC_CAN->CNTL &= ~CTRL_CCE;			//setup
#ifdef DAR_MODE							//DAR?
 LPC_CAN->CNTL |= (CTRL_DAR);			//disable retransmission
#endif
#ifdef LOOPBACK_MODE					//LOOPBACK
 LPC_CAN->CNTL |= (CTRL_TEST);			//enable test
 LPC_CAN->TEST |= (TEST_LBACK);			//enable loopback
#endif
 LPC_CAN->CNTL &= ~CTRL_INIT;			//normal operation
 if (gCANConfig.isr_ena == TRUE) { NVIC_EnableIRQ(CAN_IRQn); }

 /* Configure message object 1 to receive all 11-bit messages 0x400-0x4FF */
 msg1_obj.msgobj = 1;
 msg1_obj.mode_id = 0x400;
 msg1_obj.mask = 0x700;
 LPC_CAND_API->hwCAN_ConfigRxmsgobj(pCanHandle, (CAN_MSG_OBJ*)&msg1_obj);
}

uint32_t can_transmit(CAN_MSG1_OBJ* message_object)
{
 uint32_t transmit_timeout= 100E3;
 //check object number
 if((message_object->msgobj <1) || (message_object->msgobj >20))
 {
  return 1;								//wrong message object
 }
 //read pending transmission request of object
 while(LPC_CAN->TXREQ1 & (1<<message_object->msgobj)){}
 //wait if busy
 while(tx_busy)
 {
  transmit_timeout--;
  if(!transmit_timeout)break;
 }                            
 if(!transmit_timeout)
 {
  tx_busy =0;							//reset busy
  return 2;								//timeout return
 }
 //transmit message
 tx_busy =1;							//set busy flag
 LPC_CAND_API->hwCAN_MsgTransmit(pCanHandle, (CAN_MSG_OBJ*)message_object);
 return 0;								//return success
}

