/*
===============================================================================
 Name        : LPC15_CAN_Sample.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

/* History:
 *
 * 23.04.2015 V1.0 	new: Hardware: Target64-Board
 */

#include "board.h"
//#include <cr_section_macros.h>
#include "can.h"

#define TICKRATE_HZ (10)				//SysTick ticks per second

static volatile uint32_t ticks;			//tick counter
static volatile uint8_t	 t_flag;		//timer flag

void SysTick_Handler(void)
{
 ticks++;								//inc ticks
 t_flag =1;								//set flag
}

int main(void)							//main
{
 SystemCoreClockUpdate();
 Board_Init();
 Board_LED_Set(0, true);
 can_init(100);							//CAN setup 100kbaud
 SysTick_Config(SystemCoreClock/TICKRATE_HZ);//set SysTick

// //fill message object 1
 msg1_obj.msgobj  = 6;
 msg1_obj.mode_id = 0x700 + NODE_ID;
 msg1_obj.mask    = 0x0;
 msg1_obj.dlc     = 8;
 msg1_obj.data[0] = 'T';   msg1_obj.data[1] = 'E';   msg1_obj.data[2] = 'S';    msg1_obj.data[3] = 'T';
 msg1_obj.data[4] =  0xAA; msg1_obj.data[5] =  0xAA; msg1_obj.data[6] =  0xAA;  msg1_obj.data[7] =  0xAA;

// //fill message object 1 with 0 to create a very long message (bit stuffing)
//  msg1_obj.msgobj  = 6;
//  msg1_obj.mode_id = 0x000; msg1_obj.mask    = 0x00;
//  msg1_obj.dlc     = 8;
//  msg1_obj.data[0] = 0; msg1_obj.data[1] = 0; msg1_obj.data[2] = 0; msg1_obj.data[3] = 0;
//  msg1_obj.data[4] = 0; msg1_obj.data[5] = 0; msg1_obj.data[6] = 0; msg1_obj.data[7] = 0;

 //fill message object 2
 msg2_obj.msgobj  = 8;
 msg2_obj.mode_id = 0x700 + NODE_ID +1;
 msg2_obj.mask    = 0x0;
 msg2_obj.dlc     = 4;
 msg2_obj.data[0] = 0x11; msg2_obj.data[1] = 0x22; msg2_obj.data[2] = 0x33; msg2_obj.data[3] = 0x44;

 while(1)								//endless loop
 {
  uint32_t f_ret;						//function return
  if(message_received)					//message received?
  {
   message_received =0;					//reset message received flag
   msg2_obj.data[3]++;					//inc counter
  } 	 								//end message received
  if(t_flag)							//timer flag?
  {
   t_flag =0;							//reset flag
   f_ret=can_transmit(&msg1_obj);		//transmit object 1

   f_ret=can_transmit(&msg2_obj);		//transmit object 2

   Board_LED_Toggle(0);					//toggle LED 0
  }										//end timer flag
  //CAN BUS OFF---------------------------------------------------------------
  if(LPC_CAN->STAT &  STAT_BOFF)		//bus off?
  {
   can_init(act_kbaud);					//re-init CAN
  }										//end bus off
 }										//end endless loop
}										//end main

