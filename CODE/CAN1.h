/** ###################################################################
**     THIS COMPONENT MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename  : CAN1.h
**     Project   : DiagnosticDemo
**     Processor : MC9S12G128VLH
**     Component : Init_MSCAN
**     Version   : Component 01.067, Driver 01.11, CPU db: 3.00.017
**     Compiler  : CodeWarrior HC12 C Compiler
**     Date/Time : 2016/10/13, 13:48
**     Abstract  :
**          This file implements the MSCAN (MSCAN) module initialization
**          according to the Peripheral Initialization Bean settings,
**          and defines interrupt service routines prototypes.
**          The Motorola Scalable Controller Area Network (MSCAN) definition
**          is based on the MSCAN12 definition which is the specific
**          implementation of the Motorola Scalable CAN concept targeted
**          for the Freescale MC68HC12 Microcontroller Family.
**
**          The basic features of the MSCAN are as follows:
**          - Implementation of the CAN protocol - Version 2.0A/B
**           Standard and extended data frames
**           0 - 8 bytes data length
**           Programmable bit rate up to 1 Mbps1
**           Support for remote frames
**           5 receive buffers with FIFO storage scheme
**          - 3 transmit buffers with internal prioritization using a local
**          priority concept
**          - Flexible maskable identifier filter supports two full size
**          extended identifier filters (two 32-bit) or four 16-bit filters
**          or eight 8-bit filters
**          - Programmable wake-up functionality with integrated low-pass
**          filter
**          - Programmable loop back mode supports self-test operation
**          - Programmable listen-only mode for monitoring of CAN bus
**          - Separate signalling and interrupt capabilities for all CAN
**          receiver and transmitter error states
**          (Warning, Error Passive, Bus-Off)
**          - Programmable MSCAN clock source either Bus Clock or Oscillator
**          Clock
**          - Internal timer for time-stamping of received and transmitted
**          messages
**          - Three low power modes: Sleep, Power Down and MSCAN Enable
**          - Global initialization of configuration registers
**     Settings  :
**          Bean name                                      : CAN1
**          Device                                         : MSCAN
**          Clock Source                                   : Bus Clock
**          Baud Rate Prescaler                            : 2
**          Synchr. Jump Width                             : 1
**          Sampling                                       : One sample per bit
**          Time Segment 1                                 : 13
**          Time Segment 2                                 : 2
**          CAN frequency                                  : 8 MHz
**          Time quantum                                   : 125 ns
**          Bit rate                                       : 500 kbit/s
**          CAN Stops in Wait Mode                         : no
**          Wake-Up Mode                                   : None
**          Loop Back Test Mode                            : Disabled
**          Listen Only Mode                               : Normal operation
**          Sleep Mode Request                             : Disabled
**          Time Stamp                                     : Disabled
**          Acceptance mode                                : Two 32 bit Acceptance Filters
**          Rx acceptance ID(1st bank)                     : 0
**          Rx acceptance ID(2nd bank)                     : 0
**          Rx acceptance ID mask (1st bank)               : FFFFFFFF
**          Rx acceptance ID mask (2nd bank)               : FFFFFFFF
**          RXCAN pin                                      : PM0_RXCAN
**          RXCAN pin signal                               : 
**          TXCAN pin                                      : PM1_TXCAN
**          TXCAN pin signal                               : 
**          Wake up                                        : 
**          Wake up                                        : Disabled
**          Interrupt                                      : Vcanwkup
**          Wake interrupt priority                        : 1
**          ISR name                                       : 
**          Error                                          : 
**          Error Interrupt                                : Vcanerr
**          Status Change Interrupt                        : Disabled
**          Receiver Status Change                         : do not generate
**          Transmitt. Status Change                       : do not generate
**          Overrun Interrupt                              : Disabled
**          Error interrupt priority                       : 1
**          ISR name                                       : 
**          Receiver Full                                  : 
**          Receiver Full                                  : Enabled
**          Receiver Interrupt                             : Vcanrx
**          Rx interrupt priority                          : 1
**          ISR name                                       : Can_Rx_Interrupt
**          Transmitter empty                              : 
**          Transmitter Interrupt                          : Vcantx
**          Tx Empty Interrupt 0                           : Disabled
**          Tx Empty Interrupt 1                           : Disabled
**          Tx Empty Interrupt 2                           : Disabled
**          Tx interrupt priority                          : 1
**          ISR name                                       : 
**          Call Init in CPU init. code                    : yes
**          CAN Enable                                     : yes
**     Contents  :
**         Init - void CAN1_Init(void);
**
**     Copyright : 1997 - 2011 Freescale Semiconductor, Inc. All Rights Reserved.
**     
**     http      : www.freescale.com
**     mail      : support@freescale.com
** ###################################################################*/

#ifndef __CAN1
#define __CAN1

/* MODULE CAN1. */

/*Include shared modules, which are used for whole project*/
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
/* Include inherited components */

#include "Cpu.h"




void CAN1_Init(void);
/*
** ===================================================================
**     Method      :  CAN1_Init (component Init_MSCAN)
**
**     Description :
**         This method initializes registers of the CAN module
**         according to this Peripheral Initialization Bean settings.
**         Call this method in user code to initialize the module.
**         By default, the method is called by PE automatically; see
**         "Call Init method" property of the bean for more details.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
#pragma CODE_SEG __NEAR_SEG NON_BANKED
/*
** ===================================================================
** The interrupt service routine must be implemented by user in one
** of the user modules (see CAN1.c file for more information).
** ===================================================================
*/
__interrupt void Can_Rx_Interrupt(void);
#pragma CODE_SEG DEFAULT

/* END CAN1. */

#endif /* ifndef __CAN1 */
/*
** ###################################################################
**
**     This file was created by Processor Expert 3.05 [04.46]
**     for the Freescale HCS12 series of microcontrollers.
**
** ###################################################################
*/
