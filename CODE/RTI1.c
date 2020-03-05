/** ###################################################################
**     THIS COMPONENT MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename  : RTI1.c
**     Project   : DiagnosticDemo
**     Processor : MC9S12G128VLH
**     Component : Init_RTI
**     Version   : Component 01.097, Driver 02.02, CPU db: 3.00.017
**     Compiler  : CodeWarrior HC12 C Compiler
**     Date/Time : 2016/10/13, 13:48
**     Abstract  :
**          This file implements the RTI (RTIfree)
**          module initialization according to the Peripheral Initialization
**          Component settings, and defines interrupt service routines prototypes.
**          The RTI can be used to generate a hardware interrupt
**          at a fixed periodic rate. If enabled (by setting RTIE=1),
**          this interrupt will occur at the rate selected by the RTICTL
**          register.The RTI runs with a gated OSCCLK ).At the end of the
**          RTI time-out period the RTIF flag is set to one and a new RTI
**          time-out period starts immediately.
**     Settings  :
**          Component name                                 : RTI1
**          Device                                         : RTI
**          Settings                                       : 
**            Clock settings                               : 
**              Clock source                               : IRCCLK
**              Prescaler                                  : 10^3
**              Modulus                                    : 1
**              Divider                                    : Decimal
**              Period                                     : 1 ms
**            RTI enable in Pseudo Stop Mode               : no
**          Interrupts                                     : 
**            RTI Interrupt                                : 
**              RTI Interrupt                              : Enabled
**              Interrupt                                  : Vrti
**              Priority                                   : 1
**              ISR name                                   : RTI_Interrupt
**          Initialization                                 : 
**            Call Init method                             : yes
**     Contents  :
**         Init - void RTI1_Init(void);
**
**     Copyright : 1997 - 2011 Freescale Semiconductor, Inc. All Rights Reserved.
**     
**     http      : www.freescale.com
**     mail      : support@freescale.com
** ###################################################################*/

/* MODULE RTI1. */

#include "RTI1.h"

/*
** ###################################################################
**
**  The interrupt service routine(s) must be implemented
**  by user in one of the following user modules.
**
**  If the "Generate ISR" option is enabled, Processor Expert generates
**  ISR templates in the CPU event module.
**
**  User modules:
**      DiagnosticDemo.c
**      Events.c
**
** ###################################################################
#pragma CODE_SEG __NEAR_SEG NON_BANKED
ISR(RTI_Interrupt)
        {
        // NOTE: The routine should include the following actions to obtain
        //       correct functionality of the hardware.
        //
        //      The ISR is invoked by RTIF flag. The RTIF flag is cleared
        //      if a "1" is written to the flag in CPMUFLG register.
        //      Example: CPMUFLG = 128;
        }
#pragma CODE_SEG DEFAULT
*/
/*
** ===================================================================
**     Method      :  RTI1_Init (component Init_RTI)
**
**     Description :
**         This method initializes registers of the RTI module
**         according to this Peripheral Initialization settings. Call
**         this method in user code to initialize the module. By
**         default, the method is called by PE automatically; see "Call
**         Init method" property of the component for more details.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void RTI1_Init(void)
{
  /* CPMUINT: RTIE=0 */
  clrReg8Bits(CPMUINT, 0x80U);          
  /* CPMUFLG: RTIF=1,PORF=0,LVRF=0,LOCKIF=0,LOCK=0,ILAF=0,OSCIF=0,UPOSC=0 */
  setReg8(CPMUFLG, 0x80U);              
  /* CPMUPROT: ??=0,??=0,??=1,??=0,??=0,??=1,??=1,PROT=0 */
  setReg8(CPMUPROT, 0x26U);            /* Disable protection of clock-source register */ 
  /* CPMUCLKS: PRE=0,RTIOSCSEL=0 */
  clrReg8Bits(CPMUCLKS, 0x0AU);         
  /* CPMUPROT: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,PROT=0 */
  setReg8(CPMUPROT, 0x00U);            /* Re-Enable protection of clock-source register */ 
  /* CPMUINT: RTIE=1 */
  setReg8Bits(CPMUINT, 0x80U);          
  /* CPMURTI: RTDEC=1,RTR6=0,RTR5=0,RTR4=0,RTR3=0,RTR2=0,RTR1=0,RTR0=0 */
  setReg8(CPMURTI, 0x80U);              
}

/* END RTI1. */

/*
** ###################################################################
**
**     This file was created by Processor Expert 3.05 [04.46]
**     for the Freescale HCS12 series of microcontrollers.
**
** ###################################################################
*/