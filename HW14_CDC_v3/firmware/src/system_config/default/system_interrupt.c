/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system/common/sys_common.h"
#include "app.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

 
void __ISR(_USB_1_VECTOR, ipl4AUTO) _IntHandlerUSBInstance0(void)
{
    DRV_USBFS_Tasks_ISR(sysObj.drvUSBObject);
}

void __ISR(_TIMER_4_VECTOR, IPL4SOFT) Timer4ISR(void) {
  // code for PI control goes here
    //static int counter = 0;
    //static int plotind = 0;
    //static int decctr = 0;
    //static float u = 0;898
    //static float unew = 0;
    
    // Set two wheels to same speed by P control
    /*
    float v5_desire = 200.0/60.0;
    float v3_desire = 200.0/60.0;
    float pwm = 0;
    int kp = 50;
    float error = 0;
    
    
    float v5_actual = TMR5/700.0*50.0;
    float v3_actual = TMR3/700.0*50.0;
    
    float error1 = v5_desire-v5_actual;
    
    float pwm1 = error1*kp;
    
    if(pwm1 > 2400){
        OC1RS = 2000;
    }else if(pwm1 <0){
        OC1RS = 0;
    }else{
        ;
    }
    
    
    float error2 = v3_desire-v3_actual;
    float pwm2 = error2*kp;
    
    if(pwm1 > 2400){
        OC4RS = 2000;
    }else if(pwm1 <0){
        OC4RS = 0;
    }else{
        ;
    }
    
    */
    
    // Set two wheels to left or right
   
    int kp = 50;
    int MAX_DUTY=600;
    float left=0;
    float right =0;
    
    float error = rxVal - 320; // 240 means the dot is in the middle of the screen
        
        if (error < 0 ) { // slow down the left motor to steer to the left
            error  = -error;
            left = MAX_DUTY - kp*error;
                right = MAX_DUTY;
                
                if (left < 0){
                    left = 0;
                    }
                error = -error;
                }
    

        else { // slow down the right motor to steer to the right
            right = MAX_DUTY - kp*error;
            left = MAX_DUTY+500;
            if (right<0) {
                right = 0;
            }
        }
    
    if (error < -140){
        right = 800;
        left = 0;
        
    }
        if (error > 140){
        left = 1200;
        right = 0;
        
    }

    /*
        if (error < -100 ) { // slow down the left motor to steer to the left
            error  = -error;
            left = MAX_DUTY - kp*error;
                right = MAX_DUTY;
                if (left < 0){
                    left = 0;
                    }
                }
            
        else if(error>= -100 ||error< 0){
            error  = -error;
            left = MAX_DUTY - kp*error;
                right = MAX_DUTY/2;
                if (left < 0){
                    left = 0;
                    }
        }
        else if(error>= 0 ||error< 100){
            right = MAX_DUTY - kp*error;
            left = MAX_DUTY/2;
            if (right<0) {
                right = 0;
            }
        }

        else { // slow down the right motor to steer to the right
            right = MAX_DUTY - kp*error;
            left = MAX_DUTY;
            if (right<0) {
                right = 0;
            }
        }
    */
        OC4RS = left;
        OC1RS = right; //OC1RS = left wheel.
    
    TMR5=0;
    TMR3=0;
    
    
  IFS0bits.T4IF = 0; // clear interrupt flag, last line
  
  
}


/*******************************************************************************
 End of File
*/
