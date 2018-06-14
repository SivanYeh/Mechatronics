/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

#include "app.h"
#include <stdio.h>
#include <xc.h>
#include "i2c_master_noint.h"
#include "ST7735.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

uint8_t APP_MAKE_BUFFER_DMA_READY dataOut[APP_READ_BUFFER_SIZE];
uint8_t APP_MAKE_BUFFER_DMA_READY readBuffer[APP_READ_BUFFER_SIZE];
int len, i = 0;
int startTime = 0;

#define WHO_AM_I 0x0F       // Check the circuit
#define CTRL1_XL 0x10       // acceleration
#define CTRL2_G 0x11        // gyroscope
#define CTRL3_C 0x12        // register
#define OUT_TEMP_L 0x20
#define OUT_TEMP_H 0x21
#define OUTX_L_G 0x22
#define OUTX_H_G 0x23
#define OUTY_L_G 0x24
#define OUTY_H_G 0x25
#define OUTZ_L_G 0x26
#define OUTZ_H_G 0x27
#define OUTX_L_XL 0x28
#define OUTX_H_XL 0x29
#define OUTY_L_XL 0x2A
#define OUTY_H_XL 0x2B
#define OUTZ_L_XL 0x2C
#define OUTZ_H_XL 0x2D

// DEFINE
#define ADDR 0b1101011

// VARIABLE
static volatile int time = 24000000;
int j=0;
// Android CDC
char rx[64]; // the raw data
int rxPos = 0; // how much data has been stored
int gotRx = 0; // the flag
int rxVal = 0; // a place to store the int that was received


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
 */

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 */
// I2C SETTING

void initExpander();
void writei2c(unsigned char reg, unsigned char val);
unsigned char readi2c(unsigned char reg);
void acce_init();
void I2C_read_multiple(unsigned char address, unsigned char register, unsigned char *data, int length);

// LCD SETTING
char message[100];
int i;
void drawChar(unsigned short x, unsigned short y, char mess, unsigned short c1, unsigned short c2);
void drawString(unsigned short x, unsigned short y, char message[100], unsigned short c1, unsigned short c2);

// Motor control setting
void pwm_init();
void control_init();

/*******************************************************
 * USB CDC Device Events - Application Event Handler
 *******************************************************/

USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler
(
        USB_DEVICE_CDC_INDEX index,
        USB_DEVICE_CDC_EVENT event,
        void * pData,
        uintptr_t userData
        ) {
    APP_DATA * appDataObject;
    appDataObject = (APP_DATA *) userData;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;

    switch (event) {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

            USB_DEVICE_ControlSend(appDataObject->deviceHandle,
                    &appDataObject->getLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(appDataObject->deviceHandle,
                    &appDataObject->setLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *) pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent. Read the break duration */

            appDataObject->breakData = ((USB_DEVICE_CDC_EVENT_DATA_SEND_BREAK *) pData)->breakDuration;

            /* Complete the control transfer by sending a ZLP  */
            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:

            /* This means that the host has sent some data*/
            appDataObject->isReadComplete = true;
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. For now we accept all the data */

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We dont
             * do much with this data in this demo. */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the data write got completed. We can schedule
             * the next read. */

            appDataObject->isWriteComplete = true;
            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/***********************************************
 * Application USB Device Layer Event Handler.
 ***********************************************/
void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context) {
    USB_DEVICE_EVENT_DATA_CONFIGURED *configuredEventData;

    switch (event) {
        case USB_DEVICE_EVENT_SOF:

            /* This event is used for switch debounce. This flag is reset
             * by the switch process routine. */
            appData.sofEventHasOccurred = true;
            break;

        case USB_DEVICE_EVENT_RESET:

            /* Update LED to show reset state */

            appData.isConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuratio. We only support configuration 1 */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED*) eventData;
            if (configuredEventData->configurationValue == 1) {
                /* Update LED to show configured state */

                /* Register the CDC Device application event handler here.
                 * Note how the appData object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, APP_USBDeviceCDCEventHandler, (uintptr_t) & appData);

                /* Mark that the device is now configured */
                appData.isConfigured = true;

            }
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Switch LED to show suspended state */
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/*****************************************************
 * This function is called in every step of the
 * application state machine.
 *****************************************************/

bool APP_StateReset(void) {
    /* This function returns true if the device
     * was reset  */

    bool retVal;

    if (appData.isConfigured == false) {
        appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
        appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.isReadComplete = true;
        appData.isWriteComplete = true;
        retVal = true;
    } else {
        retVal = false;
    }

    return (retVal);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize(void) {
    /* Initialize Machine*/
    TRISAbits.TRISA4=0;     // green LED. Define pin RPA4 to output.
    TRISBbits.TRISB4=1;     // push-bottom. Define pin RPB4 to input.
    
    
    LATAbits.LATA4=0;
    
    initExpander();
    LCD_init();
    SPI1_init();
    acce_init();
    control_init();
    pwm_init();
    LCD_clearScreen(WHITE);
    //T5CKbits.T5CKR = 0b0100; // B9 is read by T5CK
    //T3CKbits.T3CKR = 0b0100; // B8 is read by T3CK
    
    
    
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    /* Device Layer Handle  */
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID;

    /* Device configured status */
    appData.isConfigured = false;

    /* Initial get line coding state */
    appData.getLineCodingData.dwDTERate = 9600;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bDataBits = 8;

    /* Read Transfer Handle */
    appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Write Transfer Handle */
    appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Intialize the read complete flag */
    appData.isReadComplete = true;

    /*Initialize the write complete flag*/
    appData.isWriteComplete = true;

    /* Reset other flags */
    appData.sofEventHasOccurred = false;
    //appData.isSwitchPressed = false;

    /* Set up the read buffer */
    appData.readBuffer = &readBuffer[0];

    startTime = _CP0_GET_COUNT();
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {
    /* Update the application state machine based
     * on the current state */

    switch (appData.state) {
        case APP_STATE_INIT:

            /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open(USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE);

            if (appData.deviceHandle != USB_DEVICE_HANDLE_INVALID) {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle, APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            } else {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device was configured */
            if (appData.isConfigured) {
                /* If the device is configured then lets start reading */
                appData.state = APP_STATE_SCHEDULE_READ;
            }
            break;

        case APP_STATE_SCHEDULE_READ:

            if (APP_StateReset()) {
                break;
            }

            /* If a read is complete, then schedule a read
             * else wait for the current read to complete */

            appData.state = APP_STATE_WAIT_FOR_READ_COMPLETE;
            if (appData.isReadComplete == true) {

                appData.isReadComplete = false;
                appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

                USB_DEVICE_CDC_Read(USB_DEVICE_CDC_INDEX_0,
                        &appData.readTransferHandle, appData.readBuffer,
                        APP_READ_BUFFER_SIZE);
                // Read r -v1.
                int ii = 0;
                // loop thru the characters in the buffer
                while (appData.readBuffer[ii] != 0) {
                    // if you got a newline
                    if (appData.readBuffer[ii] == '\n' || appData.readBuffer[ii] == '\r') {
                        rx[rxPos] = 0; // end the array
                        sscanf(rx, "%d", &rxVal); // get the int out of the array
                        gotRx = 1; // set the flag
                        break; // get out of the while loop
                    } else if (appData.readBuffer[ii] == 0) {
                        break; // there was no newline, get out of the while loop
                    } else {
                        // save the character into the array
                        rx[rxPos] = appData.readBuffer[ii];
                        rxPos++;
                        ii++;
                    }
                }
                
                if(appData.readBuffer[0]=='r'){
                    j=1;
                }
                 
                if (appData.readTransferHandle == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID) {
                    appData.state = APP_STATE_ERROR;
                    break;
                }
            }

            break;

        case APP_STATE_WAIT_FOR_READ_COMPLETE:
        case APP_STATE_CHECK_TIMER:
            if (gotRx || _CP0_GET_COUNT() - startTime > (48000000 / 2 / 5)) {
                appData.state = APP_STATE_SCHEDULE_WRITE;
            }

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was received or a switch was pressed.
             * The isReadComplete flag gets updated in the CDC event handler. */

            if (appData.isReadComplete || _CP0_GET_COUNT() - startTime > (48000000 / 2 / 100)) {
                appData.state = APP_STATE_SCHEDULE_WRITE;
            }

            break;


        case APP_STATE_SCHEDULE_WRITE:
            
                if (gotRx) {
                len = sprintf(dataOut, "got: %d\r\n", rxVal);
                i++;
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                        &appData.writeTransferHandle,
                        dataOut, len,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                rxPos = 0;
                gotRx = 0;
            } else {
                len = sprintf(dataOut, "%d\r\n", i);
                i++;
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                        &appData.writeTransferHandle, dataOut, len,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                startTime = _CP0_GET_COUNT();
            }
            
            

            if (APP_StateReset()) {
                break;
            }

            /* Setup the write */

            appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
            appData.isWriteComplete = false;
            appData.state = APP_STATE_WAIT_FOR_WRITE_COMPLETE;
            
            /* Function to check the basic connection */
            //len = sprintf(dataOut, "%d\r\n", i);
            //i++;
            
            /* Collecting data from accelerometer */
            
            _CP0_SET_COUNT(0);
            if(j==1){
            //i = 0;
            LATAbits.LATA4 = 1;

            signed short temp;
            signed short gyroX;
            signed short gyroY;
            signed short gyroZ;
            signed short accelX;
            signed short accelY;
            signed short accelZ;
            unsigned char data[14];

            // 1. READ WHO_AM_I
            unsigned char c;
            c= readi2c(WHO_AM_I);
            sprintf(message,"I'm %d :D", c);
            drawString(75,150, message, BLACK, WHITE);
        
            
            I2C_read_multiple(ADDR, OUT_TEMP_L, data,14);
            temp=data[0]|(data[1]<<8);

            gyroX=data[2]|(data[3]<<8);
            gyroY=data[4]|(data[5]<<8);
            gyroZ=data[6]|(data[7]<<8);

            accelX=data[8]|(data[9]<<8);
            accelY=data[10]|(data[11]<<8);
            accelZ=data[12]|(data[13]<<8);
            
            
            len= sprintf(dataOut, "%d %d %d %d %d %d %d\r\n", i+1, accelX, accelY, accelZ, gyroX, gyroY, gyroZ );
            i++;
            if(i==100){i=0;j=0;}
            int height= 160;
            int width= 128;
            unsigned short c1=GREEN;
            unsigned short c2=BLACK;
            int row = 0;
            int col = 0;
            int index = 0, index2=0;
            float d=0, d2=0;
            signed short x= accelX;

            d = (accelX-(-17000))/34000.0*128;
            index=d;

            d2= (accelY-(-17000))/34000.0*160;
            index2=d2;

            unsigned short h=4;

            // X axis Progress Bar

                for (row = 0; row < 128; row++) {
                    if (row<index){
                    for (col = 0; col < h; col++) {
                        LCD_drawPixel( row, 80+col, c2);
                    }}else if(row==index){
                        for (col = 0; col < h; col++) {
                        LCD_drawPixel( row, 80+col, c1);
                    }
                    }else{
                        for (col = 0; col < h; col++) {
                        LCD_drawPixel( row, 80+col, c2);
                    }
                    }
                }

            // Y axis Progress Bar

                for (col = 0; col < 160; col++) {
                    if (col<index2){
                    for (row = 0; row < h; row++) {
                        LCD_drawPixel( 64+row, col, c2);
                    }}else if(col==index2){
                        for (row = 0; row < h; row++) {
                        LCD_drawPixel( 64+row, col, c1);
                    }
                    }else{
                        for (row = 0; row < h; row++) {
                        LCD_drawPixel( 64+row, col, c2);
                        }
                    }
                }
            //********************Motor Control
            while(_CP0_GET_COUNT() - startTime < (48000000 / 2 * 5)){
                OC1RS = 100;
                OC4RS = 100;
            }
            
            OC1RS = 0;
            OC4RS = 0;
            
            
            
            
            
            //*********************************
            }else{
                dataOut[0]=0; len=1;
            }
            
            
            if (appData.isReadComplete) {
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                        &appData.writeTransferHandle,
                        appData.readBuffer, 1,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
            } else {
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                        &appData.writeTransferHandle, dataOut, len,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                startTime = _CP0_GET_COUNT();
            }
            break;

        case APP_STATE_WAIT_FOR_WRITE_COMPLETE:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was sent. The isWriteComplete
             * flag gets updated in the CDC event handler */

            if (appData.isWriteComplete == true) {
                appData.state = APP_STATE_SCHEDULE_READ;
            }

            break;

        case APP_STATE_ERROR:
            break;
        default:
            break;
    }
}

// HELPER FUNCTION
void initExpander() {
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    i2c_master_setup();

}

void writei2c(unsigned char reg, unsigned char val) {
    i2c_master_start(); // make the start bit
    i2c_master_send(ADDR << 1 | 0); // write the address, shifted left by 1, or'ed with a 0 to indicate writing
    i2c_master_send(reg); // the register to write to
    i2c_master_send(val); // the value to put in the register
    i2c_master_stop(); // make the stop bit
}

unsigned char readi2c(unsigned char reg) {
    i2c_master_start(); // make the start bit
    i2c_master_send(ADDR << 1 | 0); // write the address, shifted left by 1, or'ed with a 0 to indicate writing(ADDR=0x09))
    i2c_master_send(reg); // the register to read from
    i2c_master_restart(); // make the restart bit
    i2c_master_send(ADDR << 1 | 1); // write the address, shifted left by 1, or'ed with a 1 to indicate reading
    unsigned char r = i2c_master_recv(); // save the value returned
    i2c_master_ack(1); // make the ack so the slave knows we got it
    i2c_master_stop(); // make the stop bit
    return r;

}

void acce_init(){
    writei2c(CTRL1_XL, 0b10000010);
    writei2c(CTRL2_G, 0b10001000);
    writei2c(CTRL3_C, 0b00000100);
}

void I2C_read_multiple(unsigned char address, unsigned char regist, unsigned char *data, int length){
    int i=0;

    i2c_master_start(); // make the start bit
    i2c_master_send(ADDR << 1 | 0); // write the address, shifted left by 1, or'ed with a 0 to indicate writing(ADDR=0x09))
    i2c_master_send(regist); // the register to read from
    i2c_master_restart(); // make the restart bit
    i2c_master_send(ADDR << 1 | 1); // write the address, shifted left by 1, or'ed with a 1 to indicate reading
    //unsigned char r = i2c_master_recv(); // save the value returned
    for(i=0;i<length;i++){
        data[i]=i2c_master_recv();
        if(i<length-1){
        i2c_master_ack(0);}
        else{
            i2c_master_ack(1);
        }
    }
    i2c_master_stop(); // make the stop bit
    
}


// HELPER FUNCTION: LCD

void drawChar(unsigned short x, unsigned short y, char mess, unsigned short c1, unsigned short c2) {
    char row = mess - 0x20;
    int col = 0;

    for (col = 0; col < 5; col++) {
        char pixels = ASCII[row][col];
        int j = 0;
        for (j = 0; j < 8; j++) {
            if (x + col < 128 & y + j < 160) {
                if (((pixels >> j)&0x01) == 1) {
                    LCD_drawPixel(x + col, y + j, c1);
                } else {
                    LCD_drawPixel(x + col, y + j, c2);
                }
            }
        }
    }


}

void drawString(unsigned short x, unsigned short y, char message[100], unsigned short c1, unsigned short c2) {
    int i = 0;
    while (message[i]) {
        drawChar(x + 5 * i, y, message[i], c1, c2);
        i++;
    }
}

// Helper function for motor control

void pwm_init(){
    // Set pin for motor control
    RPA0Rbits.RPA0R = 0b0101; // A0 to OC1
    RPB13Rbits.RPB13R = 0b0101; // B13 to OC4
    
    // Use Timer2 as the base frequency for the PWM, set to 20kHz. Turn on the OCs with 0% duty
    
    T2CONbits.TCKPS = 0; // Timer2 prescaler N=1 (1:1)
    PR2 = 2399; // 48000000 Hz / 20000 Hz / 1 - 1 = 2399 (20kHz PWM from 48MHz clock with 1:1 prescaler)
    TMR2 = 0; // initial TMR2 count is 0
    OC1CONbits.OCM = 0b110; // PWM mode without fault pin; other OCxCON bits are defaults
    OC1RS = 0; // duty cycle
    OC1R = 0; // initialize before turning OC1 on; afterward it is read-only
    OC4CONbits.OCM = 0b110; // PWM mode without fault pin; other OCxCON bits are defaults
    OC4RS = 0; // duty cycle
    OC4R = 0; // initialize before turning OC4 on; afterward it is read-only
    T2CONbits.ON = 1; // turn on Timer2
    OC1CONbits.ON = 1; // turn on OC1
    OC4CONbits.ON = 1; // turn on OC4
    
    
    
}

void control_init(){
    TRISBbits.TRISB9=0; // B9 is read by T5CK
    TRISBbits.TRISB8=0; // B8 is read by T3CK
     
    
    // Configure Timer5 and Timer3 to count external pulses, and start at 0.
    T5CONbits.TCS = 1; // count external pulses
    PR5 = 0xFFFF; // enable counting to max value of 2^16 - 1
    TMR5 = 0; // set the timer count to zero
    T5CONbits.ON = 1; // turn Timer on and start counting
    T3CONbits.TCS = 1; // count external pulses
    PR3 = 0xFFFF; // enable counting to max value of 2^16 - 1
    TMR3 = 0; // set the timer count to zero
    T3CONbits.ON = 1; // turn Timer on and start counting
    
    // Set up Timer 4 as a 500Hz interrupt to perform the control math.
    
    T4CONbits.TCKPS = 2; // Timer4 prescaler N=4
    PR4 = 23999; // 48000000 Hz / 500 Hz / 4 - 1 = 23999 (500Hz from 48MHz clock with 4:1 prescaler)
    TMR4 = 0; // initial TMR4 count is 0
    T4CONbits.ON = 1;
    IPC4bits.T4IP = 4; // priority for Timer 4 
    IFS0bits.T4IF = 0; // clear interrupt flag for Timer4
    IEC0bits.T4IE = 1; // enable interrupt for Timer4

    
}


/*******************************************************************************
 End of File
 */
