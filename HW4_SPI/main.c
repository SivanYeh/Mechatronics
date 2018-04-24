/* 
 * File:   main.c
 * Author: Sivan
 *      PIC32 SPI communication
 * Created on April 16, 2018, 9:29 PM
 */

#include <xc.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <math.h>

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable secondary osc
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1 // use slowest wdt
#pragma config WINDIS = OFF // wdt no window mode
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_12 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

// DEFINE
#define time 24000000 // half of sysclk= 24MHz


#define CS LATBbits.LATB7       // chip select pin
//Variable
static volatile char buf[100] = {};
static volatile int DACfreq = 10;
static volatile int trifreq = 5;
// Function

void initSPI1(){
    TRISBbits.TRISB7=0;
    TRISBbits.TRISB14=0;
    TRISBbits.TRISB8=0;
    RPB8Rbits.RPB8R=0b0011;
    CS=1;
    
    SPI1CON=0;
    SPI1BUF;
    SPI1BRG = 1;//Set 10MHz(48MHz/(2*10MHz)-1)
    SPI1STATbits.SPIROV = 0;
    //SPI1CONbits.MODE32 = 0; //use 8 bit mode
    //SPI1CONbits.MODE16 = 0;
    SPI1CONbits.CKE = 1;
    SPI1CONbits.MSTEN = 1;
    SPI1CONbits.ON = 1;

    
    //SPI1TXB=;
    //SPI1RXB=;
    
}

//send a byte via spi and return the response
char SPI1_IO(char write){
    SPI1BUF = write;
    while(!SPI1STATbits.SPIRBF){ //wait to receive the byte.
        ;
    }
    return SPI1BUF;
}

void setVoltage(char a, short v){
    unsigned short t;
    
    t = a <<15;                 // set the channel to first bit
    t = t|0b0111000000000000;   // set the default.
    t = t|((v&0b1111111111)<<2);    //set the message.
    
    CS = 0;
    SPI1_IO(t>>8);              // send the first half of message
    SPI1_IO(t&0xFF);            // send the second half of message
    
    //SPI1_IO(t&0xFF00>>8);
    //SPI1_IO(t&0x00FF);  
    CS = 1;
    
    
}




int main(int argc, char** argv) {
    /*unsigned short master = 0, slave = 0;
    unsigned short rmaster = 0, rslave = 0;
     */
    
    
    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISAbits.TRISA4=0;     // green LED. Define pin RPA4 to output.
    TRISBbits.TRISB4=1;     // push-bottom. Define pin RPB4 to input.
    LATAbits.LATA4=0;       // turn off green LED.
    
    initSPI1();
    
    __builtin_enable_interrupts(); 
    
    /*sscanf(buf, "%04hx %04hx", &master, &slave);
    SPI1BUF = master;
    
    while(!SPI1STATbits.SPIRBF){
        ;
    }*/
    int i=0;
    int j=0;
    int g=0;
    while (1){
        _CP0_SET_COUNT(0);  // set the core timer counter to 0;
        
        //setVoltage(0,i);
        //i++;
        //if(i==1000){i=0;}
   
        
        //making sin wave 
        float f = 512 + 512* sin(i*2.0*3.14/1000.0);
        //float f = 512 + 512* sin(i/100.0);
         i++;
        setVoltage(0,f);
        if(i==1000){i=0;}
        
        //making triangle wave
        
        j++;
        
        if(j<1000){
            g++;
            setVoltage(1,g);
        }else{
            g--;
            setVoltage(1,g);
        }
        if(j==1999){j=0;g=1;}
        //if(j==2000){j=0;g=0;}
        
        
        //_CP0_SET_COUNT(0);  // set the core timer counter to 0;
        while(_CP0_GET_COUNT() < time/24000.0){ 
                LATAbits.LATA4=1;
            }
        _CP0_SET_COUNT(0);
            while(_CP0_GET_COUNT() < time/24000.0){ 
                LATAbits.LATA4=0;
            }
        
    }
}