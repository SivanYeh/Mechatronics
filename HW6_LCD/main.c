#include<xc.h>           // processor SFR definitions
#include<stdio.h>
#include<sys/attribs.h>  // __ISR macro
#include "ST7735.h"

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

// VARIABLE
char message[100];

// FUNCTION
void drawChar(unsigned short x, unsigned short y, char mess, unsigned short c1, unsigned short c2);
void drawString(unsigned short x, unsigned short y, char message[100], unsigned short c1, unsigned short c2);
void drawProgressBar(unsigned short x, unsigned short y,int i,unsigned short h, unsigned short len1, unsigned short c1,unsigned short len2, unsigned short c2);

int main() {

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
    SPI1_init();
    LCD_init();
    __builtin_enable_interrupts();  
    int i=0;
    LCD_clearScreen(WHITE);
    while(1){
        _CP0_SET_COUNT(0);  // set the core timer counter to 0;
        /*
        LCD_drawPixel(18,20, RED);
        LCD_drawPixel(19,20, RED);
        LCD_drawPixel(20,20, RED);
        LCD_drawPixel(21,20, RED);
        LCD_drawPixel(22,20, RED);
         */
        sprintf(message,"Hello world %d!", i);
        //drawChar(28,32, 'B', BLACK, WHITE);
        drawString(28,32, message, BLACK, WHITE);
        drawProgressBar(14,50,i,5,1,GREEN,100,BLACK);
        
        i++;
        if(i==100){i=0;}
        
        while(_CP0_GET_COUNT() < time/24){ 
            LATAbits.LATA4=1;
        }
        _CP0_SET_COUNT(0);
        while(_CP0_GET_COUNT() < time/24){ 
            LATAbits.LATA4=0;
        }
        
    }
}

void drawChar(unsigned short x, unsigned short y, char mess, unsigned short c1, unsigned short c2){
    char row = mess - 0x20;
    int col = 0;
    
    for(col=0;col<5;col++){
        char pixels = ASCII[row][col];
        int j = 0;
        //for(j = 0; j<8;j++){
        for(j = 0; j<8 ;j++){
            if (x+col< 128 & y+j <160){
                if(((pixels>>j)&0x01) ==1){
                    LCD_drawPixel(x+col, y+j, c1);
                }else{
                    LCD_drawPixel(x+col, y+j, c2);
                }
            }
        }
    }
     
    
}

void drawString(unsigned short x, unsigned short y, char message[100], unsigned short c1, unsigned short c2){
    int i=0;
    while(message[i]){
        drawChar(x+5*i, y ,message[i], c1, c2);
        i++;
    }
}

void drawProgressBar(unsigned short x, unsigned short y,int i, unsigned short h, unsigned short len1, unsigned short c1,unsigned short len2, unsigned short c2){
    
    
    char row = 0;
    int col = 0;
    
    for(row=0;row<len2;row++){
        for(col=0;col<h;col++){
            LCD_drawPixel(x+row, y+col, c2);
        }
    }
    
    for(row=0;row<(len1*i);row++){
        for(col=0;col<h;col++){
            LCD_drawPixel(x+row, y+col, c1);
        }
    }
        
}
 