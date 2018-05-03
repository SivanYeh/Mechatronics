#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "i2c_master_noint.h"
#include "ST7735.h"
#include<stdio.h>

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


// LSM6DS33 DEFINE

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
    TRISAbits.TRISA4 = 0; // green LED. Define pin RPA4 to output.
    TRISBbits.TRISB4 = 1; // push-bottom. Define pin RPB4 to input.
    LATAbits.LATA4 = 0; // turn off green LED.
    initExpander();
    LCD_init();
    SPI1_init();
    acce_init();
    LCD_clearScreen(WHITE);
    
    __builtin_enable_interrupts();
    
    i = 0;
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
        sprintf(message,"%d", c);
        drawString(100,150, message, BLACK, WHITE);
        
    while (1) {
        _CP0_SET_COUNT(0);
          
        I2C_read_multiple(ADDR, OUT_TEMP_L, data,14);
        temp=data[0]|(data[1]<<8);
        
        gyroX=data[2]|(data[3]<<8);
        gyroY=data[4]|(data[5]<<8);
        gyroZ=data[6]|(data[7]<<8);
        
        accelX=data[8]|(data[9]<<8);
        accelY=data[10]|(data[11]<<8);
        accelZ=data[12]|(data[13]<<8);
        
        
        /* print accelX, accelY, accelZ 
        sprintf(message,"X = %d  ", accelX);
        drawString(7,14, message, BLACK, WHITE);  
        
        sprintf(message,"Y = %d  ", accelY);
        drawString(7,28, message, BLACK, WHITE);  
        
        sprintf(message,"Z = %d  ", accelZ);
        drawString(7,42, message, BLACK, WHITE);  
        */
        // Draw the bar
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
    
    
    sprintf(message,"X = %d ", index-64);
    drawString(80,14, message, RED, WHITE);
  
    sprintf(message,"Y = %d ", index2-80);
    drawString(80,28, message, BLUE, WHITE);
    
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
       
            
        while (_CP0_GET_COUNT() < time/20) {
            LATAbits.LATA4 = 1;
        }
        _CP0_SET_COUNT(0);
        while (_CP0_GET_COUNT() < time/20) {
            LATAbits.LATA4 = 0;
            
        }
    }
}

// HELPER FUNCTION: I2C

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

    
    
        