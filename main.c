#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/examples/i2c1_master_example.h"
#include "LCD.h"
#include "LCD4Bit.h"
#include "RFID.h"
#include "math.h"
#include <stdio.h>
#include <string.h>

// * * * * * I2C info on page 16 * * * * * //
//#define MFRC552 0x28 //0101b ADDRESS?
//#define Command_Reg 0x01
//#define Status2_Reg 0x08         //page 43 of data sheet
//#define FIFOData_Reg 0x09        //page 44 of data sheet
//#define FIFOLevel_Reg 0x0A 
//#define ControlReg 0x0C
// * * * * * RFID commands * * * * * //
//#define Generate_RandomID 0x02  //generates a 10-byte random ID number
//#define Transmit 0x04           //transmits data from the FIFO buffer
//#define Receive 0x08            //activates the receiver circuits
//#define Transceive 0x0C         //transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
//#define FlushBuffer 0x80        //FIFO initialization

struct LCD theLCD;
double time;
uint16_t ms=0;
uint16_t s=0;

uint8_t *RFID_Buff[64];
unsigned char status;
unsigned char str[MAX_LEN];

void timer_callback();
void steup_LCD(char x [10],char y [10]);
void LCD_printplz(char x [10],char y [16]);
void check_data();
void empty_read();

void main(void){
    SYSTEM_Initialize();
    INTERRUPT_Initialize();
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();
    
    I2C1_Initialize();
    TMR2_Initialize();
    TMR2_SetInterruptHandler(timer_callback);    
    __delay_ms(200);
    
    MFRC522_Init();
    steup_LCD("Testing","RFID I2C");
    TMR2_Start();
    
    while(1){         
        empty_read();
        status = MFRC522_Request(PICC_REQIDL, str); //return status value to status, populate str with FIFO data. Find cards, return card type - Don't remove this sub
        status = MFRC522_Anticoll(str);             //Anti-collision, return card serial number 4 bytes
        memcpy(serNum, str, 5); // serNum is serial number with bits 1:PICC_ANTICOLL and bit2:0x20 copy over 5 bytes from raw SN to modified SN
        char str1[16];
        char str2[16];
        if (status == MI_OK )                   //if there is a new card detected  if (status == MI_OK || status == MI_NOTAGERR)
        {
         SetFormatRDM630();                    // take serNum and format it and return read     
         check_data();
         sprintf(str2, "%d%d%d%d%d%d%d%d",read[0],read[1],read[2],read[3], read[4], read[5], read[6], read[7]); //sprintf(str2, "ID1:%d %d %d %d", read[4], read[5], read[6], read[7]);//prints 450
        }
//        else
//        {
//         sprintf(str2, "ID2:%d %d %d %d", read[4], read[5], read[6], read[7]);
//         MFRC522_Halt();                       // Command card into hibernation
//         __delay_ms(50);                       // wait for low consumption
//        }
        else
        {
         sprintf(str2, "%d%d%d%d%d%d%d%d",read[0],read[1],read[2],read[3], read[4], read[5], read[6], read[7]);
         MFRC522_Halt();                       // Command card into hibernation
         __delay_ms(50);                       // wait for low consumption
        }
        time = (ms/1000.00) + s;
        sprintf(str1, "%d t=%.2fs ID:",status, time);
        //sprintf(str2, "ID: %dC", read);
        LCD_printplz(str1,str2);
        
    }
}
void empty_read(){
    for(int r=0; r<11; r++){
        read[r]=0;
    }
}

void timer_callback(){
    ms= ms+1;
    if(ms >= 1000){
        ms= ms-1000;
        s=s+1;
//        IO_RE0_Toggle();
    }
}

void check_data(){
    int checkByte1=0;
    int checkByte2=0;
    while ( checkByte1!=9 && checkByte2!=9)
    {

        for(int d=0; d<8; d++){
            if(keyI[d] == read[d])
            {
                checkByte1=checkByte1++;
                if(checkByte1>7){
                    IO_RB0_SetHigh();
                    __delay_ms(200);
                    IO_RB0_SetLow();
                    checkByte1=9;
                }
            }
            if(keyII[d] == read[d])
            {
                checkByte2=checkByte2++;
                if(checkByte2>7){
                    IO_RB1_SetHigh();
                    __delay_ms(200);
                    IO_RB1_SetLow();
                    checkByte2=9;
                }
            }
        }
    }
}

void steup_LCD(char x [10],char y [10]){
    // if error, then use the following code
    PORTD = 0;
    ANSELD = 0;
    TRISD = 0;
    LCD_initParallel(&theLCD, LCD_4BITMODE, &LATD, &LATD, 7, 6);
    LCD_begin(&theLCD, 16, 2, LCD_5x8DOTS);
    LCD_clear(&theLCD);
    LCD_home(&theLCD);
    LCD_setCursor(&theLCD, 0, 0);
    LCD_printString(&theLCD, x);
    LCD_setCursor(&theLCD, 0, 1);
    LCD_printString(&theLCD, y);
    __delay_ms(200);
}

void LCD_printplz(char x [10],char y [16]){
    // if error, then use the following code
    LCD_clear(&theLCD);
    LCD_home(&theLCD);
    LCD_setCursor(&theLCD, 0, 0);
    LCD_printString(&theLCD, x);
    LCD_setCursor(&theLCD, 0, 1);
    LCD_printString(&theLCD, y);
}

