/* 
 * File:   main.c
 * Author: pawankumar
 *
 */

#include <stdio.h>
#include <stdlib.h>

// PIC16F1619 Configuration Bit Settings

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = ON   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switch Over (Internal External Switch Over mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit cannot be cleared once it is set by software)
#pragma config ZCD = OFF        // Zero Cross Detect Disable Bit (ZCD disable.  ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PLLEN = OFF       // PLL Enable Bit (4x PLL is always enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// CONFIG3
#pragma config WDTCPS = WDTCPS1F// WDT Period Select (Software Control (WDTPS))
#pragma config WDTE = OFF        // Watchdog Timer Enable (WDT enabled)
#pragma config WDTCWS = WDTCWSSW// WDT Window Select (Software WDT window size control (WDTWS bits))
#pragma config WDTCCS = SWC     // WDT Input Clock Selector (Software control, controlled by WDTCS bits)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ  8000000

#define SCLK_GPIO_PORT &LATC
#define FSYNC_GPIO_PORT &LATC
#define DATA_GPIO_PORT &LATC

#define FSYNC_PIN_NUM   5
#define SCLK_PIN_NUM    4
#define DATA_PIN_NUM    3

#define FREQUENCY_INCREMENT_STATE   0
#define FREQUENCY_DECREMENT_STATE   1

/*
 * Frequency register = (Frequency_out * 2^28) / 25 * 10^6
 * 2^28 / 25 * 10^6 = 10.7 approx taken as 11 to avoid floating point calculation
 * Therefor Frequency register = Frequency_out * 11
 */
#define MULTIPLYING_FACTOR  11 

void ShiftOut(uint16_t Data);
void GpioSetPin(volatile uint8_t* GpioPort, uint8_t GpioPinNum, uint8_t GpioPinState);
void WriteFrequency(uint32_t Frequency);
void ClockInit(void);
void GpioInit(void);

int main() 
{   
    uint32_t OutputFreqency = 0;
    uint8_t State = FREQUENCY_INCREMENT_STATE;

    ClockInit();
    GpioInit();

    while(1)
    {
        switch(State)
        {
            case FREQUENCY_INCREMENT_STATE:
                WriteFrequency(OutputFreqency);
                OutputFreqency++;
                if(OutputFreqency > 1000) State = FREQUENCY_DECREMENT_STATE;
                break;

            case FREQUENCY_DECREMENT_STATE:    
                WriteFrequency(OutputFreqency);
                OutputFreqency--;
                if(OutputFreqency <= 100) State = FREQUENCY_INCREMENT_STATE;
                break;
        }
        __delay_ms(2);
    }

    return (EXIT_SUCCESS);
}

void GpioInit(void)
{
    TRISCbits.TRISC5 = 0;   //Set RC5 pin as output
    TRISCbits.TRISC4 = 0;   //Set RC4 pin as output
    TRISCbits.TRISC3 = 0;   //Set RC3 pin as output
}

void ClockInit(void)
{
    //Clock set to 8MHz
    OSCCONbits.IRCF3 = 1;
    OSCCONbits.IRCF2 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 0;
}

void GpioSetPin(volatile uint8_t* GpioPort, uint8_t GpioPinNum, uint8_t GpioPinState)
{   
    if(GpioPinState) *GpioPort |= (1 << GpioPinNum);  
    else *GpioPort &= (~(1 << GpioPinNum)); 
}

void ShiftOut(uint16_t Data)
{
   GpioSetPin(SCLK_GPIO_PORT, SCLK_PIN_NUM, 1);
   GpioSetPin(FSYNC_GPIO_PORT, FSYNC_PIN_NUM, 0);

   for( uint8_t Indx = 0; Indx < 16; Indx++ )
   {
     GpioSetPin(DATA_GPIO_PORT, DATA_PIN_NUM,(uint8_t)((Data & 0x8000) >> 15));
     Data <<=  1;
     GpioSetPin(SCLK_GPIO_PORT, SCLK_PIN_NUM, 0);
     GpioSetPin(SCLK_GPIO_PORT, SCLK_PIN_NUM, 1);
   }
    GpioSetPin(FSYNC_GPIO_PORT, FSYNC_PIN_NUM, 1);
}

void WriteFrequency(uint32_t Frequency)
{
    uint32_t FrequencyReg = Frequency * MULTIPLYING_FACTOR;
    uint32_t TempFrequency = 0;

    ShiftOut(0x2000);    //Control register
    TempFrequency = (FrequencyReg & (0x00003FFF)) | 0x00004000;
    ShiftOut((uint16_t)TempFrequency); //LSB bits of frequency register 
    TempFrequency = (FrequencyReg & (0x3FFF0000)) >> 16;
    TempFrequency |= 0x00004000;
    ShiftOut((uint16_t)TempFrequency);   //MSB bits of frequency register
}