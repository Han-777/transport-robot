/**
 * Title:        Lab 3 code
 * C File:       lab3_main.c
 * Platform:     PICmicro PIC16F877A @ 4 MHz
 * Written by:   Han Huang
 * Date:         7/11/2024
 * Function:     Displays the ADC reading from AN2 on an LCD, controls two LEDs based on the reading.
 */

// CONFIG
#pragma config FOSC = XT   // Oscillator Selection bits (XT oscillator)
#pragma config WDTE = OFF  // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF   // Low-Voltage Programming
#pragma config CPD = OFF   // Data EEPROM Memory Code Protection bit
#pragma config WRT = OFF   // Flash Program Memory Write Enable bits
#pragma config CP = OFF    // Flash Program Memory Code Protection bit

#include <xc.h>
#include <stdio.h>
#include "ee302lcd.h"

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 4000000
#endif

// Definitions
#define SWITCH_CLOSED 0   // Define switch state "CLOSED"
#define SWITCH1 RB0       // Define SW1 at PortB bit 0
#define LED_RED RC1       // Red LED connected to RC1
#define LED_GREEN RC0     // Green LED connected to RC0
#define LED_THRESHOLD 1.5 // Voltage threshold for LED control
typedef unsigned char uint8_t;

char lcdLine1[16] = {0};
char lcdLine2[16] = {0};

// Function prototypes
void initializeSystem(void);
void mainLoop(void);
void displayTitle(void);
void clearLCDStrings(void);
float readADCChannel(void);
void updateLCD(float voltage);
void controlLEDs(float voltage);

void main(void)
{
    initializeSystem(); // System initialization
    displayTitle();     // Display title on LCD
    while (!(SWITCH1 == SWITCH_CLOSED))
        ; // Wait for button press
    Lcd8_Clear();

    while (1) // Main program loop
    {
        mainLoop();
    }
}

void initializeSystem(void)
{
    Lcd8_Init();   // Initialize LCD in 8-bit mode
    ADCON0 = 0x51; // Select AN2 channel, set sample period to 8Tosc
    ADCON1 = 0x02; // Left justified result, set PCFG for analog input
    TRISA = 0x04;  // analog input
    TRISB = 0x01;  // Set PORTB for switches
    TRISC = 0x00;  // Set PORTC for LEDs
    PORTC = 0x02;  // Default state: Red LED ON
}

void mainLoop()
{
    static float lastVoltageReading = 0;     // Last voltage reading
    float voltageReading = readADCChannel(); // Read ADC value

    if (lastVoltageReading != voltageReading)
    {
        updateLCD(voltageReading); // Update LCD with voltage reading
    }

    controlLEDs(voltageReading); // Control LEDs based on voltage reading

    lastVoltageReading = voltageReading;
    __delay_ms(100); // Delay for approximately 10 samples per second
}

void updateLCD(float voltage)
{
    clearLCDStrings(); // Clear previous strings
    Lcd8_Clear();
    sprintf(lcdLine1, "ADC Voltage is");
    Lcd8_Set_Cursor(1, 1);
    Lcd8_Write_String(lcdLine1);
    sprintf(lcdLine2, "   %.1f V", voltage);
    Lcd8_Set_Cursor(2, 1);
    Lcd8_Write_String(lcdLine2);
}

void controlLEDs(float voltage)
{
    if (voltage < LED_THRESHOLD)
    {
        PORTC = 0x02; // Turn ON Red LED
    }
    else
    {
        PORTC = 0x01; // Turn ON Green LED
    }
}

void displayTitle(void)
{
    Lcd8_Set_Cursor(1, 1);
    Lcd8_Write_String("Laboratory 3");
    Lcd8_Set_Cursor(2, 1);
    Lcd8_Write_String(" EE302-ADC");
    __delay_ms(1500);
}

void clearLCDStrings(void)
{
    for (int i = 0; i < 16; i++)
    {
        lcdLine1[i] = 0x00;
        lcdLine2[i] = 0x00;
    }
}

float readADCChannel(void)
{
    ADCON0 |= 0x04; // Start ADC conversion
    __delay_us(50); // Wait for acquisition time (should be > 19.72us)

    while (ADCON0 & 0x04)
        ; // Wait until the conversion is complete

    uint8_t adcResult = ADRESH; // Read the ADC result

    // Convert ADC value to voltage
    return (5.0 * adcResult) / 256.0; // Convert to voltage (0-5V)
}
