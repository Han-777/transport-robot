/*
 * Title: EE302 Lab 2 - Button Controlled LCD Display
 * Author: Han Huang, 832201324, 22125451
 * Description: A simple embedded system to increment, decrement,
 *              and reset a counter using switches, displaying the result on an LCD.
 * Date: 2024-10-24
 */

#include <xc.h>
#include <stdio.h>    // Include Standard I/O header file
#include "ee302lcd.h" // Include LCD header file.

#ifndef _XTAL_FREQ
// Unless already defined assume 4MHz system frequency
// This definition is required to calibrate the delay functions, __delay_us() and __delay_ms()
#define _XTAL_FREQ 4000000
#endif

// CONFIG
#pragma config FOSC = XT   // Oscillator Selection bits (XT oscillator)
#pragma config WDTE = OFF  // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF   // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF   // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF   // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF    // Flash Program Memory Code Protection bit (Code protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// Definitions____________________________________________________________

#define CLOSED 0   // Define switch action "Closed" as 0
#define OPEN 1     // Define switch action "Open" as 1
#define SW_INC RB0 // Define switch action "SW_INC" as RB0
#define SW_DEC RB1 // Define switch action "SW_DEC" as RB1
#define SW_CLR RB2 // Define switch action "SW_CLR" as RB2

// globals _____________________________________________________

unsigned char gOutString[16] = {0x00}; // Define outString as an array of 16 characters
unsigned char counter = 0;             // Define counter as 0, the maximum value of counter is 8

// Prototypes_____________________________________________________________

void setup(void);           // Setup microcontroller and peripherals
void loop(void);            // Main loop to monitor button presses
void update_lcd(void);      // Update LCD with current counter value
void clear_outString(void); // Clear global output string buffer
void lcdTitle(void);        // Display title on LCD

// Main program
void main(void)
{
    setup();    // Initialize system
    lcdTitle(); // Display title on LCD

    // Superloop
    for (;;)
    {
        loop(); // Write data to LCD
    }
}

// Setup function
void setup(void)
{
    Lcd8_Init();  // Initialize LCD in 8-bit mode
    TRISB = 0x07; // Set RB0, RB1, and RB2 as inputs for buttons
    Lcd8_Clear(); // Clear LCD display
    update_lcd(); // Update LCD with initial counter value (0)
}

void loop(void)
{
    static unsigned char prev_sw_inc = OPEN; // Define previous switch action "SW_INC" as OPEN
    static unsigned char prev_sw_dec = OPEN; // Define previous switch action "SW_DEC" as OPEN
    static unsigned char prev_sw_clr = OPEN; // Define previous switch action "SW_CLR" as OPEN

    // Increment button logic
    if (SW_INC == CLOSED && prev_sw_inc != CLOSED) // If SW_INC is pressed and the previous state is not CLOSED
    {
        __delay_ms(50);                       // Debounce delay
        if (SW_INC == CLOSED && counter < 20) // Limit counter to 20
        {
            counter++;
            update_lcd();
        }
    }
    prev_sw_inc = SW_INC; // Update previous switch action "SW_INC"

    // Decrement button logic
    if (SW_DEC == CLOSED && prev_sw_dec != CLOSED) // If SW_DEC is pressed and the previous state is not CLOSED
    {
        __delay_ms(50);                      // Debounce delay
        if (SW_DEC == CLOSED && counter > 0) // Limit counter to 0
        {
            counter--;
            update_lcd();
        }
    }
    prev_sw_dec = SW_DEC; // Update previous switch action "SW_DEC"

    // Clear button logic
    if (SW_CLR == CLOSED && prev_sw_clr != CLOSED) // If SW_CLR is pressed and the previous state is not CLOSED
    {
        __delay_ms(50); // Debounce delay
        if (SW_CLR == CLOSED)
        {
            counter = 0;
            update_lcd();
        }
    }
    prev_sw_clr = SW_CLR; // Update previous switch action "SW_CLR"
}

/**
 * @brief Function to Update the LCD with the Current Counter Value
 *
 * @return void
 */
void update_lcd(void)
{
    clear_outString(); // Clear the output string buffer

    // Display "Button Press" centered on the first line (start at position 4)
    Lcd8_Set_Cursor(1, 4); // Set cursor to line 1, position 4
    Lcd8_Write_String("Button Press");

    // Display "=  xx" on the second line, where xx is the counter value
    Lcd8_Set_Cursor(2, 6); // Start at column 6 to align as shown in the picture
    // sprintf(gOutString, "=  %02d", counter); // Format counter as two-digit number (e.g., 05)
    // Format counter manually (if not using sprintf)
    gOutString[0] = '=';
    gOutString[1] = ' ';
    gOutString[2] = ' ';
    gOutString[3] = (counter / 10) + '0'; // Tens place
    gOutString[4] = (counter % 10) + '0'; // Ones place
    gOutString[5] = '\0';                 // Null-terminate the string
    Lcd8_Write_String(gOutString);
}

/**
 * @brief Function to Display Title on the LCD
 */
void lcdTitle(void)
{
    Lcd8_Set_Cursor(1, 1);             // Set cursor to line 1, position 1
    Lcd8_Write_String("Laboratory 2"); // Write title on line 1
    Lcd8_Set_Cursor(2, 1);             // Set cursor to line 2, position 1
    Lcd8_Write_String("EE302");        // Write subtitle on line 2
    __delay_ms(2000);                  // Delay for 2 second
}

/**
 * @brief Function to Clear the Global Output String Buffer
 *
 * @return void
 */
void clear_outString(void)
{
    for (int i = 0; i < 16; i++)
    {
        gOutString[i] = 0x00;
    }
}