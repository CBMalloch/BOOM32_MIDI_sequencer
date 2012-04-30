/*
** BOOOM32_pin_assignments.h   M5 BOOM32 project
**
** 2012-04-15 v0.3 cbm
**

 * A0  (02) AN0 light sensors 0 and 4 (AN0)
 * A1  (03) AN1 light sensors 1 and 5 (AN1)
 * B0  (04) PICkit 3 programming - MICRO PGD data
 * B1  (05) PICkit 3 programming - MICRO PGC clock
 * B2  (06) AN4 light sensors 2 and 6 (AN4)
 * B3  (07) AN5 light sensors 3 and 7 (AN5)
 * A2  (09) Stepper ch 0 red
 * A3  (10) Stepper ch 1 blue
 * B4  (11) Stepper ch 2 green
 * A4  (12) Stepper ch 3 yellow
 * B5  (14) LED
 * B7  (16) input tactile button switch
 * B8  (17) channel 0 MIDI on/off
 * B9  (18)
 * B10 (21) UART2 TX (assigned in main.c)
 * B11 (22) UART2 RX (assigned in main.c)
 * B13 (24) AN11 ADC input from potentiometer
 * B14 (25) power for bank 0 light sensors
 * B15 (26) power for bank 1 light sensors

*/

#ifndef __PIN_ASSIGNMENTS_H
#define __PIN_ASSIGNMENTS_H

#include <p32xxxx.h>

#define LED                   _LATB5
#define LED_TRIS              _TRISB5

#define LED_MIDI              _LATB8
#define LED_MIDI_TRIS         _TRISB8

#define BTN                   _RB7
#define BTN_TRIS              _TRISB7

#define LS0_ADC_ITM           (1 << 0)
#define LS0_ADC_VAL           ADC1BUF0
#define LS1_ADC_ITM           (1 << 1)
#define LS1_ADC_VAL           ADC1BUF1
#define LS2_ADC_ITM           (1 << 4)
#define LS2_ADC_VAL           ADC1BUF2
#define LS3_ADC_ITM           (1 << 5)
#define LS3_ADC_VAL           ADC1BUF3
#define POT_ADC_ITM           (1 << 11)
#define POT_ADC_VAL           ADC1BUF4

#define PWR_BANK0             _LATB14
#define PWR_BANK0_TRIS        _TRISB14
#define PWR_BANK1             _LATB15
#define PWR_BANK1_TRIS        _TRISB15

#define UART2_RX_PPS_REG      U2RXRbits.U2RXR
#define UART2_RX_PPS_ITM      0x03

#define UART2_TX_PPS_REG      RPB10Rbits.RPB10R
#define UART2_TX_PPS_ITM      0x02


// **********
// motor pins
// **********

#define STEPPER0              _LATA2
#define STEPPER0_TRIS         _TRISA2
#define STEPPER1              _LATA3
#define STEPPER1_TRIS         _TRISA3
#define STEPPER2              _LATB4
#define STEPPER2_TRIS         _TRISB4
#define STEPPER3              _LATA4
#define STEPPER3_TRIS         _TRISA4

#endif
