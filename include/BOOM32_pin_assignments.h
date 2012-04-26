/*
** SONUS32_pin_assignments.h   M5 SONUS32 project
**
** 2012-04-15 v0.3 cbm
**

 * A0  (02) AN0
 * A1  (03) AN1
 * B0  (04) PICkit 3 programming - MICRO PGD data
 * B1  (05) PICkit 3 programming - MICRO PGC clock
 * B2  (06) AN4
 * B3  (07) AN5
 * A2  (09) Stepper ch 0 red
 * A3  (10) Stepper ch 1 blue
 * B4  (11) Stepper ch 2 green
 * A4  (12) Stepper ch 3 yellow
 * B5  (14) LED
 * B7  (16) input tactile button switch
 * B8  (17)
 * B9  (18)
 * B10 (21) UART2 TX (assigned in main.c)
 * B11 (22) UART2 RX (assigned in main.c)
 * B13 (24) AN11 ADC input from potentiometer
 * B14 (25) AN10
 * B15 (26) AN9

*/

#define LED                   _RB5
#define LED_TRIS              _TRISB5

#define BTN                   _RB7
#define BTN_TRIS              _TRISB7

#define POT_ADC_ITM           ADC_CH0_POS_SAMPLEA_AN11

// #define UART2_SFR             RPxx
#define UART2_RX_PPS_REG      U2RXRbits.U2RXR
#define UART2_RX_PPS_ITM      0x03
// #define UART2_SDO_TRIS        _TRISxx

#define UART2_TX_PPS_REG      RPB10Rbits.RPB10R
#define UART2_TX_PPS_ITM      0x02


// **********
// motor pins
// **********

#define STEPPER0              _RA2
#define STEPPER0_TRIS         _TRISA2
#define STEPPER1              _RA3
#define STEPPER1_TRIS         _TRISA3
#define STEPPER2              _RB4
#define STEPPER2_TRIS         _TRISB4
#define STEPPER3              _RA4
#define STEPPER3_TRIS         _TRISA4

