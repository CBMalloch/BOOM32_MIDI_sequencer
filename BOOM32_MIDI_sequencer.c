/*******************************************************************************
  BOOM32_MIDI_sequencer

  Is currently just reading ADC and putting the readings out UART2.
  Next step is to output MIDI instead
      
*******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Includes
// *****************************************************************************
// *****************************************************************************

#include <p32xxxx.h>
#include <GenericTypeDefs.h>
#include <plib.h>
#include <project_setup.h>
#include <utility.h>

// *****************************************************************************
// *****************************************************************************
// Section: Configuration bits
// *****************************************************************************
// *****************************************************************************

#pragma config POSCMOD = OFF      // primary oscillator conf
#pragma config OSCIOFNC = OFF     // CLKO disconnect from output pin
#pragma config FSOSCEN = OFF      // secondary oscillator disable

// FNOSC FRCPLL selects FRC for input to PLL as well as PLL output for SYSCLK
#pragma config FPLLIDIV = DIV_2, FPLLMUL = MUL_20, FPLLODIV = DIV_2  // PLL 40MHz
#pragma config FNOSC = FRCPLL       // oscillator selection
#pragma config FPBDIV = DIV_1     // peripheral bus clock divisor 40MHz

#pragma config FWDTEN = OFF       // watchdog timer
#pragma config FCKSM = CSECME     // clock switching and monitor selection

#pragma config CP = OFF           // code (read and modify) protect
#pragma config BWP = OFF          // boot flash write-protect
#pragma config PWP = OFF          // program flash write-protect

// #pragma config ICESEL = ICS_PGx1  // ice/icd comm channel select
#pragma config JTAGEN = OFF       // JTAG disable
// JTAG port pins are multiplexed with PortA pins RA0, RA1, RA4, and RA5

// *****************************************************************************
// *****************************************************************************
// Section: Definitions
// *****************************************************************************
// *****************************************************************************

volatile int programStatus;

// *****************************************************************************
// *****************************************************************************
// Section: Function Prototypes
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Constant Data
// *****************************************************************************
// *****************************************************************************

int clock_interrupt_period_us = 100;
INT8 enableADC   = 0;
INT8 enablePrint = 0;
INT8 enableStep  = 0;
#define LED_blink_period_ms 500
#define ADC_interval_ms     100
#define print_interval_ms   500
int stepper_interval_us =  50000;
INT8 theStep = 0;
INT8 stepDirection = 1;  //CW

#define NSTEPS 8
// R (0) <> G (3); Y (4) <> B (2)
UINT8 stepSequence[NSTEPS][4] = {
                                  {1, 0, 0, 0},
                                  {1, 0, 0, 1},
                                  {0, 0, 0, 1},
                                  {0, 0, 1, 1},
                                  {0, 0, 1, 0},
                                  {0, 1, 1, 0},
                                  {0, 1, 0, 0},
                                  {1, 1, 0, 0},
                                };

// *****************************************************************************
// *****************************************************************************
// Section: Code
// *****************************************************************************
// *****************************************************************************

int main(void)
{

  SYSTEMConfigPerformance (FCY);
 
  UINT32  ADCresult;
   
  // 1. initializations

  LED_TRIS = 0;
  BTN_TRIS = 1;

  STEPPER0_TRIS = 0;
  STEPPER1_TRIS = 0;
  STEPPER2_TRIS = 0;
  STEPPER3_TRIS = 0;

  UART2_RX_PPS_REG = UART2_RX_PPS_ITM;
  // U2RXRbits.U2RXR = 3;   //SET U2RX to RPB11 (Pin 22)
  UART2_TX_PPS_REG = UART2_TX_PPS_ITM;
  // RPB10Rbits.RPB10R = 2; //SET RPB10R (Pin 21) to U2TX

  UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
  UARTSetFifoMode(UART2, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
  UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_2);
  UARTSetDataRate(UART2, FPB, 19200);
  UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

  // timer interrupt stuff...

  // configure the core timer roll-over rate (1 msec)
  // the internal source is the Peripheral Clock
  // timer_preset = SYSCLK_freq / pb_div / additional prescale / desired_ticks_per_sec
  // = 40MHz / 1 / 64 / 1000 = 625
  // 40MHz / 1 / 8 / 100 = 625*8 = 5000

  UINT16 timer_preset = (clock_interrupt_period_us / 8) * (FPB / 1000000);
  OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_8, timer_preset);
 
  // set up timer interrupt with a priority of 2 and sub-priority of 0
  ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);

  // enable multi-vector interrupts
  INTEnableSystemMultiVectoredInt();


  // configure and enable the ADC
  
  CloseADC10();	// ensure the ADC is off before setting the configuration
  
  // CHONA negative input select for MUX A is Vref-
  // channel 0 positive input for mux A is defined in header to POT_ADC_ITM
  //   currently AN11, pin 24
  SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | POT_ADC_ITM );
  
  // Turn module on | ouput in integer | conversion begins with clock | disable autosample
  AD1CON1 = ADC_FORMAT_INTG32 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_OFF;
  // ADC ref internal    | disable offset test    | disable scan mode | 1 samples / interrupt | use single buffer | use MUX A only
  AD1CON2 = ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF;
  // use ADC internal clock | set sample time
  // could also try ADC_CONV_CLK_PB which is the 125ns peripheral clock
  AD1CON3 = ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15;
  // select pin AN0 for input scan
  // AD1CSSLbits.CSSL0 = 1;

  // EnableADC10(); // Enable the ADC
  AD1CON1bits.ON = 1;
  // mAD1IntEnable(1);


  while(1) {

    // enableADC is set periodically (ADC_interval_ms) by an interrupt service routine
    if (enableADC) {
      
      // NOTE: the interrupt routine set AD1CON1bits.SAMP = 1;
      // use AN0, pin 2
      
      // wait for the conversion to complete so there will be vaild data in ADC result registers
      while ( ! AD1CON1bits.DONE );
  
      ADCresult = ADC1BUF0;

      AD1CON1bits.DONE = 0;
      enableADC = 0;
    }

#if (0)
    // direction not needed for BOOM32, but coded for testing
    if (ADCresult > 512) {
      stepDirection = 1;
    } else {
      stepDirection = -1;
    }

    // interval is least at extremes of pot travel
    stepper_interval_us = max(200, abs(512 - ADCresult) * 200);
#else
    // desired us/phase:
    //                / 40-160 bpm 
    //                * 60 sec/min 
    //                * 1e6 us/sec
    //                * 32 beats per rev 
    //                / 200 steps/rev
    //                / 2 phases/step
    // is 30000 to 120000
    stepper_interval_us = 30000 + (1023 - ADCresult) * 90;
#endif

    if (enableStep) {
//      snprintf (strBuf, bufLen, "Step...\n");
//      SendDataBuffer (strBuf, strlen(strBuf));

      theStep += stepDirection;
      if (theStep >= NSTEPS) theStep = 0;
      if (theStep < 0) theStep = NSTEPS - 1;

      STEPPER0 = stepSequence[theStep][0];
      STEPPER1 = stepSequence[theStep][1];
      STEPPER2 = stepSequence[theStep][2];
      STEPPER3 = stepSequence[theStep][3];

      enableStep = 0;
    }

    if (enablePrint) {
//      snprintf (strBuf, bufLen, "  stepper_interval_ms = %d\n", stepper_interval_ms);
//      SendDataBuffer (strBuf, strlen(strBuf));
      enablePrint = 0;
    }
  }

  return -1;
}


void __ISR(_TIMER_1_VECTOR, ipl2) _Timer1Handler(void)
{
  // these interrupts should hit every 1 ms
  static int blinkCounter = 0;
  static int adcCounter = 0;
  static int stepCounter = 0;
  static int printCounter = 0;

  blinkCounter -= clock_interrupt_period_us;
  if (blinkCounter <= 0) {
    LATBINV = 0x0020;
    blinkCounter = LED_blink_period_ms * 1000;
  }
  
  adcCounter -= clock_interrupt_period_us;
  if (adcCounter <= 0) {
    enableADC = 1;
    AD1CON1bits.SAMP = 1;
    adcCounter = ADC_interval_ms * 1000;
  }
  
  stepCounter -= clock_interrupt_period_us;
  if (stepCounter <= 0) {
    enableStep = 1;
    stepCounter = stepper_interval_us;
  }

  printCounter -= clock_interrupt_period_us;
  if (printCounter <= 0) {
    enablePrint = 1;
    printCounter = print_interval_ms * 1000;
  }

  mT1ClearIntFlag(); // clear the interrupt flag
}