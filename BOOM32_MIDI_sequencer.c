/*******************************************************************************
  BOOM32_MIDI_sequencer

  Is currently just reading ADC and putting the readings out UART2.
  Next step is to develop auto-contrast filter
  Then output MIDI
  
  Notes:
    Discriminating between off (clear) and on (black)
      1) determine light level by averaging all channels for one revolution
      2) a change in light level will be reflected by a correlated change in 
         the output of all channels taken together
      3) any smoothing of readings should be scaled in terms of beats
      4) don't like to use maxes or mins, since these reflect much noise
      
    However, we'll begin with a heuristic:
      Set the discrimination level for each channel x% (try 75) of the way
        from the min to the max observed during a measure of music 
        (say 1/4 revolution).
      Update the discriminator frequently.
      
*******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Includes
// *****************************************************************************
// *****************************************************************************

#include <math.h>
#include <p32xxxx.h>
#include <project_setup.h>
#include <utility.h>
#include <GenericTypeDefs.h>
#include <plib.h>

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

#define DO_MIDI 0
#define NOTE_NOTES 0

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

volatile int programStatus;
int clock_interrupt_period_us = 100;
volatile INT8 enableADC   = 0;
volatile INT8 enablePrint = 0;
volatile INT8 enableStep  = 0;
#define LED_BLINK_PERIOD_MS 500
#define ADC_INTERVAL_MS     100
#define PRINT_INTERVAL_MS   100
volatile int stepper_interval_us =  50000;
INT8 theStep = 0;
INT8 stepDirection = 1;  //CW
float smoothedPotValue = 512.0;
#define POTEWMAVALUE 0.4
// 2 phases per pole, 4 poles per step, 200 steps per revolution
#define PHASES_PER_REVOLUTION 1600
float rpm = 0;
#define ADC_READINGS_PER_LEVEL_REEVALUATION ( (int) ( 1 / 4.0 / rpm * 60 * 1000 / ADC_INTERVAL_MS ))

#define PHASES_PER_STEP 8
// R (0) <> G (3); Y (4) <> B (2)
UINT8 stepSequence[PHASES_PER_STEP][4] =  {
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
  UINT8   strBuf[bufLen];
  UINT8   bank = 0;
  #define NSENSORSPERBANK 4
  UINT32  cPot, cSensor[2][NSENSORSPERBANK];
  UINT8   i;
  #define NSENSORSINSTALLED 5
  UINT16 min_levels[NSENSORSINSTALLED], max_levels[NSENSORSINSTALLED], discriminators[NSENSORSINSTALLED];
  #define DISCRIMINATORLEVELPCT ( 75 )
  int steps_remaining_in_evaluation = 0;

  BOOL noteIsOn[NSENSORSINSTALLED] = {0, 0, 0, 0, 0};
  UINT32 sensorValue;

  SYSTEMConfigPerformance (FCY);
 
  LED_TRIS = 0;
  LED_MIDI_TRIS = 0;
  BTN_TRIS = 1;

  STEPPER0_TRIS = 0;
  STEPPER1_TRIS = 0;
  STEPPER2_TRIS = 0;
  STEPPER3_TRIS = 0;

  PWR_BANK0_TRIS = 0;  // disable both banks to start
  PWR_BANK1_TRIS = 1;

  UART2_RX_PPS_REG = UART2_RX_PPS_ITM;
  UART2_TX_PPS_REG = UART2_TX_PPS_ITM;

  UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
  UARTSetFifoMode(UART2, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
  UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
  if (DO_MIDI) {
    UARTSetDataRate(UART2, FPB, 31500);
  } else {
    UARTSetDataRate(UART2, FPB, 115200);
  }
  UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

  // timer interrupt stuff...

  // configure the core timer roll-over rate
  // the internal source is the Peripheral Clock
  // timer_preset = SYSCLK_freq / pb_div / additional prescale / desired_ticks_per_sec


  UINT16 timer_preset = (clock_interrupt_period_us * (FPB / 1000000)) / 8;
  OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_8, timer_preset);
 
  // set up timer interrupt with a priority of 2 and sub-priority of 0
  ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);

  // enable multi-vector interrupts
  INTEnableSystemMultiVectoredInt();


  // configure and enable the ADC
  
  CloseADC10();	// ensure the ADC is off before setting the configuration
  
  /*
    Notes from ref chapter
      must set the TRIS bits for these channels (1 (desired) by default)
      AD1CON1
        want form 32-bit integer (0b100)
        we set the SAMP bit to begin sampling --no--
          the ASAM bit will begin autosampling
          combine this with CLRASAM which will clear ASAM after one set of samples
          conversion should be auto after that
        autoconvert when sampling has concluded (SSRC = 0b111)
        Tad min is 65ns, which is 2.6 PBclock cycles
        Tad min is 83.33ns, which is 3.3 PBclock cycles
      AD1CON2
        CSCNA on enables scanning of inputs according to AD1CSSL
        maybe BUFM 2 8-word buffers rather than 1 16-word buffer
        ALTS alternates between mux A and mux B
      AD1CON3
        ADRC use peripheral bus clock ADC_CONV_CLK_PB
        TAD: use ADC clock divisor of 4 to give TAD of 100ns to exceed min of 83.33
        Sample time (SAMC): use 2 TADs to provide 200ns sampling to exceed min of 132
              
        New strategy: set the bit to power the necessary bank, wait 200 us,
        and then kick off a sample.
        
      AD1CHS
        CH0SA and CH0SB not used when scanning channels; CH0NA and CH0NB *are* used
      AD1PCFG
        zero bits to configure channels as analog; set to zero on reset
      AD1CSSL
        one bits to select channel for scan
        
      Will want to scan all channels on one bank, then interrupt to switch banks
      Protect user code by writing valid stuff during interrupt to user area
      
      
      
      settling time after switching is say 200 us (78us to 0.63 rise)
      
  */
  AD1CON1 =   ADC_FORMAT_INTG32           // output in integer
            | ADC_CLK_AUTO                // conversion begins with clock
            | ADC_AUTO_SAMPLING_OFF       // don't yet start autosample
          ;

  AD1CON2 =   ADC_VREF_AVDD_AVSS          // ADC ref internal
            | ADC_OFFSET_CAL_DISABLE      // disable offset test
            | ADC_SCAN_ON                 // enable scan mode (CSCNA)
            | ADC_SAMPLES_PER_INT_5 
            | ADC_ALT_BUF_OFF             // use single buffer
            | ADC_ALT_INPUT_OFF           // use MUX A only
          ;
 
  AD1CON3 =   ADC_SAMPLE_TIME_2           // use 2 TADs for sampling
            | ADC_CONV_CLK_PB             // use PBCLK
            | ADC_CONV_CLK_Tcy            // 4 PBCLK cycles, so TAD is 100ns
          ;

  // AD1CHS
  //          CH0NA negative input select for MUX A is Vref-
  //          ASAM will begin a sample sequence
  //          CLRASAM will stop autosampling after completing one sequence
  //          begin conversions automatically
  SetChanADC10 (ADC_CH0_NEG_SAMPLEA_NVREF);
  
  // AD1PCFG
  
  // these are in the order in which they will appear in BUF0
  AD1CSSL =   LS0_ADC_ITM 
            | LS1_ADC_ITM 
            | LS2_ADC_ITM 
            | LS3_ADC_ITM 
            | POT_ADC_ITM
          ;  
  
  EnableADC10();

  while(1) {

    // enableADC is set periodically (ADC_INTERVAL_MS) by an interrupt service routine
    if (enableADC) {
      
      // NOTE: the interrupt routine set AD1CON1bits.SAMP = 1;
      // use AN0, pin 2
      
      // wait for the conversions to complete
      // so there will be vaild data in ADC result registers
      while ( ! AD1CON1bits.DONE );
  
      for (i = 0; i < NSENSORSPERBANK; i++) {
        cSensor[bank][i] = ReadADC10(i);
      }
      cPot = POT_ADC_VAL;

      // current setup has 5 sensors: 2 in bank 0, 3 in bank 1
      if (1 && enablePrint && !DO_MIDI) {
        snprintf (strBuf, bufLen, 
          "%5d <-> %5d | %5d <-> %5d | %5d <-> %5d | %5d <-> %5d | %5d <-> %5d || %5d %3.2f\n",
          cSensor[0][0], discriminators[0],
          cSensor[0][1], discriminators[1],
          cSensor[1][0], discriminators[2],
          cSensor[1][1], discriminators[3],
          cSensor[1][2], discriminators[4],
          cPot,
          rpm
         );
        SendDataBuffer (strBuf, strlen(strBuf));
        enablePrint = 0;
      }
      smoothedPotValue = smoothedPotValue * (1.0 - POTEWMAVALUE)
                        + cPot * POTEWMAVALUE;
      cPot = floor(smoothedPotValue + 0.5);
    

#if (0)
      // direction not needed for BOOM32, but coded for testing
      if (cPot > 512) {
        stepDirection = 1;
      } else {
        stepDirection = -1;
      }

      // NOTE: my stepper asks for 12V and uses more than 200mA, which is the
      //   limit of my bench power supply...
      // That may be the limit of how fast we can step.
      
      // interval is least at extremes of pot travel
      float scaledValue = abs(cPot - 512.0) / 512.0;  // 1 at ends to 0 in middle
      scaledValue = 1.0 - sqrt(scaledValue);                 // 0 at ends to inf in middle
      int temp = floor(scaledValue * 1000000 + 0.5);
      #define min_stepper_interval_us 1000
      #define max_stepper_interval_us 125000
      stepper_interval_us = min_stepper_interval_us +
                    scaledValue * (max_stepper_interval_us-min_stepper_interval_us);

#else
      // desired us/phase:
      //                / 40-160 bpm
      //                * 16-32 beats per rev
      //                      ==> 40/32=1.25 - 160/16=10 RPM
      //                / 200 steps/rev
      //                / 4 coils / step
      //                / 2 phases/coil
      //                * 60 sec/min
      //                * 1e6 us/sec
      //                      --> factor = 15e6/400 = 3.75e4
      //                ==> is 3.75e3 (fast) - 6.0e4
      // is 7500 to 60000 us/phase
      #define min_stepper_interval_us   3750
      #define max_stepper_interval_us  30000
      stepper_interval_us = min_stepper_interval_us
        + ((1023 - cPot) *
          (max_stepper_interval_us - min_stepper_interval_us)) / 1024;
      rpm = ( 60 * 1e6 / stepper_interval_us) / PHASES_PER_REVOLUTION;
#endif

      steps_remaining_in_evaluation--;
      if (steps_remaining_in_evaluation <= 0) {
        for (i = 0; i < NSENSORSINSTALLED; i++) {
          discriminators[i] = min_levels[i] + (max_levels[i] - min_levels[i]) * DISCRIMINATORLEVELPCT / 100;
          min_levels[i] = 1024;
          max_levels[i] = 0;
        }
        // SendDataBuffer ("\n\n", 2);
        steps_remaining_in_evaluation = ADC_READINGS_PER_LEVEL_REEVALUATION;
      }
      
      // make determinations about what's on and what's off
      for (i = 0; i < NSENSORSINSTALLED; i++) {
        if (i < 2) {
          sensorValue = cSensor[0][i];
        } else {
          sensorValue = cSensor[1][i - 2];
        }
        if (sensorValue < min_levels[i]) min_levels[i] = sensorValue;
        if (sensorValue > max_levels[i]) max_levels[i] = sensorValue;
        
        BOOL currentValue = sensorValue < discriminators[i];
        if (noteIsOn[i] != currentValue) {
          if (currentValue) {
            // turn note on
            if (DO_MIDI) {
              snprintf (strBuf, bufLen, "%c%c%c", (char) 0x99, (char) i, (char) 64);
            } else if (NOTE_NOTES) {
              snprintf (strBuf, bufLen, "NoteOn (%d)\n", i);
            }
          } else {
            // turn note off
            if (DO_MIDI) {
              snprintf (strBuf, bufLen, "%c%c%c", (char) 0x89, (char) i, (char) 64);
            } else if (NOTE_NOTES) {
              snprintf (strBuf, bufLen, "NoteOff (%d)\n", i);
            }
          }
          if (i == 0) LED_MIDI = currentValue;
          SendDataBuffer (strBuf, strlen(strBuf));
          noteIsOn[i] = currentValue;
        }
      }

      if (0) {
        snprintf  (strBuf, bufLen, "%8d : %3d %3d : %3d %3d : %3d %3d : %3d %3d : %3d %3d :\n",
                    steps_remaining_in_evaluation,
                    min_levels[0], max_levels[0],
                    min_levels[1], max_levels[1],
                    min_levels[2], max_levels[2],
                    min_levels[3], max_levels[3],
                    min_levels[4], max_levels[4]
                  );
        SendDataBuffer (strBuf, strlen(strBuf));
      }


      
      // switch banks
      bank = 1 - bank;
      if (bank == 0) {
        PWR_BANK1 = 0;
        PWR_BANK1_TRIS = 1;
        PWR_BANK0 = 1;
        PWR_BANK0_TRIS = 0;
      } else {
        PWR_BANK0 = 0;
        PWR_BANK0_TRIS = 1;
        PWR_BANK1 = 1;
        PWR_BANK1_TRIS = 0;
      }

      AD1CON1bits.DONE = 0;
      enableADC = 0;

    }  // ADC enabled

    if (enableStep) {
      theStep += stepDirection;
      if (theStep >= PHASES_PER_STEP) theStep = 0;
      if (theStep < 0) theStep = PHASES_PER_STEP - 1;

      STEPPER0 = stepSequence[theStep][0];
      STEPPER1 = stepSequence[theStep][1];
      STEPPER2 = stepSequence[theStep][2];
      STEPPER3 = stepSequence[theStep][3];

      enableStep = 0;
    }

    if (enablePrint) {
//      snprintf (strBuf, bufLen, "  stepper_interval_ms = %d\n", stepper_interval_ms);
//      SendDataBuffer (strBuf, strlen(strBuf));
//      enablePrint = 0;
    }
  }  // infinite loop

  return -1;
}


void __ISR(_TIMER_1_VECTOR, ipl2) _Timer1Handler(void)
{
  // this interrupt should fire every 1 ms
  static int blinkRemainingUs = 0;
  static int adcRemainingUs = 0;
  static int stepRemainingUs = 0;
  static int printRemainingUs = PRINT_INTERVAL_MS * 1000;

  blinkRemainingUs -= clock_interrupt_period_us;
  if (blinkRemainingUs <= 0) {
    LED = 1 - LED;
    blinkRemainingUs = LED_BLINK_PERIOD_MS * 1000;
  }
 
  adcRemainingUs -= clock_interrupt_period_us;
  if (adcRemainingUs <= 0) {
    enableADC = 1;
    AD1CON1bits.ASAM = 1;
    adcRemainingUs = ADC_INTERVAL_MS * 1000;
  }
  
  stepRemainingUs -= clock_interrupt_period_us;
  if (stepRemainingUs <= 0) {
    enableStep = 1;
    stepRemainingUs = stepper_interval_us;
  }

  printRemainingUs -= clock_interrupt_period_us;
  if (printRemainingUs <= 0) {
    enablePrint = 1;
    printRemainingUs = PRINT_INTERVAL_MS * 1000;
  }

  mT1ClearIntFlag(); // clear the interrupt flag
}