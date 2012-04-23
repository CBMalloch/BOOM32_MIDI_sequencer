/* 
** utility.c
**
** v1.0 LDJ 8/2/07
** V2.0 LDJ 10/22/07 rev B libraries
** V2.1 LDJ 10/23/07 removed RAM wait states, enable pre-fetch
*/

#include <p32xxxx.h>
#include <plib.h>
#include <project_setup.h>
#include <utility.h>

UINT8   strBuf[bufLen];

void _general_exception_handler(unsigned cause, unsigned exception_address_EPC) {

  // This will tell you the last address.
  // You sometimes have to find it using the disassembly listing
	
  static enum {
                 EXCEP_IRQ = 0,            // interrupt
                 EXCEP_AdEL = 4,           // address error exception (load or fetch)
                 EXCEP_AdES,               // address error exception (store)
                 EXCEP_IBE,                // bus error (ifetch)
                 EXCEP_DBE,                // bus error (load/store)
                 EXCEP_Sys,                // syscall
                 EXCEP_Bp,                 // breakpoint
                 EXCEP_RI,                 // reserved instruction
                 EXCEP_CpU,                // coprocessor unusable
                 EXCEP_Overflow,           // arithmetic overflow
                 EXCEP_Trap,               // trap (possible divide by zero)
                 EXCEP_IS1 = 16,           // implementation specfic 1
                 EXCEP_CEU,                // CorExtend Unuseable
                 EXCEP_C2E                 // coprocessor 2
   } _excep_codes;
	 
  static unsigned int _excep_code;
  static unsigned int _excep_addr;
  static unsigned int decoded_cause;

  // move from coprocessor 0
  asm volatile("mfc0 %0,$13" : "=r" (_excep_code));
  asm volatile("mfc0 %0,$14" : "=r" (_excep_addr));

  _excep_code = (_excep_code & 0x0000007C) >> 2;

  decoded_cause = (cause>>10) & 0x3f;

  UINT32 pattern = 0xffffffff;
//        snprintf (strBuf, bufLen, "GEH: code 0x%02x / 0x%02x\n",
//                                  _excep_code, decoded_cause);
//        SendDataBuffer (strBuf, strlen(strBuf));
//
//        snprintf (strBuf, bufLen, "GEH: location 0x%08x\n", _excep_addr);
//        SendDataBuffer (strBuf, strlen(strBuf));

        /*

  switch (decoded_cause) {
    case 0:
      snprintf (strBuf, bufLen, "GEH: caused by interrupt\n");
      break;
    case 4:
      snprintf (strBuf, bufLen, "GEH: address exception error..load or fetch\n");
      pattern = 0xc0000c00;
      break;
    case 5:
      snprintf (strBuf, bufLen, "GEH: address exception error..store\n");
      pattern = 0xc0000c0c;
      break;
    case 6:
      snprintf (strBuf, bufLen, "GEH: bus error exception fetch\n");
      break;
    case 7:
      snprintf (strBuf, bufLen, "GEH: bus error exception load or store\n");
      pattern = 0xc0000ccc;
      break;
    case 8:
      snprintf (strBuf, bufLen, "GEH: syscall exception\n");
      break;
    case 9:
      snprintf (strBuf, bufLen, "GEH: breakpoint exception\n");
      break;
    case 10:
      snprintf (strBuf, bufLen, "GEH: reserved instruction exception\n");
      break;
    case 11:
      snprintf (strBuf, bufLen, "GEH: coprocessor unusable exception\n");
      break;
    case 12:
      snprintf (strBuf, bufLen, "GEH: arithmetic overflow exception\n");
      break;
    case 13:
      snprintf (strBuf, bufLen, "GEH: trap exception\n");
      break;
    case 14:
      snprintf (strBuf, bufLen, "GEH: reserved 14\n");
      break;
    case 15:
      snprintf (strBuf, bufLen, "GEH: reserved 15\n");
      break;
    case 16:
      snprintf (strBuf, bufLen, "GEH: reserved 16\n");
      break;
    case 17:
      snprintf (strBuf, bufLen, "GEH: reserved 17\n");
      break;
    case 18:
      snprintf (strBuf, bufLen, "GEH: reserved 18\n");
      break;
    default:
      snprintf (strBuf, bufLen, "GEH: caused by interrupt\n");
      break;
  }

  SendDataBuffer (strBuf, strlen(strBuf));
*/

  programStatus = 0x00ff0000 + decoded_cause;
  PlayPattern (pattern);
} // exception handler
//

/*
** Simple Delay functions
**
** uses:    Timer1
** Notes:   Blocking function
*/

void Delayms (unsigned t)
{
  T1CON = 0x8000;     // enable TMR1, Tpb, 1:1
  while (t--) {  // t x 1ms loop
    TMR1 = 0;
    while (TMR1 < FPB/1000);
  }
} // Delayms

void PlayPattern (UINT32 bits) {
  LED_TRIS = 0;
  while (1) {
    int m = bits;
    int i;
    for (i = 0; i < 32; i++) {
      LED = m >> 31;
      m <<= 1;
      Delayms (50);
    }
    Delayms (500);
  }
}

void SendDataBuffer(const char *buffer, UINT32 size) {
  while(size) {
    while( ! UARTTransmitterIsReady (UART2)) ;

    UARTSendDataByte(UART2, *buffer);
    buffer++;
    size--;
  }

  while( ! UARTTransmissionHasCompleted (UART2)) ;
}

UINT32 GetDataBuffer(char *buffer, UINT32 max_size) {
  UINT32 num_char;

  num_char = 0;

  while(num_char < max_size) {
    UINT8 character;

    while( ! UARTReceivedDataIsAvailable (UART2)) ;

    character = UARTGetDataByte (UART2);

    if (character == '\r') break;

    *buffer = character;
    buffer++;
    num_char++;
  }
  *buffer = '\0';
  return num_char;
}
