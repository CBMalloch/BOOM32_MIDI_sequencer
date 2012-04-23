#include <BOOM32_pin_assignments.h>

// cbm kludge 8MHz for internal oscillator FRC
#define FCY     40000000L
// FPB should be the number of ticks of timer 1 (peripheral bus prescaled by 1) in 1 second
// bus errors if FPB > 10MHz! Not so. 40MHz is working fine..
#define FPB     40000000L

extern volatile int programStatus;


