/*
** utility.h
** 
*/

#include <GenericTypeDefs.h>

#define FALSE   0
#define TRUE    !FALSE

/*
** Simple delay routine using TMR1
**
*/

void Delayms (unsigned);
void PlayPattern (UINT32);

#define bufLen 80
extern UINT8   strBuf[bufLen];

void SendDataBuffer (const char*, UINT32);
UINT32 GetDataBuffer (char*, UINT32);
