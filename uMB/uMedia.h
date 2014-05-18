/*
** uMedia.h
** 
**  Mikromedia configuration and basic peripherals access
**
*/

#ifndef _MIKROE_uMB
#define _MIKROE_uMB

#include "uMBconfig.h"

#include "HardwareProfile.h"            // need to know clock freq.

#include "drv_spi.h"

// function prototypes and macros
void uMBInit( void);            // init standard hw configuration
void TickInit( unsigned);       // init Timer3 as the main app tick

void ScreenCapture( char *filename);

#endif // _MIKROE_uMB

