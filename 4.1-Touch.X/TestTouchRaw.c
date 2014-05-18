/*
 * File:   TestTouchRaw.c
 *
 * Requires: MAL 1210
 */

#include <PICconfig.h>
#include "LCDTerminal.h"
#include "TouchScreen.h"


#include <stdio.h>          // sprintf


int main( void )
{
    char s[64];
    int  x, y;

    uMBInit();
    
    // 1. init the graphics
    LCDInit();
    SetBacklight( 200);

    // 2. set timer 3 and enable interrupt
    TickInit( 10);

    // 3. init touch module without calibration
    TouchHardwareInit( NULL);

    LCDPutString( "Test Touch Raw Input\n");

    // 4. main loop
    while( 1 )
    {
//        TouchDetectPosition();
        if (( TouchGetRawX() != -1) )//&& ( TouchGetRawY() != -1))
        {
                sprintf( s, "%04d, %04d\n", TouchGetRawX(), TouchGetRawY());
                LCDCenterString( 0, s);
        }
    } // main loop
} // main
