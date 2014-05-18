/*
 * Project: 4.1 Touch
 * File:    main.c
 *
 * Requires: MAL 1306
 */

#include "PICconfig.h"
#include "uMedia.h"
#include "LCDterminal.h"
#include "TouchScreen.h"

#include "uMedia.h"


#include <stdio.h>          // sprintf


int main( void )
{
    char s[64];

    uMBInit();
    
    // 1. init the graphics
    LCDInit();
    SetBacklight( 200);

    // 2. init the touch timer
    TickInit( 10);
    
    // 3. init touch module (do not use NVM to store calibration data)
    TouchInit( NULL, NULL, NULL, NULL);
    
    // 4. splash screen
    LCDClear();
    LCDCenterString( -1, "Splash Screen");
    LCDCenterString(  1, "Tap to continue");
    while ( TouchGetX()<0);
    LCDClear();
    
    // 5. main loop
    while( 1 )
    {

        if (( TouchGetX() != -1) && ( TouchGetY() != -1))
        {
            sprintf( s, "\n  %d, %d", TouchGetX(), TouchGetY());
            LCDPutString( s);

//            if ( TouchGetX() > 160)
//            {
//                uMBInit(); FSInit();
//                ScreenCapture( "4-1-TERM.SCR");
//            }
        }

    } // main loop
} // main



