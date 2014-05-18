/*
 * uMedia.c
 *
 * PIC32MX7 Mikromedia Plus
 */

#include "uMedia.h"
#include <plib.h>

#include "m25p80.h"
#include "Touchscreen.h"
#include "Graphics/Graphics.h"
#include "MDD File System/FSIO.h"

#define _TIMER3_ISR  __attribute__(( vector(_TIMER_3_VECTOR), interrupt(ipl1), nomips16))


void _TIMER3_ISR _T3Interrupt( void)
{
    IFS0bits.T3IF = 0;          // Clear flag

#ifdef _TOUCH
    TouchDetectPosition();
#endif
}

#define TICK_PERIOD( ms)  (GetPeripheralClock() * (ms)) / 8000

void TickInit( unsigned period_ms)
{
    // Initialize Timer3
    TMR3 = 0;
    PR3 = TICK_PERIOD( period_ms);
    T3CONbits.TCKPS = 1;        // Set prescale to 1:8
    IFS0bits.T3IF = 0;          // Clear flag
    IPC3bits.T3IP = 1;          // SetPriorityIntT3( 1);
    IEC0bits.T3IE = 1;          // Enable interrupt
    T3CONbits.TON = 1;          // Run timer
}


void uMBInit( void)
{
#ifdef _SFLASH
    DRV_SPI_INIT_DATA spi_config = SPI_FLASH_CONFIG;
#endif
    // 1. disable analog inputs
    AD1PCFG = 0xFFFF;   // all inputs digital

    // 1. disable the JTAG port
    mJTAGPortEnable( 0);

    // 2. Config system for max performance
    SYSTEMConfig( GetSystemClock(), SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    // 3. allow vectored interrupts
    INTEnableSystemMultiVectoredInt();   // Interrupt vectoring

#ifdef _SFLASH
    // 5. Initialize the serial Flash CS I/O
    SST25_CS_LAT = 1;
    SST25_CS_TRIS = 0;
    FlashInit( &spi_config);
#endif
} // uMBInit


// replace the exit handler to avoid 512 byte buffer allocation
void exit( int e)
{
    while(1);
} // exit handler


// redefine the exception handler for debugging purposes
void _general_exception_handler( unsigned c, unsigned s)
{
    while (1);
} // exception handler



// define the stopwatch function
void MMBStartStopwatch( void)
{
    OpenCoreTimer(0);

    // resets the core timer count
    WriteCoreTimer( 0);
} // start_timer


// returns time lapsed in milliseconds
// maximum measurable interval 107s (worst case @80MHz)
double MMBReadStopwatch( void)
{
    unsigned int ui;

    // get the core timer count
    ui = ReadCoreTimer();

    // convert in milliseconds (1 core tick = 2 SYS cycles)
    return ( ui * 2000.0 /GetSystemClock());
} // read_timer



#ifdef _SCREENCAPTURE
void ScreenCapture( char *filename)
{
    FSFILE *fp;
    GFX_COLOR Row[ 320];
    int i, j;

    // open file
    fp = FSfopen( filename, FS_WRITE);
    if ( fp != NULL)
    {
        // dump contents of the screen
        for(j=0; j<=GetMaxY(); j++)
        {
            // row by row
            for( i=0; i<=GetMaxX(); i++)
            {
                Row[ i] = GetPixel( i, j);
            }

            // write buffer to file
            FSfwrite( Row, sizeof(Row), 1, fp);
        }

        // close file
        FSfclose( fp);

    }
}
#endif // _SCREENCAPTURE
 