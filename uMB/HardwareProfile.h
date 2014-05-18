/*
 * File:   HardwareProfile.h
 *
 * Hardware platform: PIC32 Mikromedia Plus board
 */

#ifndef _HARDWARE_PROFILE_H
#define _HARDWARE_PROFILE_H

#define uMPIC32    432Plus

#include <xc.h>
//#include <adc.h>


/*********************************************************************
 * PIC24 clock 
 ********************************************************************/
#define GetSystemClock()        (80000000ull)
#define GetInstructionClock()   GetSystemClock()

/*********************************************************************
* Macro: #define	GetPeripheralClock()
*
* Overview: This macro returns the peripheral clock frequency used in Hertz.
* value for PIC32 is <PRE>(GetSystemClock()/(1<<OSCCONbits.PBDIV)) </PRE>
*
********************************************************************/
#define	GetPeripheralClock()    (GetSystemClock()/(1<<OSCCONbits.PBDIV))


/*********************************************************************
*  Graphics Panel configuration for the MikroE- PIC24 uMB Board
********************************************************************/
#define GRAPHICS_HARDWARE_PLATFORM      uMPIC32Plus


// display type SSD1963 controller - 16-bit PMP interface
#define GFX_USE_DISPLAY_CONTROLLER_SSD1963

// interface
#define USE_GFX_PMP
#define USE_16BIT_PMP
#define PMP_DATA_SETUP_TIME             (0)
#define PMP_DATA_WAIT_TIME              (0)
#define PMP_DATA_HOLD_TIME              (0)

/*********************************************************************
 * IOs for the Display Controller
 *********************************************************************/
// Definitions for reset pin
#define DisplayResetConfig()        _TRISC1 = 0
#define DisplayResetEnable()        _LATC1  = 0
#define DisplayResetDisable()       _LATC1  = 1

// Definitions for RS pin
#define DisplayCmdDataConfig()      _TRISB15 = 0
#define DisplaySetCommand()         _LATB15  = 0
#define DisplaySetData()            _LATB15  = 1

// Definitions for CS pin
#define DisplayConfig()             _TRISF12 = 0
#define DisplayEnable()             _LATF12  = 0
#define DisplayDisable()            _LATF12  = 1

// Definitions for Backlight control pin
#define DisplayBacklightConfig()    // controlled indirectly
#define DisplayBacklightOff()       // controlled indirectly
#define DisplayBacklightOn()        // controlled indirectly

/*
*********************************************************************
* TIMING PARAMETERS FOR 4.3" TFT PANEL (480x272):
* PART# TFT480272
*********************************************************************
*/
/*********************************************************************
* Overview: Horizontal and vertical display resolution
*                  (from the glass datasheet).
*********************************************************************/
#define DISP_HOR_RESOLUTION	480
#define DISP_VER_RESOLUTION	272
/*********************************************************************
* Overview: Image orientation (can be 0, 90, 180, 270 degrees).
*********************************************************************/
#define DISP_ORIENTATION	0
/*********************************************************************
* Overview: Panel Data Width
*********************************************************************/
#define DISP_DATA_WIDTH         24
/*********************************************************************
* Overview: LSHIFT Polarity Swap
* If defined LSHIFT is a falling trigger
*********************************************************************/
//#define DISP_INV_LSHIFT
/*********************************************************************
* Overview: Horizontal synchronization timing in pixels
*                  (from the glass datasheet).
*********************************************************************/
#define DISP_HOR_PULSE_WIDTH	10
#define DISP_HOR_BACK_PORCH	2
#define DISP_HOR_FRONT_PORCH	2
/*********************************************************************
* Overview: Vertical synchronization timing in lines
*                  (from the glass datasheet).
*********************************************************************/
#define DISP_VER_PULSE_WIDTH	20
#define DISP_VER_BACK_PORCH	2
#define DISP_VER_FRONT_PORCH	2


/*********************************************************************
 * IOs for the Touch Screen
 *********************************************************************/

#define USE_TOUCHSCREEN_STMPE610

#define TP_INT          _RA14           // Touch screen controller INT line
#define TP_INT_TRIS     _TRISA14

// serial Flash calibration addresses
#define ADDRESS_RESISTIVE_TOUCH_VERSION	(unsigned long)0xFFFFFFFE
#define ADDRESS_RESISTIVE_TOUCH_ULX     (unsigned long)0xFFFFFFFC
#define ADDRESS_RESISTIVE_TOUCH_ULY     (unsigned long)0xFFFFFFFA
#define ADDRESS_RESISTIVE_TOUCH_URX     (unsigned long)0xFFFFFFF8
#define ADDRESS_RESISTIVE_TOUCH_URY     (unsigned long)0xFFFFFFF6

#define ADDRESS_RESISTIVE_TOUCH_LLX     (unsigned long)0xFFFFFFF4
#define ADDRESS_RESISTIVE_TOUCH_LLY     (unsigned long)0xFFFFFFF2
#define ADDRESS_RESISTIVE_TOUCH_LRX     (unsigned long)0xFFFFFFF0
#define ADDRESS_RESISTIVE_TOUCH_LRY     (unsigned long)0xFFFFFFEE

// define the functions to call for the non-volatile memory
// check out touch screen module for definitions of the following function pointers
// used: NVM_READ_FUNC, NVM_WRITE_FUNC & NVM_SECTORERASE_FUNC
#define NVMSectorErase      ((NVM_SECTORERASE_FUNC)&SST25SectorErase)
#define NVMWrite            ((NVM_WRITE_FUNC)&SST25WriteWord)
#define NVMRead             ((NVM_READ_FUNC)&SST25ReadWord)
#define FlashInit(pInitData) SST25Init((DRV_SPI_INIT_DATA*)pInitData)

/*********************************************************************
 * IOs for Serial Flash
 **********************************************************************/
#define USE_M25P80
#define SPI_CHANNEL_1_ENABLE
#define SST25_CS_TRIS   _TRISA9
#define SST25_CS_LAT    _LATA9

#define ERASE_SECTOR_SIZE   65536
#define SPI_FLASH_CONFIG    { 1, 3, 6, 0, 1, 1, 0}

/*********************************************************************
 * IOs for the micro SD card interface
 *********************************************************************/

#define USE_SD_INTERFACE_WITH_SPI

    // Registers for the SPI module 
    #define MDD_USE_SPI_1

    // MDD SPI Configuration

    // Description: SD-SPI Chip Select Output bit
    #define SD_CS               _LATA6
    // Description: SD-SPI Chip Select TRIS bit
    #define SD_CS_TRIS          _TRISA6

    // Description: SD-SPI Card Detect Input bit
    #define SD_CD               _RA7
    // Description: SD-SPI Card Detect TRIS bit
    #define SD_CD_TRIS          _TRISA7

    // Description: SD-SPI Write Protect Check Input bit
    #define SD_WE               0       // not used
    // Description: SD-SPI Write Protect Check TRIS bit
    #define SD_WE_TRIS          _TRISA7 // not used, reassigned as CD

    // Description: The main SPI control register
    #define SPICON1             SPI1CON
    // Description: The SPI status register
    #define SPISTAT             SPI1STAT
    // Description: The SPI Buffer
    #define SPIBUF              SPI1BUF
    // Description: The receive buffer full bit in the SPI status register
    #define SPISTAT_RBF         SPI1STATbits.SPIRBF
    // Description: The bitwise define for the SPI control register (i.e. _____bits)
    #define SPICON1bits         SPI1CONbits
    // Description: The bitwise define for the SPI status register (i.e. _____bits)
    #define SPISTATbits         SPI1STATbits
    // Description: The enable bit for the SPI module
    #define SPIENABLE           SPI1CONbits.ON
    // Description: The definition for the SPI baud rate generator register (PIC32)
    #define SPIBRG		SPI1BRG

    // Tris pins for SCK/SDI/SDO lines
    // Description: The TRIS bit for the SCK pin
    #define SPICLOCK            _TRISD10
    // Description: The TRIS bit for the SDI pin
    #define SPIIN               _TRISD0
    // Description: The TRIS bit for the SDO pin
    #define SPIOUT              _TRISC4


    #define SPI_START_CFG_1     (PRI_PRESCAL_64_1 | SEC_PRESCAL_8_1 | MASTER_ENABLE_ON | SPI_CKE_ON | SPI_SMP_ON)
    #define SPI_START_CFG_2     (SPI_ENABLE)

    // Define the SPI frequency
    #define SPI_FREQUENCY	(20000000)

    //SPI library functions
    #define putcSPI             putcSPI1
    #define getcSPI             getcSPI1
    #define OpenSPI(config1, config2)   OpenSPI1(config1, config2)

    // Will generate an error if the clock speed is too low to interface to the card
    #if (GetSystemClock() < 100000)
        #error Clock speed must exceed 100 kHz
    #endif


/*********************************************************************
* IOS FOR THE VS1053 MP3 decoder
*********************************************************************/
// SPI2 is shared with uSD card in "VS1002 native mode"

#define MP3_RST_Config()        _TRISG12 = 0     // o reset decoder
#define MP3_RST_Enable()        _LATG12 = 0
#define MP3_RST_Disable()       _LATG12 = 1

#define MP3_DREQ                _RG14            // i request for data

#define MP3_DCS_Config()        _TRISG13 = 0     // o data select
#define MP3_DCS_Enable()        _LATG13 = 0
#define MP3_DCS_Disable()       _LATG13 = 1

#define MP3_CS_Config()         _TRISA5 = 0      // o command select
#define MP3_CS_Enable()         _LATA5 = 0
#define MP3_CS_Disable()        _LATA5 = 1

#define MP3_START_CFG_1     (PRI_PRESCAL_64_1 | SEC_PRESCAL_8_1 | MASTER_ENABLE_ON | SPI_CKE_ON | SPI_SMP_ON)
#define MP3_START_CFG_2     (SPI_ENABLE)
#define MP3_FAST_CFG_1      (PRI_PRESCAL_1_1  | SEC_PRESCAL_8_1 | MASTER_ENABLE_ON | SPI_CKE_ON | SPI_SMP_ON)

/*********************************************************************
 * USB configuration: self powered
 *********************************************************************/
#define self_power          1   // return always 1
#define USB_BUS_SENSE       1   // return always 1


/*******************************************************************
 * MDD File System selection options
 *******************************************************************/
//    #define USE_INTERNAL_FLASH

    #define ERASE_BLOCK_SIZE        1024
    #define WRITE_BLOCK_SIZE        128

#endif // HARDWARE_PROFILE_H
