/*****************************************************************************
 * Access to 4 wire resistive touch screen via STMPE610 controller
 *****************************************************************************
 * FileName:        TouchScreenSTMPE610.h
 * Processor:       PIC32
 * Compiler:        MPLAB XC32
 *****************************************************************************/

/*****************************************************************************
 Description:  This is a resistive touch screen driver that is using the 
			   Microchip Graphics Library. The calibration values are 
			   automatically checked (by reading a specific memory location
			   on the non-volatile memory) when initializing the module if the 
			   function pointers to the read and write callback functions 
			   are initialized. If the read value is invalid calibration 
			   will automatically be executed. Otherwise, the calibration
			   values will be loaded and used.
			   The driver assumes that the application side provides the 
			   read and write routines to a non-volatile memory. 
			   If the callback functions are not initialized, the calibration
			   routine will always be called at startup to initialize the
			   global calibration values.
			   This driver assumes that the Graphics Library is initialized 
			   and will be using the default font of the library.
 *****************************************************************************/

#ifndef _TOUCHSCREEN_STMPE610_H
#define _TOUCHSCREEN_STMPE610_H

#include "GenericTypeDefs.h"

// Default calibration points
#define TOUCHCAL_ULX 900
#define TOUCHCAL_ULY 150 //0xE67E
#define TOUCHCAL_URX 150 //0x2A
#define TOUCHCAL_URY 150 //0xE67E
#define TOUCHCAL_LLX 900 //0x2A
#define TOUCHCAL_LLY 900 //0xE67E
#define TOUCHCAL_LRX 150 //0x2A
#define TOUCHCAL_LRY 900 //0xE67E


// use this macro to debug the touch screen panel 
// this will enable the use of debugging functions in the C file.
// It assumes that the graphics portion is working.
//#define ENABLE_DEBUG_TOUCHSCREEN


#endif //_TOUCHSCREEN_STMPE610_H
