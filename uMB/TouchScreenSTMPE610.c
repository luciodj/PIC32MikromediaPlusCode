/*****************************************************************************
 *
 * Access to 4 wire resistive touch screen via STMPE610 controller
 *
 *****************************************************************************
 * FileName:        TouchScreenSTMPE610.c
 * Processor:       PIC32
 * Compiler:        MPLAB XC32
 *****************************************************************************/

#include "HardwareProfile.h"

#if defined (USE_TOUCHSCREEN_STMPE610)

#include "Graphics/Graphics.h"
#include "TimeDelay.h"
#include "TouchScreen.h"
#include "TouchScreenSTMPE610.h"
#include "Compiler.h"

#define USE_AND_OR

#include <plib.h>           // i2c lib

#define TP_ADDRESS          0x88 // default address
#define TP_BAUD_400kHz      100  // Assuming 40MHz FPb clock
#define TP_DEVID            0x00 // devid register
#define TP_SYS1             0x03 // reset control
#define TP_SYS2             0x04 // clock control
#define TP_INT_CTRL         0x09 // interrupt control
#define TP_INT_EN           0x0A // interrupt enable
#define TP_INT_STA          0x0B // interrupt status
#define TP_ADC_CTRL1        0x20 // adc control
#define TP_ADC_CTRL2        0x21
#define TP_TSC_CTRL         0x40 // touch screen control
#define TP_TSC_CFG          0x41 // touch screen configuration
#define TP_WDW_TR_X         0x42 // window control
#define TP_WDW_TR_Y         0x44
#define TP_WDW_BL_X         0x46
#define TP_WDW_BL_Y         0x48
#define TP_FIFO_TH          0x4A // FIFO control
#define TP_FIFO_STA         0x4B
#define TP_FIFO_SIZE        0x4C
#define TP_TSC_DATA_X       0x4D // touch data
#define TP_TSC_DATA_Y       0x4F
#define TP_TSC_DATA_Z       0x51
#define TP_FRACTION_Z       0x56
#define TP_TSC_DATA_XYZ     0x57


// Default Calibration Inset Value (percentage of vertical or horizontal resolution)
// Calibration Inset = ( CALIBRATIONINSET / 2 ) % , Range of 0–20% with 0.5% resolution
// Example with CALIBRATIONINSET == 20, the calibration points are measured
// 10% from the corners.
#ifndef CALIBRATIONINSET
    #define CALIBRATIONINSET   20       // range 0 <= CALIBRATIONINSET <= 40 
#endif

#define CAL_X_INSET    (((GetMaxX()+1)*(CALIBRATIONINSET>>1))/100)
#define CAL_Y_INSET    (((GetMaxY()+1)*(CALIBRATIONINSET>>1))/100)
#define SAMPLE_POINTS   4

//////////////////////// Resistive Touch Driver Version ////////////////////////////
// The first four bits is the calibration inset, next 8 bits is assigned the version 
// number and 0xF is assigned to this 4-wire resistive driver.
const WORD mchpTouchScreenVersion = 0xF110 | CALIBRATIONINSET;


//////////////////////// GUI Color Assignments ///////////////////////
// Set the colors used in the calibration screen, defined by 
// GraphicsConfig.h or gfxcolors.h 
#if (COLOR_DEPTH == 1)
    #define RESISTIVETOUCH_FOREGROUNDCOLOR BLACK	   
    #define RESISTIVETOUCH_BACKGROUNDCOLOR WHITE	   
#elif (COLOR_DEPTH == 4)
    #define RESISTIVETOUCH_FOREGROUNDCOLOR BLACK	   
    #define RESISTIVETOUCH_BACKGROUNDCOLOR WHITE	   
#elif (COLOR_DEPTH == 8) || (COLOR_DEPTH == 16) || (COLOR_DEPTH == 24) 
    #define RESISTIVETOUCH_FOREGROUNDCOLOR BRIGHTRED	   
    #define RESISTIVETOUCH_BACKGROUNDCOLOR WHITE	   
#endif


//////////////////////// LOCAL PROTOTYPES ////////////////////////////
void    TouchGetCalPoints(void);
void 	TouchStoreCalibration(void);
void 	TouchCheckForCalibration(void);
void 	TouchLoadCalibration(void);
void    TouchCalculateCalPoints(void);

#ifdef ENABLE_DEBUG_TOUCHSCREEN
void    TouchScreenResistiveTestXY(void);
#endif

extern NVM_READ_FUNC           pCalDataRead;                // function pointer to data read
extern NVM_WRITE_FUNC          pCalDataWrite;               // function pointer to data write
extern NVM_SECTORERASE_FUNC    pCalDataSectorErase;         // function pointer to data sector erase

//////////////////////// GLOBAL VARIABLES ////////////////////////////
//#ifndef TOUCHSCREEN_RESISTIVE_PRESS_THRESHOLD
//    // you may define the threshold with a value, define the new value in the
//    // HardwareProfile.h
//    #define TOUCHSCREEN_RESISTIVE_PRESS_THRESHOLD     256	// between 0-0x03ff the lesser this value
//											                // the lighter the screen must be pressed
//#endif

#define CALIBRATION_DELAY   300				                // delay between calibration touch points

//// Current ADC values for X and Y channels
volatile SHORT  adcX = -1;
volatile SHORT  adcY = -1;

// coefficient values
volatile long   _trA;
volatile long   _trB;
volatile long   _trC;
volatile long   _trD;

// copy of the stored or sampled raw points (this is the calibration data stored)
/*      xRawTouch[0] - x sample from upper left corner; 
        xRawTouch[1] - x sample from upper right corner
        xRawTouch[2] - x sample from lower right corner
        xRawTouch[3] - x sample from lower left corner
        yRawTouch[0] - y sample from upper left corner; 
        yRawTouch[1] - y sample from upper right corner
        yRawTouch[2] - y sample from lower right corner
        yRawTouch[3] - y sample from lower left corner
*/
volatile SHORT  xRawTouch[SAMPLE_POINTS] = {TOUCHCAL_ULX, TOUCHCAL_URX, TOUCHCAL_LRX, TOUCHCAL_LLX};   
volatile SHORT  yRawTouch[SAMPLE_POINTS] = {TOUCHCAL_ULY, TOUCHCAL_URY, TOUCHCAL_LRY, TOUCHCAL_LLY};   

// WARNING: Watch out when selecting the value of SCALE_FACTOR 
// since a big value will overflow the signed int type 
// and the multiplication will yield incorrect values.
#ifndef TOUCHSCREEN_RESISTIVE_CALIBRATION_SCALE_FACTOR
    // default scale factor is 256
    #define TOUCHSCREEN_RESISTIVE_CALIBRATION_SCALE_FACTOR 8
#endif

// use this scale factor to avoid working in floating point numbers
#define SCALE_FACTOR (1<<TOUCHSCREEN_RESISTIVE_CALIBRATION_SCALE_FACTOR)

void AddressTP( BYTE add)
{
    // 1. write the device addess
    while( 1)
    {
        StartI2C2();    IdleI2C2();
        // send command and address
        MasterWriteI2C2( TP_ADDRESS);  IdleI2C2();
        // exit if received an acknowledge
        if ( I2C2STATbits.ACKSTAT == 0)
            break;
        StopI2C2();     IdleI2C2();
    } // wait until you receive an acknowledge
    // 2. send the register address
    MasterWriteI2C2( add);      IdleI2C2();
} // address TP


BYTE readTPRegister( BYTE reg)
{
    BYTE r;
    // 1. select device and register
    AddressTP( reg);
    // 2. issue a read command
    RestartI2C2();    IdleI2C2();
    r = MasterWriteI2C2( TP_ADDRESS + 1);  IdleI2C2();
    //3. get one byte of data in
    r = MasterReadI2C2();
    // 4. terminate sequence
    NotAckI2C2();
    IdleI2C2();
    StopI2C2(); IdleI2C2();
    // 5. return value read
    return r;
} // read TP Register

WORD readTPRegisterWord( BYTE reg)
{
    WORD r;
    // 1. select device and register
    AddressTP( reg);
    // 2. issue a read command
    RestartI2C2();    IdleI2C2();
    MasterWriteI2C2( TP_ADDRESS + 1);  IdleI2C2();
    //3. get one byte of data in
    r = MasterReadI2C2()<<8;
    // 4. continue read sequence
    AckI2C2(); IdleI2C2();
    r |= MasterReadI2C2();
    // 5. terminate sequence
    NotAckI2C2(); IdleI2C2();
    StopI2C2(); IdleI2C2();
    // 6. return value read
    return r;
} // read TP Register


unsigned long readTPRegister24( BYTE reg)
{
    unsigned long r;
    // 1. select device and register
    AddressTP( reg);
    // 2. issue a read command
    RestartI2C2();    IdleI2C2();
    MasterWriteI2C2( TP_ADDRESS + 1);  IdleI2C2();
    //3. get one byte of data in
    r = MasterReadI2C2();
    // 4. continue read sequence
    AckI2C2(); IdleI2C2();
    r = (r<<8) | MasterReadI2C2();
    // 5. continue read sequence
    AckI2C2(); IdleI2C2();
    r = (r<<8) | MasterReadI2C2();
    // 6. terminate sequence
    NotAckI2C2(); IdleI2C2();
    StopI2C2(); IdleI2C2();
    // 7. return value read
    return r;
} // read TP Register


void writeTPRegister( BYTE reg, BYTE b)
{
    AddressTP( reg);
    MasterWriteI2C2( b); IdleI2C2();
    StopI2C2(); IdleI2C2();
} // write TP register

void writeTPRegisterWord( BYTE reg, WORD w)
{
    AddressTP( reg);
    MasterWriteI2C2( w >> 8); IdleI2C2();
    MasterWriteI2C2( (BYTE)w); IdleI2C2();
    StopI2C2(); IdleI2C2();
} // write TP register


/*********************************************************************
* Function: void TouchDetectPosition(void)
********************************************************************/
SHORT TouchDetectPosition(void)
{
    unsigned long r;
    
    // check if Touch Screen control has a new touch event available
    r = readTPRegister( TP_TSC_CTRL);
    if ( ((WORD)r & 0x80) == 0)
//    if ( TP_INT == 0)
    {
        if (adcX >= 0)
        {
            writeTPRegister( TP_FIFO_STA, 1);   // reset FIFO
            writeTPRegister( TP_FIFO_STA, 0);   // unreset
        }

        adcX = -1;
        adcY = -1;
        return 0;      // no touch
    }

    r = readTPRegister( TP_TSC_DATA_XYZ);
    r = (r<<8) | readTPRegister( TP_TSC_DATA_XYZ);
    r = (r<<8) | readTPRegister( TP_TSC_DATA_XYZ);

    adcX = r >> 12;
    adcY = r & 0xFFF;

    if (( adcX ==0) || (adcY==0))
    {
        adcX = -1;
        adcY = -1;
    }

    writeTPRegister( TP_INT_STA, 0xFF); // clear all interrupts

    return 1;                       // touch screen acquisition is done
}

/*********************************************************************
* Function: void Touch_STMPE610Init(void)
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: Initializes the Touch controller for the touch detection.
*
* Note: none
*
********************************************************************/
SHORT Touch_STMPE610Init(void)
{
    WORD w;

    // configure I2C port for accelerometer access
    OpenI2C2( I2C_ON | I2C_7BIT_ADD | I2C_STRICT_EN, TP_BAUD_400kHz);
    DelayMs( 1);

    // 1. test Read DEVID register
    w = readTPRegisterWord( TP_DEVID);
    if ( w != 0x811)
        return -1;                      // Failed to identify

    // 2. perform SOFT_RESET
    writeTPRegister( TP_SYS1,     0x02);
    DelayMs( 5);

    writeTPRegister( TP_SYS2,     0x00);// turn on all the clocks
    writeTPRegister( TP_TSC_CTRL, 0x23);// 8 tracking, XY, TSC enabled
    writeTPRegister( TP_INT_EN,   0x01);// touch detect

    // 3. configure the ADC
    writeTPRegister( TP_ADC_CTRL1,0x48);// 80 clock, 12-bit, internal ref
    writeTPRegister( TP_ADC_CTRL2,0x03);// ADC clock = 6.5 MHz

    // 4. configure touch control
    writeTPRegister( TP_TSC_CFG,  0x64);// avg 2, 1ms delay, 5ms settling
    writeTPRegister( TP_FRACTION_Z,0x2);// fractional 6.2

    // configure FIFO
    writeTPRegister( TP_FIFO_TH,  1);   // non zero
    writeTPRegister( TP_FIFO_STA, 1);   // reset
    writeTPRegister( TP_FIFO_STA, 0);   // unreset
    
    // 4. configure interrupt
    writeTPRegister( TP_INT_STA,  0xff);// clear all interrupts
    writeTPRegister( TP_INT_CTRL, 0x05);// active high, level, global enable

    // configure window
//    writeTPRegisterWord( TP_WDW_TR_X, 0x0fff);
//    writeTPRegisterWord( TP_WDW_TR_Y, 0x0fff);
//    writeTPRegisterWord( TP_WDW_BL_X, 0);
//    writeTPRegisterWord( TP_WDW_BL_Y, 0);


    return 0;                           // Success
}

/*********************************************************************
* Function: void TouchHardwareInit(void)
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: Initializes touch screen module.
*
* Note: none
*
********************************************************************/
void TouchHardwareInit(void *initValues)
{
    
    TP_INT_TRIS = 1;                // ensure INT pin is an input
    Touch_STMPE610Init();           // intialize the controller

}

/*********************************************************************
* Function: SHORT TouchGetX()
*
* PreCondition: none
*
* Input: none
*
* Output: x coordinate
*
* Side Effects: none
*
* Overview: returns x coordinate if touch screen is pressed
*           and -1 if not
*
* Note: none
*
********************************************************************/
SHORT TouchGetX(void)
{
    long    result;

    result = TouchGetRawX();

    if(result >= 0)
    {
        result = (long)((((long)_trC*result) + _trD)>>TOUCHSCREEN_RESISTIVE_CALIBRATION_SCALE_FACTOR);

#ifdef TOUCHSCREEN_RESISTIVE_FLIP_X
        result = GetMaxX() - result;
#endif	
    }
    return ((SHORT)result);
}

/*********************************************************************
* Function: SHORT TouchGetRawX()
*
* PreCondition: none
*
* Input: none
*
* Output: x coordinate
*
* Side Effects: none
*
* Overview: returns x coordinate if touch screen is pressed
*           and -1 if not
*
* Note: none
*
********************************************************************/
SHORT TouchGetRawX(void)
{
//    SHORT r;

#ifdef TOUCHSCREEN_RESISTIVE_SWAP_XY
    return adcY;    
#else
    return adcX>>2;
#endif
//    writeTPRegister( TP_INT_STA, 0xFF); // clear all interrupts
//    return r;
}

/*********************************************************************
* Function: SHORT TouchGetY()
*
* PreCondition: none
*
* Input: none
*
* Output: y coordinate
*
* Side Effects: none
*
* Overview: returns y coordinate if touch screen is pressed
*           and -1 if not
*
* Note: none
*
********************************************************************/
SHORT TouchGetY(void)
{

    long    result;

    result = TouchGetRawY();
    
    if(result >= 0)
    {
        result = (long)((((long)_trA*result) + (long)_trB)>>TOUCHSCREEN_RESISTIVE_CALIBRATION_SCALE_FACTOR);

		#ifdef TOUCHSCREEN_RESISTIVE_FLIP_Y
			result = GetMaxY() - result;
		#endif	
	}
    return ((SHORT)result);
}

/*********************************************************************
* Function: SHORT TouchGetRawY()
*
* PreCondition: none
*
* Input: none
*
* Output: y coordinate
*
* Side Effects: none
*
* Overview: returns y coordinate if touch screen is pressed
*           and -1 if not
*
* Note: none
*
********************************************************************/
SHORT TouchGetRawY(void)
{
//    SHORT r;

    // check if Touch Screen control has a new touch event available
//    if ( TP_INT == 0)
//        return -1;      // no touch

#ifndef TOUCHSCREEN_RESISTIVE_SWAP_XY
    return adcY>>2;
#else
    return adcX>>2;
#endif
//    writeTPRegister( TP_INT_STA, 0xFF); // clear all interrupts
//    return r;
}

/*********************************************************************
* Function: void TouchStoreCalibration(void)
*
* PreCondition: Non-volatile memory initialization function must be called before
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: stores calibration parameters into non-volatile memory
*
* Note: none
*
********************************************************************/
void TouchStoreCalibration(void)
{
    if (pCalDataWrite != NULL)
    {
        // the upper left X sample address is used since it is the first one
        // and this assumes that all stored values are located in one
        // sector
        if (pCalDataSectorErase != NULL)
                pCalDataSectorErase(ADDRESS_RESISTIVE_TOUCH_ULX);

        pCalDataWrite(xRawTouch[0], ADDRESS_RESISTIVE_TOUCH_ULX);
        pCalDataWrite(yRawTouch[0], ADDRESS_RESISTIVE_TOUCH_ULY);

        pCalDataWrite(xRawTouch[1], ADDRESS_RESISTIVE_TOUCH_URX);
        pCalDataWrite(yRawTouch[1], ADDRESS_RESISTIVE_TOUCH_URY);

        pCalDataWrite(xRawTouch[3], ADDRESS_RESISTIVE_TOUCH_LLX);
        pCalDataWrite(yRawTouch[3], ADDRESS_RESISTIVE_TOUCH_LLY);

        pCalDataWrite(xRawTouch[2], ADDRESS_RESISTIVE_TOUCH_LRX);
        pCalDataWrite(yRawTouch[2], ADDRESS_RESISTIVE_TOUCH_LRY);

        pCalDataWrite(mchpTouchScreenVersion, ADDRESS_RESISTIVE_TOUCH_VERSION);
    }
}


/*********************************************************************
* Function: void TouchLoadCalibration(void)
*
* PreCondition: Non-volatile memory initialization function must be called before
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: loads calibration parameters from non-volatile memory
*
* Note: none
*
********************************************************************/
void TouchLoadCalibration(void)
{

    if (pCalDataRead != NULL)
    {

        xRawTouch[0] = pCalDataRead(ADDRESS_RESISTIVE_TOUCH_ULX);
        yRawTouch[0] = pCalDataRead(ADDRESS_RESISTIVE_TOUCH_ULY);

        xRawTouch[1] = pCalDataRead(ADDRESS_RESISTIVE_TOUCH_URX);
        yRawTouch[1] = pCalDataRead(ADDRESS_RESISTIVE_TOUCH_URY);

        xRawTouch[3] = pCalDataRead(ADDRESS_RESISTIVE_TOUCH_LLX);
        yRawTouch[3] = pCalDataRead(ADDRESS_RESISTIVE_TOUCH_LLY);

        xRawTouch[2] = pCalDataRead(ADDRESS_RESISTIVE_TOUCH_LRX);
        yRawTouch[2] = pCalDataRead(ADDRESS_RESISTIVE_TOUCH_LRY);

    }

    TouchCalculateCalPoints();
    
}

/*********************************************************************
* Function: void TouchGetCalPoints(void)
*
* PreCondition: InitGraph() must be called before
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: gets values for 3 touches
*
* Note: none
*
********************************************************************/
//void TouchCalculateCalPoints(WORD *xRawTouch, WORD *yRawTouch, WORD *xPoint, WORD *yPoint)
void TouchCalculateCalPoints(void)
{
    long trA, trB, trC, trD;                    // variables for the coefficients
    long trAhold, trBhold, trChold, trDhold;
    long test1, test2;                          // temp variables (must be signed type)

    SHORT    xPoint[SAMPLE_POINTS], yPoint[SAMPLE_POINTS];

    yPoint[0] = yPoint[1] = CAL_Y_INSET; 
    yPoint[2] = yPoint[3] = (GetMaxY() - CAL_Y_INSET);
    xPoint[0] = xPoint[3] = CAL_X_INSET;
    xPoint[1] = xPoint[2] = (GetMaxX() - CAL_X_INSET);

    // calculate points transfer functions
    // based on two simultaneous equations solve for the constants

    // use sample points 1 and 4
    // Dy1 = aTy1 + b; Dy4 = aTy4 + b
    // Dx1 = cTx1 + d; Dy4 = aTy4 + b

    test1 = (long)yPoint[0] - (long)yPoint[3];
    test2 = (long)yRawTouch[0] - (long)yRawTouch[3];

    trA = ((long)((long)test1 * SCALE_FACTOR) / test2);
    trB = ((long)((long)yPoint[0] * SCALE_FACTOR) - (trA * (long)yRawTouch[0]));

    test1 = (long)xPoint[0] - (long)xPoint[2];
    test2 = (long)xRawTouch[0] - (long)xRawTouch[2];

    trC = ((long)((long)test1 * SCALE_FACTOR) / test2);
    trD = ((long)((long)xPoint[0] * SCALE_FACTOR) - (trC * (long)xRawTouch[0]));

    trAhold = trA;
    trBhold = trB;
    trChold = trC;
    trDhold = trD;

    // use sample points 2 and 3
    // Dy2 = aTy2 + b; Dy3 = aTy3 + b
    // Dx2 = cTx2 + d; Dy3 = aTy3 + b

    test1 = (long)yPoint[1] - (long)yPoint[2];
    test2 = (long)yRawTouch[1] - (long)yRawTouch[2];

    trA = ((long)(test1 * SCALE_FACTOR) / test2);
    trB = ((long)((long)yPoint[1] * SCALE_FACTOR) - (trA * (long)yRawTouch[1]));

    test1 = (long)xPoint[1] - (long)xPoint[3];
    test2 = (long)xRawTouch[1] - (long)xRawTouch[3];

    trC = ((long)((long)test1 * SCALE_FACTOR) / test2);
    trD = ((long)((long)xPoint[1] * SCALE_FACTOR) - (trC * (long)xRawTouch[1]));

    // get the average and use the average
    _trA = (trA + trAhold) >> 1;
    _trB = (trB + trBhold) >> 1;
    _trC = (trC + trChold) >> 1;
    _trD = (trD + trDhold) >> 1;

}

/*********************************************************************
* Function: void TouchGetCalPoints(void)
*
* PreCondition: InitGraph() must be called before
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: gets values for 3 touches
*
* Note: none
*
********************************************************************/
void TouchCalHWGetPoints(void)
{
    #define TOUCH_DIAMETER	10
    #define SAMPLE_POINTS   4

    XCHAR   calStr1[] = {'o','n',' ','t','h','e',' ','f','i','l','l','e','d',0};
    XCHAR   calStr2[] = {'c','i','r','c','l','e',0};
    XCHAR  	calTouchPress[] = {'P','r','e','s','s',' ','&',' ','R','e','l','e','a','s','e',0};

    XCHAR   calRetryPress[] = {'R','e','t','r','y',0};
    XCHAR   *pMsgPointer;
    SHORT   counter;

    WORD    dx[SAMPLE_POINTS], dy[SAMPLE_POINTS];
    WORD    textHeight, msgX, msgY;
    SHORT   tempX, tempY;

    SetFont((void *) &FONTDEFAULT);
    SetColor(RESISTIVETOUCH_FOREGROUNDCOLOR);

    textHeight = GetTextHeight((void *) &FONTDEFAULT);

    while
    (
        !OutTextXY
        (
            (GetMaxX() - GetTextWidth((XCHAR *)calStr1, (void *) &FONTDEFAULT)) >> 1,
            (GetMaxY() >> 1),
            (XCHAR *)calStr1
        )
    );

    while
    (
        !OutTextXY
        (
            (GetMaxX() - GetTextWidth((XCHAR *)calStr2, (void *) &FONTDEFAULT)) >> 1,
            ((GetMaxY() >> 1) + textHeight),
            (XCHAR *)calStr2
        )
    );

    // calculate center points (locate them at 15% away from the corners)
	// draw the four touch points

    dy[0] = dy[1] = CAL_Y_INSET; 
    dy[2] = dy[3] = (GetMaxY() - CAL_Y_INSET);
    dx[0] = dx[3] = CAL_X_INSET;
    dx[1] = dx[2] = (GetMaxX() - CAL_X_INSET);


    msgY = ((GetMaxY() >> 1) - textHeight);
    pMsgPointer = calTouchPress;
	
    // get the four samples or calibration points
    for(counter = 0; counter < SAMPLE_POINTS;)
    {
    
        // redraw the filled circle to unfilled (previous calibration point)
        if (counter > 0)
        {
            SetColor(RESISTIVETOUCH_BACKGROUNDCOLOR);
            while(!(FillCircle(dx[counter-1], dy[counter-1], TOUCH_DIAMETER-3)));
        }

        // draw the new filled circle (new calibration point)
        SetColor(RESISTIVETOUCH_FOREGROUNDCOLOR);
        while(!(Circle(dx[counter], dy[counter], TOUCH_DIAMETER)));
        while(!(FillCircle(dx[counter], dy[counter], TOUCH_DIAMETER-3)));

        // show points left message
        msgX = (GetMaxX() - GetTextWidth((XCHAR *)pMsgPointer, (void *) &FONTDEFAULT)) >> 1;
        TouchShowMessage(pMsgPointer, RESISTIVETOUCH_FOREGROUNDCOLOR, msgX, msgY);

        // Wait for press
        do {} 
    	    while((TouchGetRawX() == -1) && (TouchGetRawY() == -1));

        tempX = TouchGetRawX();
        tempY = TouchGetRawY();

        // wait for release
        do {}
            while((TouchGetRawX() != -1) && (TouchGetRawY() != -1));

        // check if the touch was detected properly
        if ((tempX == -1) || (tempY == -1))
        {   
            // cannot proceed retry the touch, display RETRY PRESS message 

            // remove the previous string
            TouchShowMessage(pMsgPointer, RESISTIVETOUCH_BACKGROUNDCOLOR, msgX, msgY);
            pMsgPointer = calRetryPress;
            // show the retry message
            msgX = (GetMaxX() - GetTextWidth((XCHAR *)pMsgPointer, (void *) &FONTDEFAULT)) >> 1;
            TouchShowMessage(pMsgPointer, RESISTIVETOUCH_FOREGROUNDCOLOR, msgX, msgY);
        }
        else
        {    

            // remove the previous string
            TouchShowMessage(pMsgPointer, RESISTIVETOUCH_BACKGROUNDCOLOR, msgX, msgY);
            pMsgPointer = calTouchPress;



            #ifdef TOUCHSCREEN_RESISTIVE_FLIP_Y
                yRawTouch[3 - counter] = tempY; //TouchGetRawY();
            #else
                yRawTouch[counter] = tempY; //ouchGetRawY();
            #endif
    
            #ifdef TOUCHSCREEN_RESISTIVE_FLIP_X
            xRawTouch[3 - counter] = tempX; //TouchGetRawX();
    		#else
            xRawTouch[counter] = tempX; //TouchGetRawX();
            #endif

            counter++;
        }

        // Wait for release
        do {} 
            while((TouchGetRawX() != -1) && (TouchGetRawY() != -1));

        DelayMs(CALIBRATION_DELAY);
      
    }

    TouchCalculateCalPoints();
 
    #ifdef ENABLE_DEBUG_TOUCHSCREEN
        TouchScreenResistiveTestXY();
    #endif
}


/*********************************************************************
* Function: void TouchScreenResistiveTestXY(void)
*
* PreCondition: TouchHardwareInit has been called
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
* Overview: prints raw x,y calibrated x,y and calibration factors to screen
*
* Note: modify pre-processor macro to include/exclude this test code
*       a common place to call this from is at the end of TouchCalHWGetPoints()
*
********************************************************************/
#ifdef ENABLE_DEBUG_TOUCHSCREEN
#include <stdio.h>
void TouchScreenResistiveTestXY(void)
{
    #define BUFFCHARLEN 60
    char buffChar[BUFFCHARLEN];
    WORD buffCharW[BUFFCHARLEN];
    unsigned char i;
    SHORT tempXX, tempYY,tempXX2,tempYY2, calXX, calYY;
    tempXX = tempYY = -1;
    tempXX2 = tempYY2 = 0;

    // store the last calibration
    TouchStoreCalibration();

    while(1)
    {
       
        // use this to always show the values even if not pressing the screen
//        tempXX = TouchGetRawX(); 
//        tempYY = TouchGetRawY();
        
        calXX = TouchGetX();
        calYY = TouchGetY();
        
        if((tempXX != tempXX2)||(tempYY != tempYY2))
        {
            SetColor(RESISTIVETOUCH_BACKGROUNDCOLOR);
            ClearDevice();
            SetColor(RESISTIVETOUCH_FOREGROUNDCOLOR);
            sprintf(buffChar,"raw_x=%d, raw_y=%d",(WORD)tempXX, (WORD)tempYY);
			
            #ifdef USE_MULTIBYTECHAR
              for(i = 0; i < BUFFCHARLEN;i++)
              {
                buffCharW[i] = buffChar[i];
              }
              while(!OutTextXY(0,0,(XCHAR*)buffCharW));
            #else
              while(!OutTextXY(0,0,(XCHAR*)buffChar));
            #endif
            
            sprintf(buffChar,"cal_x=%d, cal_y=%d",(WORD)calXX, (WORD)calYY);
            #ifdef USE_MULTIBYTECHAR
              for(i = 0; i < BUFFCHARLEN;i++)
              {
                buffCharW[i] = buffChar[i];
              }
              while(!OutTextXY(0,40,(XCHAR*)buffCharW));
            #else
              while(!OutTextXY(0,40,(XCHAR*)buffChar));
            #endif
            
            sprintf(buffChar,"_tr:A=%d,B=%d",(WORD)_trA,(WORD)_trB);
            #ifdef USE_MULTIBYTECHAR
              for(i = 0; i < BUFFCHARLEN;i++)
              {
                buffCharW[i] = buffChar[i];
              }
              while(!OutTextXY(0,80,(XCHAR*)buffCharW));
            #else
              while(!OutTextXY(0,80,(XCHAR*)buffChar));
            #endif

            sprintf(buffChar,"_tr:C=%d,D=%d",(WORD)_trC,(WORD)_trD);
            #ifdef USE_MULTIBYTECHAR
              for(i = 0; i < BUFFCHARLEN;i++)
              {
                buffCharW[i] = buffChar[i];
              }
              while(!OutTextXY(0,100,(XCHAR*)buffCharW));
            #else
              while(!OutTextXY(0,100,(XCHAR*)buffChar));
            #endif


        }
        
        tempXX2=tempXX;
        tempYY2=tempYY;

        do{tempXX = TouchGetRawX(); tempYY = TouchGetRawY();}
    	while((tempXX == -1) && (tempYY == -1));   
    }
}
#endif //#ifdef ENABLE_DEBUG_TOUCHSCREEN


#endif // #if defined (USE_TOUCHSCREEN_STMPE610)

