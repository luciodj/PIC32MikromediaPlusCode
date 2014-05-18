/*
 * File: PICconfig.h
 *
 * PIC32 Mikromedia Plus default microcontroller configuration
 */
#include <xc.h>

// this is the default configuration for all book's projects 
// configuration bit settings
#pragma config POSCMOD=HS, FNOSC=PRIPLL
#pragma config FPLLIDIV=DIV_4, FPLLMUL=MUL_20, FPLLODIV=DIV_1
#pragma config FPBDIV=DIV_2
#pragma config FWDTEN=OFF, CP=OFF, BWP=OFF

// can be commented in all projects that don't make use of the USB module
#pragma config UPLLIDIV=DIV_4, UPLLEN=ON

// Select RMII Ethernet interface with Alternate pin-out
#pragma config FMIIEN = OFF             // Ethernet RMII/MII Enable (RMII Enabled)
#pragma config FETHIO = OFF             // Ethernet I/O Pin Select (Alternate Ethernet I/O)
