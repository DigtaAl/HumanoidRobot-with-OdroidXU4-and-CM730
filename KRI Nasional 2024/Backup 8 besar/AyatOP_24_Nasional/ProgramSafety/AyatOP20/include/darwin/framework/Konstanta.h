/*
 * VoidAction.h
 *
 *  Created on: 2024. 4. 20.
 *      Author: Digta
 */

#ifndef KONSTANTA_H
#define KONSTANTA_H

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../Data/config.ini"
#define SCRIPT_FILE_PATH    "script.asc"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"
#define U2D_DEV_CM    	    "/dev/USBCM"
#define U2D_DEV_Interface   "/dev/USBInterface"

//============= State ==================                                                // Buffer 9
const char Initial  = 0;
const char Ready    = 1;
const char Set      = 2;
const char Play     = 3;
const char Finish   = 4;

//============= STATUS KOMUNIKASI ==================       
const char STAT_NETRAL		= 0;
const char STAT_ROBOT1		= 1;
const char STAT_ROBOT2		= 2;
const char STAT_ROBOT3     	= 3;
const char STAT_ROBOT4   	= 4;
const char STAT_KICKOFF   	= 10;

#endif // KONSTANTA_H
