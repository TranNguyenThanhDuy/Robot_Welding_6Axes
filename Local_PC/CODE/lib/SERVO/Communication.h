#ifndef _COMM_H_
#define _COMM_H_
//===========================================================================
//	File Name	: Communication.h
//	Description : Sending/Receiving Protocol Functions
//===========================================================================

//===========================================================================
//	Include Files
//===========================================================================

// Serial Communication Functions
#include "Communication_Setting.h"
//#include "Communication_Setting_AVR128.h"

//===========================================================================
//	Communication Functions
//===========================================================================

//------------------------------------------------------------------
//                 Timer Function which have to call in INT.
//------------------------------------------------------------------
void CommTimer_Tick();

//------------------------------------------------------------------
//                 Timer Functions which CommInterface use.
//------------------------------------------------------------------
void CommTimer_Reset(unsigned int nTime);
int  CommTimer_IsExpired();

//------------------------------------------------------------------
//                 Protocol Functions which CommInterface use.
//------------------------------------------------------------------
int  FuncCommand(unsigned char iSlaveNo, unsigned char byCmd, unsigned char* lpIN, unsigned short dwINlen, unsigned char* lpOUT, unsigned short dwOUTlen, unsigned short dwWaitTime, int bTimeOutReaction);
int  FuncCommandNoResp(unsigned char iSlaveNo, unsigned char byCmd, unsigned char* lpIN, unsigned short dwINlen);
int  FuncCommandRespNoDebug(unsigned char iSlaveNo, unsigned char byCmd, unsigned char* lpIN, unsigned short dwINlen, unsigned char* lpOUT, unsigned short dwOUTlen, unsigned short dwWaitTime, int bTimeOutReaction);

#define	DoSendCommand		FuncCommand        //Full Response - Full Debug (Use for Debug)
#define	DoSendCommandNoResp	FuncCommandNoResp  //No Response - No Debug (Use for Motion Func)
#define	DoSendCommandRespNoDebug	FuncCommandRespNoDebug //Full Response - No Debug (Use for Get Parameter and Value Func)

#endif	// _COMM_H_
