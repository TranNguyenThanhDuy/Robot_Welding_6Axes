//////////////////////////////////////////////////////////////////////////
//
//	File:			FAS_EziMOTIONPlusR_V8.h
//	Last Modified Date:	24.08.30
//	Last Modified Ver:	1.0.1
//
//////////////////////////////////////////////////////////////////////////

#pragma once

#include "Linux_Type_Define.h"

//------------------------------------------------------------------------------
//					IO Functions
//------------------------------------------------------------------------------
 int 	FAS_SetIOInput_V8(unsigned char nPortID, unsigned char iSlaveNo, ULONGLONG uIOSETMask, ULONGLONG uIOCLRMask);
 int 	FAS_SetIOInputEx_V8(unsigned char nPortID, unsigned char iSlaveNo, unsigned int dwIOSETMaskLow, unsigned int dwIOSETMaskHigh, unsigned int dwIOCLRMaskLow, unsigned int dwIOCLRMaskHigh);
 int 	FAS_GetIOInput_V8(unsigned char nPortID, unsigned char iSlaveNo, ULONGLONG* uIOInput);
 int 	FAS_GetIOInputEx_V8(unsigned char nPortID, unsigned char iSlaveNo, unsigned int* dwIOInputLow, unsigned int* dwIOInputHigh);

 int 	FAS_GetInputAssignMap_V8(unsigned char nPortID, unsigned char iSlaveNo, unsigned char iInPinNo, unsigned char* nInLogicNo, unsigned char* bLevel);
 int 	FAS_GetOutputAssignMap_V8(unsigned char nPortID, unsigned char iSlaveNo, unsigned char iOutPinNo, unsigned char* nOutLogicNo, unsigned char* bLevel);
 int 	FAS_SetInputAssignMap_V8(unsigned char nPortID, unsigned char iSlaveNo, unsigned char iInPinNo, unsigned char nInLogicNo, unsigned char bLevel);
 int 	FAS_SetOutputAssignMap_V8(unsigned char nPortID, unsigned char iSlaveNo, unsigned char iOutPinNo, unsigned char nOutLogicNo, unsigned char bLevel);

//------------------------------------------------------------------------------
//					Read Status and Position
//------------------------------------------------------------------------------
 int 	FAS_GetIOStatus_V8(unsigned char nPortID, unsigned char iSlaveNo, ULONGLONG* uInStatus, unsigned int* dwOutStatus);
 int 	FAS_GetIOStatusEx_V8(unsigned char nPortID, unsigned char iSlaveNo, unsigned int* dwInStatusLow, unsigned int* dwInStatusHigh, unsigned int* dwOutStatus);
 int 	FAS_GetMotionStatus_V8(unsigned char nPortID, unsigned char iSlaveNo, int* lCmdPos, int* lActPos, int* lPosErr, int* lActVel, unsigned int* dwAxisStatus);
 int 	FAS_GetAllStatus_V8(unsigned char nPortID, unsigned char iSlaveNo, ULONGLONG* uInStatus, unsigned int* dwOutStatus, unsigned int* dwAxisStatus, int* lCmdPos, int* lActPos, int* lPosErr, int* lActVel, unsigned short* wPosItemNo);
 int 	FAS_GetAllStatusEx_V8(unsigned char nPortID, unsigned char iSlaveNo, unsigned int* dwInStatusLow, unsigned int* dwInStatusHigh, unsigned int* dwOutStatus, unsigned int* dwAxisStatus, int* lCmdPos, int* lActPos, int* lPosErr, int* lActVel, unsigned short* wPosItemNo);

 int 	FAS_GetAllABSStatus_V8(unsigned char nPortID, unsigned char iSlaveNo, ULONGLONG* uInStatus, unsigned int* dwOutStatus, unsigned int* dwAxisStatus, int* lCmdPos, int* lActPos, int* lPosErr, int* lActVel, unsigned short* wCurrentPosItemNo);

 int 	FAS_GetRunPTStatus_V8(unsigned char nPortID, unsigned char iSlaveNo, unsigned short* wPosItemNo);
