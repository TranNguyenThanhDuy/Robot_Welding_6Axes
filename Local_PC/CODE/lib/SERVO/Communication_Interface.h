#ifndef _COMMINTERFACE_H_
#define _COMMINTERFACE_H_
//===========================================================================
//	File Name	: Communication_Interface.h
//	Description : Plus-R Command Functions.
//===========================================================================

//===========================================================================
//	Include Files
//===========================================================================

#include "Motion_Define.h"
#include "Communication.h"

//===========================================================================
//	Plus-R Command Functions
//===========================================================================

//------------------------------------------------------------------------------
//					No-Motion Command Functions
//------------------------------------------------------------------------------
int SERVO_GetSlaveInfo(unsigned char iSlaveNo, unsigned char* pType, char* lpBuff, int nBuffSize);
int SERVO_GetMotorInfo(unsigned char iSlaveNo, unsigned char* pType, char* lpBuff, int nBuffSize);
int SERVO_GetFirmwareInfo(unsigned char iSlaveNo, unsigned char* pType);
int SERVO_Ack(unsigned char iSlaveNo, unsigned short dwWaitTime);

//------------------------------------------------------------------------------
//					Parameter Functions
//------------------------------------------------------------------------------
int SERVO_SaveAllParameter(unsigned char iSlaveNo);
int SERVO_SetParameter(unsigned char iSlaveNo, unsigned char iParaNo, long lParaValue);
int SERVO_GetParameter(unsigned char iSlaveNo, unsigned char iParaNo, long* lParamValue);
int SERVO_GetROMParameter(unsigned char iSlaveNo, unsigned char iParaNo, long* lROMParam);

//------------------------------------------------------------------------------
//					IO Functions
//------------------------------------------------------------------------------
int SERVO_SetIOInput(unsigned char iSlaveNo, unsigned long dwIOSETMask, unsigned long dwIOCLRMask);
int SERVO_GetIOInput(unsigned char iSlaveNo, unsigned long* dwIOInput);

int SERVO_SetIOOutput(unsigned char iSlaveNo, unsigned long dwIOSETMask, unsigned long dwIOCLRMask);
int SERVO_GetIOOutput(unsigned char iSlaveNo, unsigned long* dwIOOutput);

int SERVO_GetIOAssignMap(unsigned char iSlaveNo, unsigned char iIOPinNo, unsigned long* dwIOLogicMask, unsigned char* bLevel);
int SERVO_SetIOAssignMap(unsigned char iSlaveNo, unsigned char iIOPinNo, unsigned long dwIOLogicMask, unsigned char bLevel);

int SERVO_IOAssignMapReadROM(unsigned char iSlaveNo);

//------------------------------------------------------------------------------
//					Servo Driver Control Functions
//------------------------------------------------------------------------------		
int SERVO_ServoEnable(unsigned char iSlaveNo, int bOnOff);
int SERVO_ServoAlarmReset(unsigned char iSlaveNo);
int SERVO_StepAlarmReset(unsigned char iSlaveNo, int bReset);

//------------------------------------------------------------------------------
//					Read Status and Position
//------------------------------------------------------------------------------
int SERVO_GetAxisStatus(unsigned char iSlaveNo, unsigned long* dwAxisStatus);
int SERVO_GetIOAxisStatus(unsigned char iSlaveNo, unsigned long* dwInStatus, unsigned long* dwOutStatus, unsigned long* dwAxisStatus);
int SERVO_GetMotionStatus(unsigned char iSlaveNo, long* lCmdPos, long* lActPos, long* lPosErr, long* lActVel, unsigned short* wPosItemNo); /*Question : lPosItemNo�� unsigned short�� �ƴѰ�??*/
int SERVO_GetAllStatus(unsigned char iSlaveNo, unsigned long* dwInStatus, unsigned long* dwOutStatus, unsigned long* dwAxisStatus, long* lCmdPos, long* lActPos, long* lPosErr, long* lActVel, unsigned short* wPosItemNo);

int SERVO_SetCommandPos(unsigned char iSlaveNo, long lCmdPos);
int SERVO_SetActualPos(unsigned char iSlaveNo, long lActPos);
int SERVO_ClearPosition(unsigned char iSlaveNo);
int SERVO_GetCommandPos(unsigned char iSlaveNo, long* lCmdPos);
int SERVO_GetActualPos(unsigned char iSlaveNo, long* lActPos);
int SERVO_GetPosError(unsigned char iSlaveNo, long* lPosErr);
int SERVO_GetActualVel(unsigned char iSlaveNo, long* lActVel);

int SERVO_GetAlarmType(unsigned char iSlaveNo, unsigned char* nAlarmType);

//------------------------------------------------------------------
//					Motion Functions.
//------------------------------------------------------------------
int SERVO_MoveStop(unsigned char iSlaveNo);
int SERVO_EmergencyStop(unsigned char iSlaveNo);

int SERVO_MoveOriginSingleAxis(unsigned char iSlaveNo);
int SERVO_MoveSingleAxisAbsPos(unsigned char iSlaveNo, long lAbsPos, unsigned long lVelocity);
int SERVO_MoveSingleAxisIncPos(unsigned char iSlaveNo, long lIncPos, unsigned long lVelocity);
int SERVO_MoveToLimit(unsigned char iSlaveNo, unsigned long lVelocity, int iLimitDir);
int SERVO_MoveVelocity(unsigned char iSlaveNo, unsigned long lVelocity, int iVelDir);

int SERVO_PositionAbsOverride(unsigned char iSlaveNo, long lOverridePos);
int SERVO_PositionIncOverride(unsigned char iSlaveNo, long lOverridePos);
int SERVO_VelocityOverride(unsigned char iSlaveNo, unsigned long lVelocity);

int SERVO_TriggerPulseOutput(unsigned char iSlaveNo, int bStartTrigger, long lStartPos, unsigned long dwPeriod, unsigned long dwPulseTime, unsigned char nOutputPin, unsigned long dwReserved);
int SERVO_TriggerPulseStatus(unsigned char iSlaveNo, unsigned char* bTriggerStatus);

int SERVO_MovePush(unsigned char iSlaveNo, unsigned long dwStartSpd, unsigned long dwMoveSpd, long lPosition, unsigned short wAccel, unsigned short wDecel, unsigned short wPushRate, unsigned long dwPushSpd, long lEndPosition);
int SERVO_GetPushStatus(unsigned char iSlaveNo, unsigned char* nPushStatus);

//------------------------------------------------------------------
//					Ex-Motion Functions.
//------------------------------------------------------------------
//int SERVO_MoveSingleAxisAbsPosEx(unsigned char iSlaveNo, long lAbsPos, unsigned long lVelocity, MOTION_OPTION_EX* lpExOption);
//int SERVO_MoveSingleAxisIncPosEx(unsigned char iSlaveNo, long lIncPos, unsigned long lVelocity, MOTION_OPTION_EX* lpExOption);
//int SERVO_MoveVelocityEx(unsigned char iSlaveNo, unsigned long lVelocity, int iVelDir, VELOCITY_OPTION_EX* lpExOption);

//------------------------------------------------------------------
//					All-Motion Functions.
//------------------------------------------------------------------
int SERVO_AllMoveStop();
int SERVO_AllEmergencyStop();
int SERVO_AllMoveOriginSingleAxis();
int SERVO_AllMoveSingleAxisAbsPos(long lAbsPos, unsigned long lVelocity);
int SERVO_AllMoveSingleAxisIncPos(long lIncPos, unsigned long lVelocity);

//------------------------------------------------------------------
//					Position Table Functions.
//------------------------------------------------------------------
int SERVO_PosTableReadItem(unsigned char iSlaveNo, unsigned short wItemNo, LPITEM_NODE lpItem);
int SERVO_PosTableWriteItem(unsigned char iSlaveNo, unsigned short wItemNo, LPITEM_NODE lpItem);
int SERVO_PosTableWriteROM(unsigned char iSlaveNo);
int SERVO_PosTableReadROM(unsigned char iSlaveNo);
int SERVO_PosTableRunItem(unsigned char iSlaveNo, unsigned short wItemNo);
int SERVO_PosTableIsData(unsigned char iSlaveNo, LPPOSTABLE_DATAFLAG pBitFlag);
int SERVO_PosTableIsDataEx(unsigned char iSlaveNo, unsigned long dwSectionNo, LPPOSTABLE_DATAFLAG_EX pBitFlag);

int SERVO_PosTableRunOneItem(unsigned char iSlaveNo, int bNextMove, unsigned short wItemNo);
int SERVO_PosTableCheckStopMode(unsigned char iSlaveNo, unsigned char* pStopMode);

int SERVO_PosTableReadOneItem(unsigned char iSlaveNo, unsigned short wItemNo, unsigned short wOffset, long* lPosItemVal);
int SERVO_PosTableWriteOneItem(unsigned char iSlaveNo, unsigned short wItemNo, unsigned short wOffset, long lPosItemVal);

#endif	// _COMMINTERFACE_H_