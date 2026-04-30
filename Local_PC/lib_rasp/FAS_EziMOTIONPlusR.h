//////////////////////////////////////////////////////////////////////////
//
//	File:			FAS_EziMOTIONPlusR.h
//	Last Modified Date:	24.08.30
//	Last Modified Ver:	1.0.2
//
//////////////////////////////////////////////////////////////////////////

#pragma once

#include <codecvt>
#include <locale>

#include "ReturnCodes_Define.h"
#include "MOTION_DEFINE.h"
#include "COMM_Define.h"

extern "C"
{
	//------------------------------------------------------------------------------
	//					Connection Functions
	//------------------------------------------------------------------------------
	 int 	FAS_Connect(const wchar_t* sPort, unsigned int dwBaud, unsigned char nPortID);

	 int	FAS_OpenPort(const wchar_t* sPort, unsigned int dwBaud, unsigned char nPortID);
	 int 	FAS_AttachSlave(unsigned char nPortID, unsigned char iSlaveNo);

	 void 	FAS_Close(unsigned char nPortID);

	 int 	FAS_IsSlaveExist(unsigned char nPortID, unsigned char iSlaveNo);

	//------------------------------------------------------------------------------
	//					Log Functions
	//------------------------------------------------------------------------------
	 void 	FAS_EnableLog(int bEnable);
	 void 	FAS_SetLogLevel(enum LOG_LEVEL level);
	 int 	FAS_SetLogPath(const wchar_t* lpPath);

	 void 	FAS_PrintCustomLog(unsigned char nPortID, enum LOG_LEVEL level, const wchar_t* lpszMsg);

	//------------------------------------------------------------------------------
	//					Info Functions
	//------------------------------------------------------------------------------
	 int 	FAS_GetSlaveInfo(unsigned char nPortID, unsigned char iSlaveNo, unsigned char* pType, char* lpBuff, int nBuffSize);
	 int 	FAS_GetMotorInfo(unsigned char nPortID, unsigned char iSlaveNo, unsigned char* pType, char* lpBuff, int nBuffSize);
	 int 	FAS_GetSlaveInfoEx(unsigned char nPortID, unsigned char iSlaveNo, DRIVE_INFO* lpDriveInfo);

	//------------------------------------------------------------------------------
	//					Parameter Functions
	//------------------------------------------------------------------------------
	 int 	FAS_SaveAllParameters(unsigned char nPortID, unsigned char iSlaveNo);
	 int 	FAS_SetParameter(unsigned char nPortID, unsigned char iSlaveNo, unsigned char iParamNo, int lParamValue);
	 int 	FAS_GetParameter(unsigned char nPortID, unsigned char iSlaveNo, unsigned char iParamNo, int* lParamValue);
	 int 	FAS_GetROMParameter(unsigned char nPortID, unsigned char iSlaveNo, unsigned char iParamNo, int* lRomParam);

	//------------------------------------------------------------------------------
	//					IO Functions
	//------------------------------------------------------------------------------
	 int 	FAS_SetIOInput(unsigned char nPortID, unsigned char iSlaveNo, unsigned int dwIOSETMask, unsigned int dwIOCLRMask);
	 int 	FAS_GetIOInput(unsigned char nPortID, unsigned char iSlaveNo, unsigned int* dwIOInput);

	 int 	FAS_SetIOOutput(unsigned char nPortID, unsigned char iSlaveNo, unsigned int dwIOSETMask, unsigned int dwIOCLRMask);
	 int 	FAS_GetIOOutput(unsigned char nPortID, unsigned char iSlaveNo, unsigned int* dwIOOutput);

	 int 	FAS_GetIOAssignMap(unsigned char nPortID, unsigned char iSlaveNo, unsigned char iIOPinNo, unsigned int* dwIOLogicMask, unsigned char* bLevel);
	 int 	FAS_SetIOAssignMap(unsigned char nPortID, unsigned char iSlaveNo, unsigned char iIOPinNo, unsigned int dwIOLogicMask, unsigned char bLevel);

	 int 	FAS_IOAssignMapReadROM(unsigned char nPortID, unsigned char iSlaveNo);

	//------------------------------------------------------------------------------
	//					Servo Driver Control Functions
	//------------------------------------------------------------------------------
	 int 	FAS_ServoEnable(unsigned char nPortID, unsigned char iSlaveNo, int bOnOff);
	 int 	FAS_ServoAlarmReset(unsigned char nPortID, unsigned char iSlaveNo);
	 int 	FAS_StepAlarmReset(unsigned char nPortID, unsigned char iSlaveNo, int bReset);
	 int 	FAS_BrakeSet(unsigned char nPortID, unsigned char iSlaveNo, int bSet, int* nResult);

	//------------------------------------------------------------------------------
	//					Read Status and Position
	//------------------------------------------------------------------------------
	 int 	FAS_GetAxisStatus(unsigned char nPortID, unsigned char iSlaveNo, unsigned int* dwAxisStatus);
	 int 	FAS_GetIOAxisStatus(unsigned char nPortID, unsigned char iSlaveNo, unsigned int* dwInStatus, unsigned int* dwOutStatus, unsigned int* dwAxisStatus);
	 int 	FAS_GetMotionStatus(unsigned char nPortID, unsigned char iSlaveNo, int* lCmdPos, int* lActPos, int* lPosErr, int* lActVel, unsigned short* wPosItemNo);
	 int 	FAS_GetAllStatus(unsigned char nPortID, unsigned char iSlaveNo, unsigned int* dwInStatus, unsigned int* dwOutStatus, unsigned int* dwAxisStatus, int* lCmdPos, int* lActPos, int* lPosErr, int* lActVel, unsigned short* wPosItemNo);
	 int 	FAS_GetAllStatusEx(unsigned char nPortID, unsigned char iSlaveNo, unsigned char* pTypes, int* pDatas);

	 int 	FAS_SetCommandPos(unsigned char nPortID, unsigned char iSlaveNo, int lCmdPos);
	 int 	FAS_SetActualPos(unsigned char nPortID, unsigned char iSlaveNo, int lActPos);
	 int 	FAS_ClearPosition(unsigned char nPortID, unsigned char iSlaveNo);
	 int 	FAS_GetCommandPos(unsigned char nPortID, unsigned char iSlaveNo, int* lCmdPos);
	 int 	FAS_GetActualPos(unsigned char nPortID, unsigned char iSlaveNo, int* lActPos);
	 int 	FAS_GetPosError(unsigned char nPortID, unsigned char iSlaveNo, int* lPosErr);
	 int 	FAS_GetActualVel(unsigned char nPortID, unsigned char iSlaveNo, int* lActVel);

	 int 	FAS_GetAlarmType(unsigned char nPortID, unsigned char iSlaveNo, unsigned char* nAlarmType);

	//------------------------------------------------------------------
	//					Motion Functions.
	//------------------------------------------------------------------
	 int 	FAS_MoveStop(unsigned char nPortID, unsigned char iSlaveNo);
	 int 	FAS_EmergencyStop(unsigned char nPortID, unsigned char iSlaveNo);

	 int 	FAS_MovePause(unsigned char nPortID, unsigned char iSlaveNo, int bPause);

	 int 	FAS_MoveOriginSingleAxis(unsigned char nPortID, unsigned char iSlaveNo);
	 int 	FAS_MoveSingleAxisAbsPos(unsigned char nPortID, unsigned char iSlaveNo, int lAbsPos, unsigned int lVelocity);
	 int 	FAS_MoveSingleAxisIncPos(unsigned char nPortID, unsigned char iSlaveNo, int lIncPos, unsigned int lVelocity);
	 int 	FAS_MoveToLimit(unsigned char nPortID, unsigned char iSlaveNo, unsigned int lVelocity, int iLimitDir);
	 int 	FAS_MoveVelocity(unsigned char nPortID, unsigned char iSlaveNo, unsigned int lVelocity, int iVelDir);

	 int 	FAS_PositionAbsOverride(unsigned char nPortID, unsigned char iSlaveNo, int lOverridePos);
	 int 	FAS_PositionIncOverride(unsigned char nPortID, unsigned char iSlaveNo, int lOverridePos);
	 int 	FAS_VelocityOverride(unsigned char nPortID, unsigned char iSlaveNo, unsigned int lVelocity);

	 int 	FAS_MoveLinearAbsPos(unsigned char nPortID, unsigned char nNoOfSlaves, unsigned char* iSlavesNo, int* lAbsPos, unsigned int lFeedrate, unsigned short wAccelTime);
	 int 	FAS_MoveLinearIncPos(unsigned char nPortID, unsigned char nNoOfSlaves, unsigned char* iSlavesNo, int* lIncPos, unsigned int lFeedrate, unsigned short wAccelTime);

	 int 	FAS_MoveLinearAbsPos2(unsigned char nPortID, unsigned char nNoOfSlaves, unsigned char* iSlavesNo, int* lAbsPos, unsigned int lFeedrate, unsigned short wAccelTime);
	 int 	FAS_MoveLinearIncPos2(unsigned char nPortID, unsigned char nNoOfSlaves, unsigned char* iSlavesNo, int* lIncPos, unsigned int lFeedrate, unsigned short wAccelTime);

	 int 	FAS_MoveCircleAbsPos1(unsigned char nPortID, unsigned char nNoOfSlaves, unsigned char* iSlavesNo, int* lplCirEndAbs, int* lplCirCenterAbs, int iDirection, unsigned int lFeedrate, unsigned short wAccelTime, int bSCurve);
	 int 	FAS_MoveCircleIncPos1(unsigned char nPortID, unsigned char nNoOfSlaves, unsigned char* iSlavesNo, int* lplCirEndInc, int* lplCirCenterInc, int iDirection, unsigned int lFeedrate, unsigned short wAccelTime, int bSCurve);
	 int 	FAS_MoveCircleAbsPos2(unsigned char nPortID, unsigned char nNoOfSlaves, unsigned char* iSlavesNo, int* lplCirEndAbs, unsigned int lRadius, int iDirection, unsigned int lFeedrate, unsigned short wAccelTime, int bSCurve);
	 int 	FAS_MoveCircleIncPos2(unsigned char nPortID, unsigned char nNoOfSlaves, unsigned char* iSlavesNo, int* lplCirEndInc, unsigned int lRadius, int iDirection, unsigned int lFeedrate, unsigned short wAccelTime, int bSCurve);
	 int 	FAS_MoveCircleAbsPos3(unsigned char nPortID, unsigned char nNoOfSlaves, unsigned char* iSlavesNo, int* lplCirCenterAbs, unsigned int nAngle, int iDirection, unsigned int lFeedrate, unsigned short wAccelTime, int bSCurve);
	 int 	FAS_MoveCircleIncPos3(unsigned char nPortID, unsigned char nNoOfSlaves, unsigned char* iSlavesNo, int* lplCirCenterInc, unsigned int nAngle, int iDirection, unsigned int lFeedrate, unsigned short wAccelTime, int bSCurve);

	 int 	FAS_TriggerOutput_RunA(unsigned char nPortID, unsigned char iSlaveNo, int bStartTrigger, int lStartPos, unsigned int dwPeriod, unsigned int dwPulseTime);
	 int 	FAS_TriggerOutput_Status(unsigned char nPortID, unsigned char iSlaveNo, unsigned char* bTriggerStatus);

	 int 	FAS_SetTriggerOutputEx(unsigned char nPortID, unsigned char iSlaveNo, unsigned char nOutputNo, unsigned char bRun, unsigned short wOnTime, unsigned char nTriggerCount, int* arrTriggerPosition);
	 int 	FAS_GetTriggerOutputEx(unsigned char nPortID, unsigned char iSlaveNo, unsigned char nOutputNo, unsigned char* bRun, unsigned short* wOnTime, unsigned char* nTriggerCount, int* arrTriggerPosition);

	 int 	FAS_MovePush(unsigned char nPortID, unsigned char iSlaveNo, unsigned int dwStartSpd, unsigned int dwMoveSpd, int lPosition, unsigned short wAccel, unsigned short wDecel, unsigned short wPushRate, unsigned int dwPushSpd, int lEndPosition, unsigned short wPushMode);
	 int 	FAS_GetPushStatus(unsigned char nPortID, unsigned char iSlaveNo, unsigned char* nPushStatus);

	//------------------------------------------------------------------
	//					Ex-Motion Functions.
	//------------------------------------------------------------------
	 int 	FAS_MoveSingleAxisAbsPosEx(unsigned char nPortID, unsigned char iSlaveNo, int lAbsPos, unsigned int lVelocity, MOTION_OPTION_EX* lpExOption);
	 int 	FAS_MoveSingleAxisIncPosEx(unsigned char nPortID, unsigned char iSlaveNo, int lIncPos, unsigned int lVelocity, MOTION_OPTION_EX* lpExOption);
	 int 	FAS_MoveVelocityEx(unsigned char nPortID, unsigned char iSlaveNo, unsigned int lVelocity, int iVelDir, VELOCITY_OPTION_EX* lpExOption);

	//------------------------------------------------------------------
	//					All-Motion Functions.
	//------------------------------------------------------------------
	 int 	FAS_AllMoveStop(unsigned char nPortID);
	 int 	FAS_AllEmergencyStop(unsigned char nPortID);
	 int 	FAS_AllMoveOriginSingleAxis(unsigned char nPortID);
	 int 	FAS_AllMoveSingleAxisAbsPos(unsigned char nPortID, int lAbsPos, unsigned int lVelocity);
	 int 	FAS_AllMoveSingleAxisIncPos(unsigned char nPortID, int lIncPos, unsigned int lVelocity);

	//------------------------------------------------------------------
	//					Position Table Functions.
	//------------------------------------------------------------------
	 int 	FAS_PosTableReadItem(unsigned char nPortID, unsigned char iSlaveNo, unsigned short wItemNo, LPITEM_NODE lpItem);
	 int 	FAS_PosTableWriteItem(unsigned char nPortID, unsigned char iSlaveNo, unsigned short wItemNo, LPITEM_NODE lpItem);
	 int 	FAS_PosTableWriteROM(unsigned char nPortID, unsigned char iSlaveNo);
	 int 	FAS_PosTableReadROM(unsigned char nPortID, unsigned char iSlaveNo);
	 int 	FAS_PosTableRunItem(unsigned char nPortID, unsigned char iSlaveNo, unsigned short wItemNo);

	 int 	FAS_PosTableReadOneItem(unsigned char nPortID, unsigned char iSlaveNo, unsigned short wItemNo, unsigned short wOffset, int* lPosItemVal);
	 int 	FAS_PosTableWriteOneItem(unsigned char nPortID, unsigned char iSlaveNo, unsigned short wItemNo, unsigned short wOffset, int lPosItemVal);

	 int 	FAS_PosTableSingleRunItem(unsigned char nPortID, unsigned char iSlaveNo, int bNextMove, unsigned short wItemNo);

	//------------------------------------------------------------------
	//					Gap Control Functions.
	//------------------------------------------------------------------
	 int 	FAS_GapControlEnable(unsigned char nPortID, unsigned char iSlaveNo, unsigned short wItemNo, int lGapCompSpeed, int lGapAccTime, int lGapDecTime, int lGapStartSpeed);
	 int 	FAS_GapControlDisable(unsigned char nPortID, unsigned char iSlaveNo);
	 int 	FAS_IsGapControlEnable(unsigned char nPortID, unsigned char iSlaveNo, int* bIsEnable, unsigned short* wCurrentItemNo);

	 int 	FAS_GapControlGetADCValue(unsigned char nPortID, unsigned char iSlaveNo, int* lADCValue);
	 int 	FAS_GapOneResultMonitor(unsigned char nPortID, unsigned char iSlaveNo, unsigned char* bUpdated, int* iIndex, int* lGapValue, int* lCmdPos, int* lActPos, int* lCompValue, int* lReserved);

	//------------------------------------------------------------------
	//					Alarm Type History Functions.
	//------------------------------------------------------------------
	 int 	FAS_GetAlarmLogs(unsigned char nPortID, unsigned char iSlaveNo, ALARM_LOG* pAlarmLog);
	 int 	FAS_ResetAlarmLogs(unsigned char nPortID, unsigned char iSlaveNo);

	//------------------------------------------------------------------
	//					I/O Module Functions.
	//------------------------------------------------------------------
	 int 	FAS_GetInput(unsigned char nPortID, unsigned char iSlaveNo, unsigned int* uInput, unsigned int* uLatch);

	 int 	FAS_ClearLatch(unsigned char nPortID, unsigned char iSlaveNo, unsigned int uLatchMask);
	 int 	FAS_GetLatchCount(unsigned char nPortID, unsigned char iSlaveNo, unsigned char iInputNo, unsigned int* uCount);
	 int 	FAS_GetLatchCountAll(unsigned char nPortID, unsigned char iSlaveNo, unsigned int* ppuAllCount);
	 int 	FAS_GetLatchCountAll32(unsigned char nPortID, unsigned char iSlaveNo, unsigned int* ppuAllCount);
	 int 	FAS_ClearLatchCount(unsigned char nPortID, unsigned char iSlaveNo, unsigned int uInputMask);

	 int 	FAS_GetOutput(unsigned char nPortID, unsigned char iSlaveNo, unsigned int* uOutput, unsigned int* uStatus);
	 int 	FAS_SetOutput(unsigned char nPortID, unsigned char iSlaveNo, unsigned int uSetMask, unsigned int uClearMask);

	 int 	FAS_SetTrigger(unsigned char nPortID, unsigned char iSlaveNo, unsigned char uOutputNo, TRIGGER_INFO* pTrigger);
	 int 	FAS_SetRunStop(unsigned char nPortID, unsigned char iSlaveNo, unsigned int uRunMask, unsigned int uStopMask);
	 int 	FAS_GetTriggerCount(unsigned char nPortID, unsigned char iSlaveNo, unsigned char uOutputNo, unsigned int* uCount);

	 int 	FAS_GetIOLevel(unsigned char nPortID, unsigned char iSlaveNo, unsigned int* uIOLevel);
	 int 	FAS_SetIOLevel(unsigned char nPortID, unsigned char iSlaveNo, unsigned int uIOLevel);
	 int 	FAS_LoadIOLevel(unsigned char nPortID, unsigned char iSlaveNo);
	 int 	FAS_SaveIOLevel(unsigned char nPortID, unsigned char iSlaveNo);

	 int 	FAS_GetInputFilter(unsigned char nPortID, unsigned char iSlaveNo, unsigned short* filter);
	 int 	FAS_SetInputFilter(unsigned char nPortID, unsigned char iSlaveNo, unsigned short filter);

	 int 	FAS_GetIODirection(unsigned char nPortID, unsigned char iSlaveNo, unsigned int* direction);
	 int 	FAS_SetIODirection(unsigned char nPortID, unsigned char iSlaveNo, unsigned int direction);

	//------------------------------------------------------------------
	//					Ezi-IO AD Functions
	//------------------------------------------------------------------
	 int 	FAS_SetADConfig(unsigned char nPortID, unsigned char iSlaveNo, unsigned char channel, enum AD_DATA_TYPE type, int value, int* recv);
	 int 	FAS_GetADConfig(unsigned char nPortID, unsigned char iSlaveNo, unsigned char channel, enum AD_DATA_TYPE type, int* value);

	 int 	FAS_LoadADConfig(unsigned char nPortID, unsigned char iSlaveNo);
	 int 	FAS_SaveADConfig(unsigned char nPortID, unsigned char iSlaveNo);

	 int 	FAS_ReadADValue(unsigned char nPortID, unsigned char iSlaveNo, unsigned char channel, short* advalue);
	 int 	FAS_ReadADAllValue(unsigned char nPortID, unsigned char iSlaveNo, unsigned char offset, AD_BUFFER* adbuffer);

	 int 	FAS_GetAllADResult(unsigned char nPortID, unsigned char iSlaveNo, AD_RESULT* result);
	 int 	FAS_GetADResult(unsigned char nPortID, unsigned char iSlaveNo, unsigned char channel, float* adresult);
	 int 	FAS_SetADRange(unsigned char nPortID, unsigned char iSlaveNo, unsigned char channel, enum AD_RANGE range);
}
