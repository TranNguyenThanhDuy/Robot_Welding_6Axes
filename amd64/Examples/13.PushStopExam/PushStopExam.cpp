#include <iostream>
#include <unistd.h>
#include "../Include/FAS_EziMOTIONPlusR.h"

bool Connect(const wchar_t* sPort, unsigned int dwBaudRate, int nPortID);
bool CheckDriveErr(int nPortID, unsigned char iSlaveNo);
bool SetServoOn(int nPortID, unsigned char iSlaveNo);
bool OperatePushMode(int nPortID, unsigned char iSlaveNo);
bool ReturnPosition(int nPortID, unsigned char iSlaveNo);
bool CheckPushResult(int nPortID, unsigned char iSlaveNo);

int main()
{
	const wchar_t* sPort = L"ttyUSB0";
	unsigned char iSlaveNo = 1;
	unsigned int dwBaudRate = 115200;
	int nPortID = 0;

	// Device Connect
	if (Connect(sPort, dwBaudRate, nPortID) == false)
	{
		getchar();
		exit(1);
	}

	// Drive Error Check
	if (CheckDriveErr(nPortID, iSlaveNo) == false)
	{
		getchar();
		exit(1);
	}

	// ServoOn 
	if (SetServoOn(nPortID, iSlaveNo) == false)
	{
		getchar();
		exit(1);
	}

	// Operate PushMode
	if (OperatePushMode(nPortID, iSlaveNo) == false)
	{
		getchar();
		exit(1);
	}

	// Check PushMotion Result
	if (CheckPushResult(nPortID, iSlaveNo) == false)
	{
		getchar();
		exit(1);
	}

	// Return Position
	if (ReturnPosition(nPortID, iSlaveNo) == false)
	{
		getchar();
		exit(1);
	}

	// Connection Close
	FAS_Close(nPortID);

	getchar();
}

bool Connect(const wchar_t* sPort, unsigned int dwBaudRate, int nPortID)
{
	bool bSuccess = false;

	if (FAS_Connect(sPort, dwBaudRate, nPortID) == false)
	{
		printf("Connection Fail!\n");
		bSuccess = false;
	}
	else
	{
		printf("Connection Success!\n");
		bSuccess = true;
	}

	return bSuccess;
}

bool CheckDriveErr(int nPortID, unsigned char iSlaveNo)
{
	// Check Drive's Error
	EZISERVO_AXISSTATUS AxisStatus;

	if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(AxisStatus.dwValue)) != FMM_OK)
	{
		printf("Function(FAS_GetAxisStatus) was failed.\n");
		return false;
	}

	if (AxisStatus.FFLAG_ERRORALL)
	{
		// if Drive's Error was detected, Reset the ServoAlarm
		if (FAS_ServoAlarmReset(nPortID, iSlaveNo) != FMM_OK)
		{
			printf("Function(FAS_ServoAlarmReset) was failed.\n");
			return false;
		}
	}

	return true;
}

bool SetServoOn(int nPortID, unsigned char iSlaveNo)
{
	// Check Drive's Servo Status
	EZISERVO_AXISSTATUS AxisStatus;

	// if ServoOnFlagBit is OFF('0'), switch to ON('1')
	if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(AxisStatus.dwValue)) != FMM_OK)
	{
		printf("Function(FAS_GetAxisStatus) was failed.\n");
		return false;
	}

	if (AxisStatus.FFLAG_SERVOON == 0)
	{
		if (FAS_ServoEnable(nPortID, iSlaveNo, 1) != FMM_OK)
		{
			printf("Function(FAS_ServoEnable) was failed.\n");
			return false;
		}

		do
		{
			usleep(1 * 1000);

			if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(AxisStatus.dwValue)) != FMM_OK)
			{
				printf("Function(FAS_GetAxisStatus) was failed.\n");
				return false;
			}

			if (AxisStatus.FFLAG_SERVOON)
				printf("Servo ON \n");
		} while (!AxisStatus.FFLAG_SERVOON); // Wait until FFLAG_SERVOON is ON
	}
	else
	{
		printf("Servo is already ON \n");
	}

	return true;
}

bool OperatePushMode(int nPortID, unsigned char iSlaveNo)
{
	EZISERVO_AXISSTATUS AxisStatus;
	unsigned int dwStartSpd, dwMoveSpd;
	unsigned short wAccel, wDecel;
	int lPosition;

	unsigned int dwPushSpd;
	unsigned short wPushRate;
	unsigned short wPushMode;
	int lEndPosition;

	// normal position motion
	dwStartSpd = 1;
	dwMoveSpd = 50000;
	wAccel = 500;
	wDecel = 500;
	lPosition = 500000;

	// push motion
	dwPushSpd = 2000;
	wPushRate = 50;
	wPushMode = 0;	// Stop Mode Push
	lEndPosition = lPosition + 10000;

	printf("---------------------------\n");

	if (FAS_MovePush(nPortID, iSlaveNo, dwStartSpd, dwMoveSpd, lPosition, wAccel, wDecel, wPushRate, dwPushSpd, lEndPosition, wPushMode) != FMM_OK)
	{
		printf("Function(FAS_MovePush) was failed.\n");
		return false;
	}

	// Check the Axis status until motor stops and the Inposition value is checked
	do
	{
		usleep(1 * 1000);

		if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(AxisStatus.dwValue)) != FMM_OK)
		{
			printf("Function(FAS_GetAxisStatus) was failed.\n");
			return false;
		}

	} while (AxisStatus.FFLAG_MOTIONING);

	return true;
}

bool CheckPushResult(int nPortID, unsigned char iSlaveNo)
{
	unsigned int dwOutput = 0;
	EZISERVO_OUTLOGIC outLogic;

	printf("---------------------------\n");
	printf("Check Result Function\n");

	if (FAS_GetIOOutput(nPortID, iSlaveNo, &dwOutput) != FMM_OK)
	{
		printf("Function(FAS_GetIOOutput) was failed. \n");
		return false;
	}
	else
	{
		outLogic.dwValue = dwOutput;

		if (outLogic.BIT_RESERVED0 == true)
			printf("Work Detected! \n");
		else
			printf("Work Not Detected! \n");
	}

	return true;
}

bool ReturnPosition(int nPortID, unsigned char iSlaveNo)
{
	// Return Position
	EZISERVO_AXISSTATUS AxisStatus;
	int lAbsPos;
	int lVelocity;

	// Move the motor by 0 pulse (target position : Absolute position)
	lAbsPos = 0;
	lVelocity = 50000;

	usleep(100 * 1000);
	printf("---------------------------\n");
	printf("Return Postion Start! \n");
	if (FAS_MoveSingleAxisAbsPos(nPortID, iSlaveNo, lAbsPos, lVelocity) != FMM_OK)
	{
		printf("Function(FAS_MoveSingleAxisAbsPos) was failed.\n");
		return false;
	}

	// Check the Axis status until motor stops and the Inposition value is checked
	do
	{
		usleep(1 * 1000);

		if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(AxisStatus.dwValue)) != FMM_OK)
		{
			printf("Function(FAS_GetAxisStatus) was failed.\n");
			return false;
		}

	} while (AxisStatus.FFLAG_MOTIONING || !(AxisStatus.FFLAG_INPOSITION));

	return true;
}
