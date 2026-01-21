#include <iostream>
#include <unistd.h>
#include "../Include/FAS_EziMOTIONPlusR.h"

bool Connect(const wchar_t* sPort, unsigned int dwBaudRate, int nPortID);
bool CheckDriveErr(int nPortNo, unsigned char iSlaveNo);
bool SetServoOn(int nPortID, unsigned char iSlaveNo);
bool MovePos(int nPortID, unsigned char iSlaveNo, int nDistance, int nAccDecTime);
bool MovePosEx(int nPortID, unsigned char iSlaveNo, int nDistance, int nAccDecTime);

int main()
{
	const wchar_t* sPort = L"ttyUSB0";
	unsigned char iSlaveNo = 1;
	unsigned int dwBaudRate = 115200;
	int nPortID = 0;
	int distance;
	int accdec;

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

	/* SetParameter and Move */

	// AccDecTime = 100 ms
	distance = 100000;
	accdec = 100;
	if (MovePos(nPortID, iSlaveNo, distance, accdec) == false)
	{
		getchar();
		exit(1);
	}

	// AccDecTime = 200 ms
	distance = -100000;
	accdec = 200;
	if (MovePos(nPortID, iSlaveNo, distance, accdec) == false)
	{
		getchar();
		exit(1);
	}

	// AccDecTime = 300 ms
	distance = 100000;
	accdec = 300;
	if (MovePos(nPortID, iSlaveNo, distance, accdec) == false)
	{
		getchar();
		exit(1);
	}

	/* MoveEx */

	// AccDecTime = 100 ms
	distance = -100000;
	accdec = 100;
	if (MovePosEx(nPortID, iSlaveNo, distance, accdec) == false)
	{
		getchar();
		exit(1);
	}

	// AccDecTime = 200 ms
	distance = 100000;
	accdec = 200;
	if (MovePosEx(nPortID, iSlaveNo, distance, accdec) == false)
	{
		getchar();
		exit(1);
	}

	// AccDecTime = 300 ms
	distance = -100000;
	accdec = 300;
	if (MovePosEx(nPortID, iSlaveNo, distance, accdec) == false)
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

bool MovePos(int nPortID, unsigned char iSlaveNo, int nDistance, int nAccDecTime)
{
	// SetParameter & MoveSingleAxisIncPos

	EZISERVO_AXISSTATUS AxisStatus;
	int lIncPos;
	int lVelocity;

	// Set Acc Time
	if (FAS_SetParameter(nPortID, iSlaveNo, SERVO_AXISACCTIME, nAccDecTime) != FMM_OK)
	{
		printf("Function(FAS_SetParameter) was failed.\n");
		return false;
	}

	// Set Dec Time
	if (FAS_SetParameter(nPortID, iSlaveNo, SERVO_AXISDECTIME, nAccDecTime) != FMM_OK)
	{
		printf("Function(FAS_SetParameter) was failed.\n");
		return false;
	}

	printf("---------------------------\n");

	// Move the motor by 100000 pulse (target position : Absolute position)
	lIncPos = nDistance;
	lVelocity = 40000;

	printf("Move Motor! \n");
	if (FAS_MoveSingleAxisIncPos(nPortID, iSlaveNo, lIncPos, lVelocity) != FMM_OK)
	{
		printf("Function(FAS_MoveSingleAxisIncPos) was failed.\n");
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

bool MovePosEx(int nPortID, unsigned char iSlaveNo, int nDistance, int nAccDecTime)
{
	// MoveSingleAxisIncPosEx
	EZISERVO_AXISSTATUS AxisStatus;
	MOTION_OPTION_EX opt = { 0 };
	int lIncPos;
	int lVelocity;

	printf("---------------------------\n");

	lIncPos = nDistance;
	lVelocity = 40000;
	opt.flagOption.BIT_USE_CUSTOMACCEL = 1;
	opt.flagOption.BIT_USE_CUSTOMDECEL = 1;
	opt.wCustomAccelTime = nAccDecTime;
	opt.wCustomDecelTime = nAccDecTime;

	printf("Move Motor! [Ex]\n");
	if (FAS_MoveSingleAxisIncPosEx(nPortID, iSlaveNo, lIncPos, lVelocity, &opt) != FMM_OK)
	{
		printf("Function(FAS_MoveSingleAxisIncPosEx) was failed.\n");
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
