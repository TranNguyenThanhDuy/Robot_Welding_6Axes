#include <iostream>
#include <unistd.h>
#include "../Include/FAS_EziMOTIONPlusR.h"

bool Connect(const wchar_t* sPort, unsigned int dwBaudRate, int nPortID);
bool CheckDriveErr(int nPortID, unsigned char iSlaveNo);
bool SetServoOn(int nPortID, unsigned char iSlaveNo);
bool PosIncOverride(int nPortID, unsigned char iSlaveNo);
bool PosAbsOvrride(int nPortID, unsigned char iSlaveNo);

#define INCPOS 200000 // IncMove Target Position
#define ABSPOS 0 // AbsMove Target Position

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

	// Move AxisIncPos
	if (PosIncOverride(nPortID, iSlaveNo) == false)
	{
		getchar();
		exit(1);
	}

	// Move AxisAbsPos
	if (PosAbsOvrride(nPortID, iSlaveNo) == false)
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

bool PosIncOverride(int nPortID, unsigned char iSlaveNo)
{
	// Move AxisIncPos & PositionIncOverride
	EZISERVO_AXISSTATUS AxisStatus;
	int lIncPos = INCPOS;
	int lVelocity = 20000;
	int lActualPos = 0;
	int lChangePos = 0;
	int lPosDetect = 100000;

	printf("---------------------------\n");
	printf("[Inc Mode] Move Motor Start! \n");

	// 1. Move Command
	// Move the motor by INCPOS(200000) [pulse]
	if (FAS_MoveSingleAxisIncPos(nPortID, iSlaveNo, lIncPos, lVelocity) != FMM_OK)
	{
		printf("Function(FAS_MoveSingleAxisAbsPos) was failed.\n");
		return false;
	}

	// 2. Check Condition
	// Check current position
	do
	{
		usleep(1 * 1000);

		if (FAS_GetActualPos(nPortID, iSlaveNo, &lActualPos) != FMM_OK)
		{
			printf("Function(FAS_GetActualPos) was failed.\n");
			return false;
		}
	} while (lActualPos <= (lPosDetect));

	// 3. Change Position
	// If the current position is less than the target position, change final position.
	lChangePos += lIncPos;
	if (FAS_PositionIncOverride(nPortID, iSlaveNo, lChangePos) != FMM_OK)
	{
		printf("Function(FAS_PositionIncOverride) was failed.\n");
		return false;
	}
	else
		printf("[Before] Target Position : %d[pulse] / [After] Target Position : %d[pulse] \n", lIncPos, lChangePos + lIncPos);

	// 4. Confirm Move Complete
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

bool PosAbsOvrride(int nPortID, unsigned char iSlaveNo)
{
	// Move AxisAbsPos & PositionAbsOverride
	EZISERVO_AXISSTATUS AxisStatus;
	int lVelocity = 40000;
	int lIncEndPos = 0;
	int lActualPos = 0;
	int lAbsPos;
	int lChangePos = 0;

	if (FAS_GetActualPos(nPortID, iSlaveNo, &lIncEndPos) != FMM_OK)
	{
		printf("Function(FAS_GetActualPos) was failed.\n");
		return false;
	}

	printf("---------------------------\n");

	// 1. Move Command
	// Move the motor by ((lIncEndPos)* 1/ 4) pulse (target position : Absolute position)
	lAbsPos = lIncEndPos / 4;

	if (FAS_MoveSingleAxisAbsPos(nPortID, iSlaveNo, lAbsPos, lVelocity) != FMM_OK)
	{
		printf("Function(FAS_MoveSingleAxisAbsPos) was failed.\n");
		return false;
	}
	printf("[ABS Mode] Move Motor Start! \n");

	// 2. Check Condition
	// Check current position
	do
	{
		usleep(1 * 1000);

		if (FAS_GetActualPos(nPortID, iSlaveNo, &lActualPos) != FMM_OK)
		{
			printf("Function(FAS_GetActualPos) was failed.\n");
			return false;
		}
	} while (lActualPos >= (lIncEndPos / 2));

	// 3. Change Position
	// if the current position falls below half the INC End position, change the target position to zero.
	lChangePos = ABSPOS;
	if (FAS_PositionAbsOverride(nPortID, iSlaveNo, lChangePos) != FMM_OK)
	{
		printf("Function(FAS_PositionAbsOverride) was failed.\n");
		return false;
	}
	else
		printf("Before Target Posistion : %d[pulse] / Change Target Posistion : %d[pulse] \n", lAbsPos, lChangePos);

	// 4. Confirm Move Complete
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
