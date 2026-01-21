#include <iostream>
#include <unistd.h>
#include "../Include/FAS_EziMOTIONPlusR.h"

bool Connect(const wchar_t* sPort, unsigned int dwBaudRate, int nPortID);
bool CheckDriveErr(int nPortID, unsigned char iSlaveNo);
bool SetServoOn(int nPortID, unsigned char iSlaveNo);
bool MovePosAndVelOverride(int nPortID, unsigned char iSlaveNo);
bool JogAndVelOverride(int nPortID, unsigned char iSlaveNo);

#define CW  1
#define CCW 0

#define INCPOS 200000 // IncMove Target Position

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

	// Move AxisIncPos & VelocityOverride
	if (MovePosAndVelOverride(nPortID, iSlaveNo) == false)
	{
		getchar();
		exit(1);
	}

	// MoveVelocity  & VelocityOverride
	if (JogAndVelOverride(nPortID, iSlaveNo) == false)
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

bool MovePosAndVelOverride(int nPortID, unsigned char iSlaveNo)
{
	// MoveSingleAxisIncPos & VelocityOverride

	// Move the motor by INCPOS(200000) [pulse]
	EZISERVO_AXISSTATUS AxisStatus;
	int lVelocity = 20000;
	int lActualPos = 0;
	int lPosDetect = 100000;
	int lIncPos = INCPOS;
	int lChangeVelocity = 0;

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

	// 3. Change Velocity
	// If the current position is less than the target position, change velocity.
	lChangeVelocity = lVelocity * 2;
	if (FAS_VelocityOverride(nPortID, iSlaveNo, lChangeVelocity) != FMM_OK)
	{
		printf("Function(FAS_VelocityOverride) was failed.\n");
		return false;
	}
	else
		printf("Before Velocity : %d[pps] / Change Velocity : %d[pps] \n", lVelocity, lChangeVelocity);

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

bool JogAndVelOverride(int nPortID, unsigned char iSlaveNo)
{
	// MoveVelocity & VelocityOverride

	int lVelocity = 10000;
	int lChangeVelocity = 0;
	int lActualVelocity = 0;

	printf("---------------------------\n");

	// 1. Move Command
	// Move the motor 
	if (FAS_MoveVelocity(nPortID, iSlaveNo, lVelocity, CW) != FMM_OK)
	{
		printf("Function(FAS_MoveVelocity) was failed. \n");
		return false;
	}
	else
	{
		printf("Current Velocity : %d \n", lVelocity);
	}

	printf(">> Wait... 3 Second \n");
	usleep(3 * 1000 * 1000);

	// 2. Change Velocity
	if (FAS_GetActualVel(nPortID, iSlaveNo, &lActualVelocity) != FMM_OK)
	{
		printf("Function(FAS_GetActualVel) was failed. \n");
		return false;
	}

	lChangeVelocity = lActualVelocity * 2;
	if (FAS_VelocityOverride(nPortID, iSlaveNo, lChangeVelocity) != FMM_OK)
	{
		printf("Function(FAS_VelocityOverride) was failed. \n");
		return false;
	}
	else
		printf("Before Velocity : %d [pps] / Change Velocity : %d [pps] \n", lActualVelocity, lChangeVelocity);

	printf(">> Wait... 5 Second \n");
	usleep(5 * 1000 * 1000);

	// 3. Change Velocity
	if (FAS_GetActualVel(nPortID, iSlaveNo, &lActualVelocity) != FMM_OK)
	{
		printf("Function(FAS_GetActualVel) was failed. \n");
		return false;
	}

	lChangeVelocity = lActualVelocity * 2;
	if (FAS_VelocityOverride(nPortID, iSlaveNo, lChangeVelocity) != FMM_OK)
	{
		printf("Function(FAS_VelocityOverride) was failed. \n");
		return false;
	}
	else
		printf("Before Velocity : %d [pps] / Change Velocity : %d [pps] \n", lActualVelocity, lChangeVelocity);

	printf(">> Wait... 10 Second \n");
	usleep(10 * 1000 * 1000);

	// 4. Stop
	FAS_MoveStop(nPortID, iSlaveNo);
	printf("FAS_MoveStop! \n");

	return true;
}
