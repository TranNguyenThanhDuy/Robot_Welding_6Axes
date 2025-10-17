#include <iostream>
#include <unistd.h>
#include "../Include/FAS_EziMOTIONPlusR.h"

bool Connect(const wchar_t* sPort, unsigned int dwBaudRate, int nPortID);
bool CheckDriveErr(int nPortID, unsigned char iSlaveNo);
bool SetServoOn(int nPortID, unsigned char iSlaveNo);
bool MoveLinearIncPos(int nPortID, unsigned char* iSlaveNo);
bool MoveLinearAbsPos(int nPortID, unsigned char* iSlaveNo);

#define SLAVE_CNT 2

int main()
{
	const wchar_t* sPort = L"ttyUSB0";
	unsigned char iSlaveNo[SLAVE_CNT] = { 1, 2 };
	unsigned int dwBaudRate = 115200;
	int nPortID = 0;

	// Device Connect
	if (Connect(sPort, dwBaudRate, nPortID) == false)
	{
		getchar();
		exit(1);
	}
	for (int nID = 0; nID < SLAVE_CNT; nID++)
	{
		// Drive Error Check
		if (CheckDriveErr(nPortID, iSlaveNo[nID]) == false)
		{
			getchar();
			exit(1);
		}

		// ServoOn 
		if (SetServoOn(nPortID, iSlaveNo[nID]) == false)
		{
			getchar();
			exit(1);
		}
	}

	// Move AxisIncPos
	if (MoveLinearIncPos(nPortID, iSlaveNo) == false)
	{
		getchar();
		exit(1);
	}

	// Move AxisAbsPos
	if (MoveLinearAbsPos(nPortID, iSlaveNo) == false)
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
		printf("[Slave : %d] Function(FAS_GetAxisStatus) was failed.\n", iSlaveNo);
		return false;
	}

	if (AxisStatus.FFLAG_ERRORALL)
	{
		// if Drive's Error was detected, Reset the ServoAlarm
		if (FAS_ServoAlarmReset(nPortID, iSlaveNo) != FMM_OK)
		{
			printf("[Slave : %d] Function(FAS_ServoAlarmReset) was failed.\n", iSlaveNo);
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
		printf("[Slave : %d] Function(FAS_GetAxisStatus) was failed.\n", iSlaveNo);
		return false;
	}

	if (AxisStatus.FFLAG_SERVOON == 0)
	{
		if (FAS_ServoEnable(nPortID, iSlaveNo, 1) != FMM_OK)
		{
			printf("[Slave : %d] Function(FAS_ServoEnable) was failed.\n", iSlaveNo);
			return false;
		}

		do
		{
			usleep(1 * 1000);

			if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(AxisStatus.dwValue)) != FMM_OK)
			{
				printf("[Slave : %d] Function(FAS_GetAxisStatus) was failed.\n", iSlaveNo);
				return false;
			}

			if (AxisStatus.FFLAG_SERVOON)
				printf("[Slave : %d] Servo ON \n", iSlaveNo);
		} while (!AxisStatus.FFLAG_SERVOON); // Wait until FFLAG_SERVOON is ON
	}
	else
	{
		printf("Servo is already ON \n");
	}

	return true;
}

bool MoveLinearIncPos(int nPortID, unsigned char* iSlaveNo)
{
	// Move Linear IncPos
	EZISERVO_AXISSTATUS AxisStatus;
	int lIncPos[2] = { 370000, 370000 };
	unsigned int dwVelocity = 0;
	unsigned short wAccelTime = 100;

	printf("---------------------------\n");
	// Increase the motor by 370000 pulse (target position : Relative position)
	dwVelocity = 40000;

	printf("[Linear Inc Mode] Move Motor ! \n");

	if (FAS_MoveLinearIncPos2(nPortID, SLAVE_CNT, iSlaveNo, lIncPos, dwVelocity, wAccelTime) != FMM_OK)
	{
		printf("Function(FAS_MoveLinearIncPos2) was failed.\n");
		return false;
	}

	// Check the Axis status until motor stops and the Inposition value is checked
	for (int nID = 0; nID < SLAVE_CNT; nID++)
	{
		do
		{
			usleep(1 * 1000);

			if (FAS_GetAxisStatus(nPortID, iSlaveNo[nID], &(AxisStatus.dwValue)) != FMM_OK)
			{
				printf("[Slave : %d] Function(FAS_GetAxisStatus) was failed.\n", iSlaveNo[nID]);
				return false;
			}
		} while (AxisStatus.FFLAG_MOTIONING || !(AxisStatus.FFLAG_INPOSITION));
	}

	return true;
}

bool MoveLinearAbsPos(int nPortID, unsigned char* iSlaveNo)
{
	// Move Linear AbsPos
	EZISERVO_AXISSTATUS AxisStatus;
	int lAbsPos[2] = { 0, 0 };
	unsigned int dwVelocity = 0;
	unsigned short wAccelTime = 100;

	printf("---------------------------\n");

	// Move the motor by 0 pulse (target position : Absolute position)
	dwVelocity = 40000;

	printf("[Linear Abs Mode] Move Motor ! \n");

	if (FAS_MoveLinearAbsPos2(nPortID, SLAVE_CNT, iSlaveNo, lAbsPos, dwVelocity, wAccelTime) != FMM_OK)
	{
		printf("Function(FAS_MoveLinearAbsPos2) was failed.\n");
		return false;
	}

	// Check the Axis status until motor stops and the Inposition value is checked
	for (int nID = 0; nID < SLAVE_CNT; nID++)
	{
		do
		{
			usleep(1 * 1000);

			if (FAS_GetAxisStatus(nPortID, iSlaveNo[nID], &(AxisStatus.dwValue)) != FMM_OK)
			{
				printf("[Slave : %d] Function(FAS_GetAxisStatus) was failed.\n", iSlaveNo[nID]);
				return false;
			}
		} while (AxisStatus.FFLAG_MOTIONING || !(AxisStatus.FFLAG_INPOSITION));
	}

	return true;
}
