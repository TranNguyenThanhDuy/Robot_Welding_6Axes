#include <iostream>
#include <unistd.h>
#include "../Include/FAS_EziMOTIONPlusR.h"

bool Connect(const wchar_t* sPort, unsigned int dwBaudRate, int nPortID);
bool CheckDriveErr(int nPortID, unsigned char iSlaveNo);
bool SetServoOn(int nPortID, unsigned char iSlaveNo);
bool SetPosTable(int nPortID, unsigned char iSlaveNo);
bool RunPosTable(int nPortID, unsigned char iSlaveNo);

int main()
{
	const wchar_t* sPort = L"ttyUSB0";
	unsigned char iSlaveNo = 0;
	unsigned int dwBaudRate = 115200;
	int nPortID = 0;

	// Device Connect
	if (Connect(sPort, dwBaudRate, nPortID) == false)
	{
		getchar();
		exit(1);
	}

	// Drive Error Checka
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

	// Sets values in the position table.
	if (SetPosTable(nPortID, iSlaveNo) == false)
	{
		getchar();
		exit(1);
	}

	// Run Position Table
	if (RunPosTable(nPortID, iSlaveNo) == false)
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

bool SetPosTable(int nPortID, unsigned char iSlaveNo)
{
	// Sets values in the position table.
	ITEM_NODE nodeItem;
	unsigned short wItemNo = 1;

	printf("--------------------------- \n");
	// Gets the data values of that node.
	if (FAS_PosTableReadItem(nPortID, iSlaveNo, wItemNo, &nodeItem) != FMM_OK)
	{
		printf("Function(FAS_PosTableReadItem) was failed.\n");
		return false;
	}

	// Overwrite part of node's data
	nodeItem.dwMoveSpd = 50000;
	nodeItem.lPosition = 250000;
	nodeItem.wBranch = 3;
	nodeItem.wContinuous = 1;

	// Set node data in Position Table
	if (FAS_PosTableWriteItem(nPortID, iSlaveNo, wItemNo, &nodeItem))
	{
		printf("Function(FAS_PosTableWriteItem) was failed.\n");
		return false;
	}
	else
		printf("Setting PosTable Data ! \n");

	return true;
}

bool RunPosTable(int nPortID, unsigned char iSlaveNo)
{
	EZISERVO_AXISSTATUS AxisStatus;
	unsigned short wItemNo = 1;

	printf("--------------------------- \n");
	if (FAS_PosTableRunItem(nPortID, iSlaveNo, wItemNo) != FMM_OK)
	{
		printf("Function(FAS_PosTableRunItem) was failed.\n");
		return false;
	}

	printf("PosTable Run! \n");

	do
	{
		usleep(1 * 1000);

		if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(AxisStatus.dwValue)) != FMM_OK)
		{
			printf("Function(FAS_GetAxisStatus) was failed.\n");
			return false;
		}

		if (AxisStatus.FFLAG_PTSTOPPED == true)
			printf("Position Table Run Stop! \n");
	} while (!AxisStatus.FFLAG_PTSTOPPED);

	return true;
}
