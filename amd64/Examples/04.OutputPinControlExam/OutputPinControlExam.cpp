#include <iostream>
#include <unistd.h>
#include "../Include/FAS_EziMOTIONPlusR.h"

bool Connect(const wchar_t* sPort, unsigned int dwBaudRate, int nPortID);
bool SetOutputPin(int nPortID, unsigned char iSlaveNo);
bool ControlOutputSignal(int nPortID, unsigned char iSlaveNo);

#define INPUTPIN    12
#define OUTPUTPIN   10

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

	// Set Output pin 
	if (SetOutputPin(nPortID, iSlaveNo) == false)
	{
		getchar();
		exit(1);
	}

	// Control output pin signal
	if (ControlOutputSignal(nPortID, iSlaveNo) == false)
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

bool SetOutputPin(int nPortID, unsigned char iSlaveNo)
{
	unsigned char byPinNo = 0;
	unsigned char byLevel = 0;

	unsigned int dwLogicMask = 0;
	unsigned int dwOutputMask;

	printf("----------------------------------------------------------------------------- \n");
	// Check OutputPin Status
	for (int i = 0; i < OUTPUTPIN; i++)
	{
		if (FAS_GetIOAssignMap(nPortID, iSlaveNo, INPUTPIN + i, &dwLogicMask, &byLevel) != FMM_OK)
		{
			printf("Function(FAS_GetIOAssignMap) was failed.\n");
			return false;
		}

		if (dwLogicMask != IN_LOGIC_NONE)
			printf("Output Pin[%d] : Logic Mask 0x%08x (%s) \n", i, dwLogicMask, (byLevel == LEVEL_LOW_ACTIVE) ? "Low Active" : "High Active");
		else
			printf("Output Pin[%d] : Not Assigned \n", i);
	}

	printf("----------------------------------------------------------------------------- \n");

	// Set Output pin Value.
	byPinNo = 3;
	byLevel = LEVEL_HIGH_ACTIVE;
	dwOutputMask = SERVO_OUT_BITMASK_USEROUT0;

	if (FAS_SetIOAssignMap(nPortID, iSlaveNo, INPUTPIN + byPinNo, dwOutputMask, byLevel) != FMM_OK)
	{
		printf("Function(FAS_SetIOAssignMap) was failed.\n");
		return false;
	}

	// Show Output pins status
	for (int i = 0; i < OUTPUTPIN; i++)
	{
		if (FAS_GetIOAssignMap(nPortID, iSlaveNo, INPUTPIN + i, &dwLogicMask, &byLevel) != FMM_OK)
		{
			printf("Function(FAS_GetIOAssignMap) was failed.\n");
			return false;
		}

		if (dwLogicMask != IN_LOGIC_NONE)
			printf("Output Pin[%d] : Logic Mask 0x%08x (%s) \n", i, dwLogicMask, (byLevel == LEVEL_LOW_ACTIVE) ? "Low Active" : "High Active");
		else
			printf("Output Pin[%d] : Not Assigned \n", i);
	}

	return true;
}

bool ControlOutputSignal(int nPortID, unsigned char iSlaveNo)
{
	unsigned int dwOutputMask = SERVO_OUT_BITMASK_USEROUT0;

	printf("----------------------------------------------------------------------------- \n");

	// Control output signal on and off for 60 seconds
	for(int i = 0; i < 30; i++)
	{
		usleep(1000);
		// USEROUT0: ON
		if (FAS_SetIOOutput(nPortID, iSlaveNo, dwOutputMask, 0) != FMM_OK)
		{
			printf("Function(FAS_SetIOOutput) was failed.\n");
			return false;
		}

		usleep(1000);
		// USEROUT0: OFF
		if (FAS_SetIOOutput(nPortID, iSlaveNo, 0, dwOutputMask) != FMM_OK)
		{
			printf("Function(FAS_SetIOOutput) was failed.\n");
			return false;
		}
	}

	printf("finish WaitSecond! \n");
	return true;
}

