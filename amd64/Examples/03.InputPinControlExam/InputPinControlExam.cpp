#include <iostream>
#include <unistd.h>
#include "../Include/FAS_EziMOTIONPlusR.h"

#define INPUTPIN 12

bool Connect(const wchar_t* sPort, unsigned int dwBaudRate, int nPortID);
bool SetInputPin(int nPortID, unsigned char iSlaveNo);
bool CheckInputSignal(int nPortID, unsigned char iSlaveNo);

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

	// Set Input pins 
	if (SetInputPin(nPortID, iSlaveNo) == false)
	{
		getchar();
		exit(1);
	}

	// Check Input Pin Signal
	if (CheckInputSignal(nPortID, iSlaveNo) == false)
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

bool SetInputPin(int nPortID, unsigned char iSlaveNo)
{
	unsigned char byPinNo = 0;
	unsigned int dwLogicMask = 0;
	unsigned char byLevel = 0;

	unsigned int dwInputMask;

	printf("----------------------------------------------------------------------------- \n");

	// Set Input pin Value.
	byPinNo = 3;
	byLevel = LEVEL_LOW_ACTIVE;
	dwInputMask = SERVO_IN_BITMASK_USERIN0;

	if (FAS_SetIOAssignMap(nPortID, iSlaveNo, byPinNo, dwInputMask, byLevel) != FMM_OK)
	{
		printf("Function(FAS_SetIOAssignMap) was failed.\n");
		return false;
	}
	else
	{
		printf("SERVO_IN_BITMASK_USERIN0 (Pin%d) : [%s]\n", byPinNo, (byLevel == LEVEL_LOW_ACTIVE) ? "Low Active" : "High Active");
	}

	printf("----------------------------------------------------------------------------- \n");

	// Show Input pins status
	for (int i = 0; i < INPUTPIN; i++)
	{
		if (FAS_GetIOAssignMap(nPortID, iSlaveNo, i, &dwLogicMask, &byLevel) != FMM_OK)
		{
			printf("Function(FAS_SetIOAssignMap) was failed.\n");
			return false;
		}

		if (dwLogicMask != IN_LOGIC_NONE)
			printf("Input PIN[%d] : Logic Mask 0x%08x (%s) \n", i, dwLogicMask, (byLevel == LEVEL_LOW_ACTIVE) ? "Low Active" : "High Active");
		else
			printf("Input Pin[%d] : Not Assigned \n", i);
	}

	return true;
}

bool CheckInputSignal(int nPortID, unsigned char iSlaveNo)
{
	unsigned int dwInput;
	unsigned int dwInputMask = SERVO_IN_BITMASK_USERIN0;

	printf("----------------------------------------------------------------------------- \n");

	// When the value SERVO_IN_BITMASK_USERIN0 is entered with the set PinNo, an input confirmation message is displyed.
	// Monitoring input signal for 60 seconds
	for(int i = 0; i < 600; i++)
	{
		usleep(100 * 1000);

		if (FAS_GetIOInput(nPortID, iSlaveNo, &dwInput) == FMM_OK)
		{
			if (dwInput & dwInputMask)
			{
				printf("INPUT PIN DETECTED.\n");
			}
		}
		else
			return false;
	} 

	printf("finish WaitSecond! \n");
	return true;
}

