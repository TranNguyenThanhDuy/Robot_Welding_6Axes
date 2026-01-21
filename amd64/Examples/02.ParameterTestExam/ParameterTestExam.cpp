#include <iostream>
#include "../Include/FAS_EziMOTIONPlusR.h"

bool Connect(const wchar_t* sPort, unsigned int dwBaudRate, int nPortID);
bool SetParameter(int nPortID, unsigned char iSlaveNo);

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

	// Load and Set Parameter
	if (SetParameter(nPortID, iSlaveNo) == false)
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

bool SetParameter(int nPortID, unsigned char iSlaveNo)
{
	int nRtn;
	int nChangeValue = 100;
	int lParamVal = 0;

	printf("----------------------------------------------------------------------------- \n");
	// Check The Axis Start Speed Parameter Status
	nRtn = FAS_GetParameter(nPortID, iSlaveNo, SERVO_AXISSTARTSPEED, &lParamVal);
	if (nRtn != FMM_OK)
	{
		printf("Function(FAS_GetParameter) was failed.\n");
		return false;
	}
	else
	{
		printf("Load Parameter[Before] : Start Speed = %d[pps] \n", lParamVal);
	}

	printf("----------------------------------------------------------------------------- \n");
	// Change the (Axxis Start Speed Parameter) vlaue to (nChangeValue) value.
	nRtn = FAS_SetParameter(nPortID, iSlaveNo, SERVO_AXISSTARTSPEED, nChangeValue);
	if (nRtn != FMM_OK)
	{
		printf("Function(FAS_SetParameter) was failed.\n");
		return false;
	}
	else
	{
		printf("Set Parameter: Start Speed = %d[pps] \n", nChangeValue);
	}

	printf("----------------------------------------------------------------------------- \n");
	// Check the changed Axis Start Speed Parameter again.
	nRtn = FAS_GetParameter(nPortID, iSlaveNo, SERVO_AXISSTARTSPEED, &lParamVal);
	if (nRtn != FMM_OK)
	{
		printf("Function(FAS_GetParameter) was failed.\n");
		return false;
	}
	else
	{
		printf("Load Parameter[After] : Start Speed = %d[pps] \n", lParamVal);
	}

	return true;
}
