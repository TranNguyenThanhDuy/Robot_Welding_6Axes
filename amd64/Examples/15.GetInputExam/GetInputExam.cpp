/*****************************************************************************************************************************************
** Notification before use														**
******************************************************************************************************************************************
** Depending on the type of product you are using, the definitions of Parameter, IO Logic, AxisStatus, etc. may be different.		**
** This example is based on Ezi-SERVO, so please apply the appropriate value depending on the product you are using.			**
******************************************************************************************************************************************
** ex)	FM_EZISERVO_PARAM			// Parameter enum when using Ezi-SERVO							**
**		FM_EZIMOTIONLINK_PARAM		// Parameter enum when using Ezi-MOTIONLINK						**
******************************************************************************************************************************************/

#include <iostream>
#include "../Include/FAS_EziMOTIONPlusR.h"

bool Connect(const wchar_t* sPort, unsigned int dwBaudRate, int nPortID);
bool GetInput(int nPortNo, unsigned char iSlaveNo);

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

	// Get Input Status
	if (GetInput(nPortID, iSlaveNo) == false)
	{
		getchar();
		exit(1);
	}

	//Connection Close
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

bool GetInput(int nPortID, unsigned char iSlaveNo)
{
	unsigned int uInput = 0;
	unsigned int uLatch = 0;
	bool bON;

	if (FAS_GetInput(nPortID, iSlaveNo, &uInput, &uLatch) != FMM_OK)
	{
		printf("Function(FAS_GetInput) was failed.\n");
		return false;
	}

	printf("FAS_GetInput Success! \n");

	for (int i = 0; i < 16; i++)
	{
		bON = ((uInput & (0x01 << i)) != 0);
		printf("Input bit %d is %s.\n", i, ((bON) ? "ON" : "OFF"));
	}

	return true;
}
