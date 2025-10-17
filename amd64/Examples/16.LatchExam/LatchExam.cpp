/*****************************************************************************************************************************************
** Notification before use														**
******************************************************************************************************************************************
** Depending on the type of product you are using, the definitions of Parameter, IO Logic, AxisStatus, etc. may be different.		**
** This example is based on Ezi-SERVO, so please apply the appropriate value depending on the product you are using.			**
******************************************************************************************************************************************
** ex)	FM_EZISERVO_PARAM			// Parameter enum when using Ezi-SERVO							**
**		FM_EZIMOTIONLINK_PARAM		// Parameter enum when using Ezi-MOTIONLINK						**
*****************************************************************************************************************************************/

#include <iostream>
#include <unistd.h>
#include "../Include/FAS_EziMOTIONPlusR.h"

bool Connect(const wchar_t* sPort, unsigned int dwBaudRate, int nPortID);
bool LatchCount(int nPortID, unsigned char iSlaveNo);

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

	// get input pin latch signal, get a specific pin latch count, get all pin latch count, clear latch
	if (LatchCount(nPortID, iSlaveNo) != false)
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

bool LatchCount(int nPortID, unsigned char iSlaveNo)
{
	unsigned int uInput = 0;
	unsigned int uLatch = 0;

	unsigned char cInputNo = 0;	// Pin 0
	unsigned int nLatchCount = 0;

	unsigned int cntLatchAll[16] = { 0 };
	unsigned int uLatchMask;

	printf("Monitor a specific pin Input latch signal while 1 min... \n");

	// Monitor the specific pin input result value.
	for(int i = 0; i < 600; i++)
	{
		// Latch status
		if (FAS_GetInput(nPortID, iSlaveNo, &uInput, &uLatch) != FMM_OK)
		{
			printf("Function(FAS_GetInput) was failed.\n");
			return false;
		}

		// Latch count
		if (FAS_GetLatchCount(nPortID, iSlaveNo, cInputNo, &nLatchCount) != FMM_OK)
		{
			printf("Function(FAS_GetLatchCount) was failed.\n");
			return false;
		}

		printf("Pin %d is %s and %s (latch count %d) \n", cInputNo, ((uInput & (0x01 << cInputNo)) ? "ON" : "OFF"), ((uLatch & (0x01 << cInputNo)) ? "latched" : "not latched"), nLatchCount);
		usleep(100 * 1000);
	}

	// Clear the specific pin's Latch status
	uLatchMask = (0x01 << cInputNo);
	if (FAS_ClearLatch(nPortID, iSlaveNo, uLatchMask) != FMM_OK)
	{
		printf("Function(FAS_ClearLatch) was failed.\n");
		return false;
	}
	else
	{
		printf("FAS_ClearLatch Success! \n");
	}

	// Get Latch status again
	if (FAS_GetInput(nPortID, iSlaveNo, &uInput, &uLatch) != FMM_OK)
	{
		printf("Function(FAS_GetInput) was failed.\n");
		return false;
	}

	printf("Pin %d is %s \n", cInputNo, ((uLatch & (0x01 << cInputNo)) ? "latched" : "not latched"));

	// Get latch counts of all inputs (16 inputs)
	if (FAS_GetLatchCountAll(nPortID, iSlaveNo, cntLatchAll) != FMM_OK)
	{
		printf("Function(FAS_GetLatchCountAll) was failed.\n");
		return false;
	}
	else
	{
		for (int i = 0; i < 16; i++)
		{
			printf("[FAS_GetLatchCountAll]  Pin[%d] : [%d]count \n", i, cntLatchAll[i]);
		}
	}

	// Clear the latch count of the specific pin
	if (FAS_ClearLatchCount(nPortID, iSlaveNo, uLatchMask) != FMM_OK)
	{
		printf("Function(FAS_ClearLatchCount) was failed.\n");
		return false;
	}
	else
	{
		printf("FAS_ClearLatchCount Success! \n");
	}

	return true;
}

