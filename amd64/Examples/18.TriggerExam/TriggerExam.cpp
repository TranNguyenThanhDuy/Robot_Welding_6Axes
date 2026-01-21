/*****************************************************************************************************************************************
** Notification before use														**
******************************************************************************************************************************************
** Depending on the type of product you are using, the definitions of Parameter, IO Logic, AxisStatus, etc. may be different.		**
** This example is based on Ezi-SERVO, so please apply the appropriate value depending on the product you are using.			**
******************************************************************************************************************************************
** ex)	FM_EZISERVO_PARAM			// Parameter enum when using Ezi-SERVO						 	**
**		FM_EZIMOTIONLINK_PARAM		// Parameter enum when using Ezi-MOTIONLINK						**
*****************************************************************************************************************************************/

#include <iostream>
#include <unistd.h>
#include "../Include/FAS_EziMOTIONPlusR.h"

bool Connect(const wchar_t* sPort, unsigned int dwBaudRate, int nPortID);
bool SetTrigger(int nPortID, unsigned char iSlaveNo);

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

	// Trigger Set and Check Count
	if (SetTrigger(nPortID, iSlaveNo) == false)
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

bool SetTrigger(int nPortID, unsigned char iSlaveNo)
{
	TRIGGER_INFO Trg_Info = { 0 };
	unsigned char cOutputNo;

	unsigned int uRunMask;
	unsigned int uStopMask;
	unsigned int uCount = 0;

	// Set Trigger value.
	cOutputNo = 0;			// Output pin #0
	Trg_Info.wCount = 20;	// Set the number of Trigger outputs
	Trg_Info.wOnTime = 250;	// On Time Setting : 250 [ms]
	Trg_Info.wPeriod = 500;	// Set Trigger period : 500 [ms]

	if (FAS_SetTrigger(nPortID, iSlaveNo, cOutputNo, &Trg_Info) != FMM_OK)
	{
		printf("Function(FAS_SetTrigger) was failed.\n");
		return false;
	}
	else
	{
		printf("FAS_SetTrigger Success! \n");
	}

	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	~~ Fastech's product which has Input (or Output) only (Ezi-IO-I16, Ezi-IO-O16, Ezi-IO-I32, Ezi-IO-O32...), BitMask of Input (or Output) starts from 0x01,	~~
	~~ However, for Input+Output mixed products (Ezi-IO-I8O8, Ezi-IO-I16O16, etc.), the BitMask of Output is allocated after the BitMasks of Input.				~~
	~~ Ezi-IO-I8O8, for example, the BitMask of Input 0 is 0x0001, and the BitMask of Output 0 is 0x0100.														~~
	~~ For detailed allocation methods, please refer to Section 1.3 of the "doc.md" file of this example.														~~
	~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	~~ Fastech의 Input/Output 단일 제품 (Ezi-IO-I16, Ezi-IO-O16, Ezi-IO-I32, Ezi-IO-O32 등)은 Input과 Output의 BitMask가 각각 0x01부터 시작하지만,					~~
	~~ Input+Output 혼합 제품 (Ezi-IO-I8O8, Ezi-IO-I16O16 등)은 Input의 BitMask를 전부 할당한 뒤, Output의 BitMask가 이어서 할당되어있습니다.						~~
	~~ 예를들어 Ezi-IO-I8O8의 Input 0번의 BitMask는 0x0001, Output 0번의 BitMask는 0x0100입니다.																	~~
	~~ 자세한 할당 방식은 본 예제의 "doc.md"파일 1.3절을 참고 부탁드립니다.																							~~
	~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	// Run Output
	uRunMask = (0x01 << cOutputNo); // Run Output Pin #0
	uStopMask = 0x00000000;
	if (FAS_SetRunStop(nPortID, iSlaveNo, uRunMask, uStopMask) != FMM_OK)
	{
		printf("Function(FAS_SetRunStop) was failed.\n");
		return false;
	}
	else
	{
		printf("FAS_SetRunStop Success! \n");
	}

	do
	{
		usleep(100 * 1000);

		if (FAS_GetTriggerCount(nPortID, iSlaveNo, cOutputNo, &uCount) != FMM_OK)
		{
			printf("Function(FAS_GetTriggerCount) was failed.\n");
			return false;
		}
		else
		{
			printf("Get Trigger [%d] count \n", uCount);
		}
	}
	while (uCount < Trg_Info.wCount);

	return true;
}
