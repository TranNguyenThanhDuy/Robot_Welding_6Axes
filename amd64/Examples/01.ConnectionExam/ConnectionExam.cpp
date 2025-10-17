#include <iostream>
#include "../Include/FAS_EziMOTIONPlusR.h"

bool Connect(const wchar_t* sPort, unsigned int dwBaudRate, int nPortID);
bool CheckDriveInfo(const wchar_t* sPort, int nPortID, unsigned char iSlaveNo);

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

	// Check Drive information
	if (CheckDriveInfo(sPort, nPortID, iSlaveNo) == false)
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

bool CheckDriveInfo(const wchar_t* sPort, int nPortID, unsigned char iSlaveNo)
{
	unsigned char byType = 0;
	char IpBuff[256] = "";
	int nBuffSize = 256;
	int nRtn;

	// Read Drive's information
	nRtn = FAS_GetSlaveInfo(nPortID, iSlaveNo, &byType, IpBuff, nBuffSize);
	if (nRtn != FMM_OK)
	{
		printf("Function(FAS_GetSlaveInfo) was failed.\n");
		return false;
	}

	printf("Port %S / Slave No %d : TYPE= %d, Version= %s\n", sPort, iSlaveNo, byType, IpBuff);

	return true;
}
