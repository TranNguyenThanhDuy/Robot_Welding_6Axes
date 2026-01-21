#include <iostream>
#include <unistd.h>
#include "../Include/FAS_EziMOTIONPlusR.h"

#define CW  1
#define CCW 0

bool Connect(const wchar_t* sPort, unsigned int dwBaudRate, int nPortID);
bool CheckDriveErr(int nPortID, unsigned char iSlaveNo);
bool SetServoOn(int nPortID, unsigned char iSlaveNo);
bool SetOriginParameter(int nPortID, unsigned char iSlaveNo);
bool OriginSearch(int nPortID, unsigned char iSlaveNo);

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
	if (SetOriginParameter(nPortID, iSlaveNo) == false)
	{
		getchar();
		exit(1);
	}

	// MoveVelocity  & VelocityOverride
	if (OriginSearch(nPortID, iSlaveNo) == false)
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

bool SetOriginParameter(int nPortID, unsigned char iSlaveNo)
{
	// Set Origin Parameter
	int nOrgSpeed = 50000;
	int nOrgSearchSpeed = 1000;
	int nOrgAccDecTime = 50;
	int nOrgMethod = 2;	// Origin Method = 2 is 'Limit Origin' in the Ezi-SERVO Plus-R model.
	int nOrgDir = CW;
	int nOrgOffset = 0;
	int nOrgPositionSet = 0;
	int nOrgTorqueRatio = 50;

	if (FAS_SetParameter(nPortID, iSlaveNo, SERVO_ORGSPEED, nOrgSpeed) != FMM_OK)
	{
		printf("Function(FAS_SetParameter[SERVO_ORGSPEED]) was failed.\n");
		return false;
	}

	if (FAS_SetParameter(nPortID, iSlaveNo, SERVO_ORGSEARCHSPEED, nOrgSearchSpeed) != FMM_OK)
	{
		printf("Function(FAS_SetParameter[SERVO_ORGSEARCHSPEED]) was failed.\n");
		return false;
	}

	if (FAS_SetParameter(nPortID, iSlaveNo, SERVO_ORGACCDECTIME, nOrgAccDecTime) != FMM_OK)
	{
		printf("Function(FAS_SetParameter[SERVO_ORGACCDECTIME]) was failed.\n");
		return false;
	}

	if (FAS_SetParameter(nPortID, iSlaveNo, SERVO_ORGMETHOD, nOrgMethod) != FMM_OK)
	{
		printf("Function(FAS_SetParameter[SERVO_ORGMETHOD]) was failed.\n");
		return false;
	}

	if (FAS_SetParameter(nPortID, iSlaveNo, SERVO_ORGDIR, nOrgDir) != FMM_OK)
	{
		printf("Function(FAS_SetParameter[SERVO_ORGDIR]) was failed.\n");
		return false;
	}

	if (FAS_SetParameter(nPortID, iSlaveNo, SERVO_ORGOFFSET, nOrgOffset) != FMM_OK)
	{
		printf("Function(FAS_SetParameter[SERVO_ORGOFFSET]) was failed.\n");
		return false;
	}

	if (FAS_SetParameter(nPortID, iSlaveNo, SERVO_ORGPOSITIONSET, nOrgPositionSet) != FMM_OK)
	{
		printf("Function(FAS_SetParameter[SERVO_ORGPOSITIONSET]) was failed.\n");
		return false;
	}

	if (FAS_SetParameter(nPortID, iSlaveNo, SERVO_ORGTORQUERATIO, nOrgTorqueRatio) != FMM_OK)
	{
		printf("Function(FAS_SetParameter[SERVO_ORGTORQUERATIO]) was failed.\n");
		return false;
	}

	return true;
}

bool OriginSearch(int nPortID, unsigned char iSlaveNo)
{
	// Act Origin Search Function
	EZISERVO_AXISSTATUS AxisStatus;

	printf("---------------------------\n");

	// Origin Search Start
	if (FAS_MoveOriginSingleAxis(nPortID, iSlaveNo) != FMM_OK)
	{
		printf("Function(FAS_MoveOriginSingleAxis) was failed.\n");
		return false;
	}

	// Check the Axis status until OriginReturning value is released.
	do
	{
		usleep(1 * 1000);

		if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(AxisStatus.dwValue)) != FMM_OK)
		{
			printf("Function(FAS_GetAxisStatus) was failed.\n");
			return false;
		}

	} while (AxisStatus.FFLAG_ORIGINRETURNING);

	if (AxisStatus.FFLAG_ORIGINRETOK)
	{
		printf("Origin Search Success! \n");
		return true;
	}
	else
	{
		printf("Origin Search Fail !\n");
		return false;
	}
}
