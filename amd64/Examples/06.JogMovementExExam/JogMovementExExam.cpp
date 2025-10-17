#include <iostream>
#include <unistd.h>
#include "../Include/FAS_EziMOTIONPlusR.h"

bool Connect(const wchar_t* sPort, unsigned int dwBaudRate, int nPortID);
bool CheckDriveErr(int nPortID, unsigned char iSlaveNo);
bool SetServoOn(int nPortID, unsigned char iSlaveNo);
bool JogMove(int nPortID, unsigned char iSlaveNo, int nAccDecTime);
bool JogExMove(int nPortID, unsigned char iSlaveNo, int nAccDecTime);

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

	// MoveVelocity with AccDecTime = 100
	if (JogMove(nPortID, iSlaveNo, 100) == false)
	{
		getchar();
		exit(1);
	}

	// MoveVelocity with AccDecTime = 200
	if (JogMove(nPortID, iSlaveNo, 200) == false)
	{
		getchar();
		exit(1);
	}

	// MoveVelocity with AccDecTime = 300
	if (JogMove(nPortID, iSlaveNo, 300) == false)
	{
		getchar();
		exit(1);
	}

	// MoveVelocityEx with AccDecTime = 100
	if (JogExMove(nPortID, iSlaveNo, 100) == false)
	{
		getchar();
		exit(1);
	}

	// MoveVelocityEx with AccDecTime = 200
	if (JogExMove(nPortID, iSlaveNo, 200) == false)
	{
		getchar();
		exit(1);
	}

	// MoveVelocityEx with AccDecTime = 300
	if (JogExMove(nPortID, iSlaveNo, 300) == false)
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

bool JogMove(int nPortID, unsigned char iSlaveNo, int nAccDecTime)
{
	// SetParameter & MoveVelocity
	int nTargetVeloc = 10000;
	int nDirect = 1;
	int nSeconds = 3 * 1000; // Wait 3 Sec

	EZISERVO_AXISSTATUS AxisStatus;

	// Set Jog Acc/Dec Time
	if (FAS_SetParameter(nPortID, iSlaveNo, SERVO_JOGACCDECTIME, nAccDecTime) != FMM_OK)
	{
		printf("Function(FAS_SetParameter) was failed.\n");
		return false;
	}

	printf("---------------------------\n");
	if (FAS_MoveVelocity(nPortID, iSlaveNo, nTargetVeloc, nDirect) != FMM_OK)
	{
		printf("Function(FAS_MoveVelocity) was failed.\n");
		return false;
	}

	printf("Move Motor(Jog Mode)! \n");

	usleep(nSeconds * 1000);

	if (FAS_MoveStop(nPortID, iSlaveNo) != FMM_OK)
	{
		printf("Function(FAS_MoveStop) was failed.\n");
		return false;
	}

	do
	{
		// Wait until FFLAG_MOTIONING is OFF
		usleep(1 * 1000);

		if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(AxisStatus.dwValue)) != FMM_OK)
		{
			printf("Function(FAS_GetAxisStatus) was failed.\n");
			return false;
		}

		if (AxisStatus.FFLAG_MOTIONING == 0)
		{
			printf("Move Stop! \n");
		}
	} while (AxisStatus.FFLAG_MOTIONING);

	return true;
}

bool JogExMove(int nPortID, unsigned char iSlaveNo, int nAccDecTime)
{
	// MoveVelocityEx

	// Set velocity
	int nDirect = 1;
	int lVelocity = 30000;
	int nSeconds = 3 * 1000; // Wait 3 Sec

	EZISERVO_AXISSTATUS AxisStatus;
	VELOCITY_OPTION_EX opt = { 0 };

	// set user setting bit(BIT_USE_CUSTOMACCDEC) and acel/decel time value
	opt.flagOption.BIT_USE_CUSTOMACCDEC = 1;
	opt.wCustomAccDecTime = nAccDecTime;

	printf("-----------------------------------------------------------\n");
	if (FAS_MoveVelocityEx(nPortID, iSlaveNo, lVelocity, nDirect, &opt) != FMM_OK)
	{
		printf("Function(FAS_MoveVelocityEx) was failed.\n");
		return false;
	}

	printf("Move Motor(Jog Ex Mode)! \n");

	usleep(nSeconds * 1000);

	if (FAS_MoveStop(nPortID, iSlaveNo) != FMM_OK)
	{
		printf("Function(FAS_MoveStop) was failed.\n");
		return false;
	}

	do
	{
		// Wait until FFLAG_MOTIONING is OFF
		usleep(1 * 1000);

		if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(AxisStatus.dwValue)) != FMM_OK)
		{
			printf("Function(FAS_GetAxisStatus) was failed.\n");
			return false;
		}

		if (AxisStatus.FFLAG_MOTIONING == 0)
		{
			printf("Move Stop! \n");
		}
	} while (AxisStatus.FFLAG_MOTIONING);

	return true;
}
