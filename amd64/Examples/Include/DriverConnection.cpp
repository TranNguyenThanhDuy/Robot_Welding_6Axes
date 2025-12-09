#include "DriverConnection.h"
#include <iostream>
#include <string>
#include <unistd.h>

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

    printf("Port %ls / Slave No %d : TYPE= %d, Version= %s\n", sPort, iSlaveNo, byType, IpBuff);

    return true;
}

void Driver_Connection(const wchar_t* sPort, unsigned int dwBaudRate, int nPortID, unsigned char iSlaveNo)
{
    // Device Connect
    if (Connect(sPort, dwBaudRate, nPortID) == false)
    {
        getchar();
        exit(1);
    }

    // Check Drive information
    // if (CheckDriveInfo(sPort, nPortID, iSlaveNo) == false)
    // {
    //     getchar();
    //     exit(1);
    // }

    // Keep connection open; caller is responsible for FAS_Close(nPortID)
}

bool ServoOn(int nPortID, unsigned char iSlaveNo)
{
    EZISERVO_AXISSTATUS AxisStatus;

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
                printf("Servo ON\n");
        } while (!AxisStatus.FFLAG_SERVOON);
    }
    else
    {
        printf("Servo is already ON\n");
    }

    return true;
}

bool ServoOff(int nPortID, unsigned char iSlaveNo)
{
    EZISERVO_AXISSTATUS AxisStatus;

    if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(AxisStatus.dwValue)) != FMM_OK)
    {
        printf("Function(FAS_GetAxisStatus) was failed.\n");
        return false;
    }

    if (AxisStatus.FFLAG_SERVOON != 0)
    {
        if (FAS_ServoEnable(nPortID, iSlaveNo, 0) != FMM_OK)
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

            if (!AxisStatus.FFLAG_SERVOON)
                printf("Servo OFF\n");
        } while (AxisStatus.FFLAG_SERVOON);
    }
    else
    {
        printf("Servo is already OFF\n");
    }

    return true;
}
