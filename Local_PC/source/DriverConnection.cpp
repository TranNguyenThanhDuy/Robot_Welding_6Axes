#include "DriverConnection.h"
#include <iostream>
#include <string>
#include <unistd.h>

bool Connect(const wchar_t* sPort, unsigned int dwBaudRate, int nPortID)
{
    bool bSuccess = false;

    if (FAS_Connect(sPort, dwBaudRate, nPortID) == false)
    {
        std::cout << "Connection Fail!" << std::endl;
        bSuccess = false;
    }
    else
    {
        std::cout << "Connection Success!" << std::endl;
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
        std::cout << "Function(FAS_GetSlaveInfo) was failed." << std::endl;
        return false;
    }

    std::string portStr;
    for (const wchar_t* p = sPort; *p; ++p) {
        portStr.push_back(static_cast<char>(*p));
    }
    std::cout << "Port " << portStr << " / Slave No " << static_cast<int>(iSlaveNo)
              << " : TYPE= " << static_cast<int>(byType) << ", Version= " << IpBuff << std::endl;

    return true;
}

bool Driver_Connection(const wchar_t* sPort, unsigned int dwBaudRate, int nPortID,
                       unsigned char iSlaveNo)
{
    // Device Connect
    if (Connect(sPort, dwBaudRate, nPortID) == false)
    {
        return false;
    }

    // Check Drive information
    // if (CheckDriveInfo(sPort, nPortID, iSlaveNo) == false)
    // {
    //     getchar();
    //     exit(1);
    // }

    // Keep connection open; caller is responsible for FAS_Close(nPortID)
    return true;
}

bool ServoOn(int nPortID, unsigned char iSlaveNo)
{
    EZISERVO_AXISSTATUS AxisStatus;

    if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(AxisStatus.dwValue)) != FMM_OK)
    {
        std::cout << "Function(FAS_GetAxisStatus) was failed." << std::endl;
        return false;
    }

    if (AxisStatus.FFLAG_SERVOON == 0)
    {
        if (FAS_ServoEnable(nPortID, iSlaveNo, 1) != FMM_OK)
        {
            std::cout << "Function(FAS_ServoEnable) was failed." << std::endl;
            return false;
        }

        do
        {
            usleep(1 * 1000);

            if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(AxisStatus.dwValue)) != FMM_OK)
            {
                std::cout << "Function(FAS_GetAxisStatus) was failed." << std::endl;
                return false;
            }

            if (AxisStatus.FFLAG_SERVOON)
                std::cout << "Servo ON" << std::endl;
        } while (!AxisStatus.FFLAG_SERVOON);
    }
    else
    {
        std::cout << "Servo is already ON" << std::endl;
    }

    return true;
}

bool ServoOff(int nPortID, unsigned char iSlaveNo)
{
    EZISERVO_AXISSTATUS AxisStatus;

    if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(AxisStatus.dwValue)) != FMM_OK)
    {
        std::cout << "Function(FAS_GetAxisStatus) was failed." << std::endl;
        return false;
    }

    if (AxisStatus.FFLAG_SERVOON != 0)
    {
        if (FAS_ServoEnable(nPortID, iSlaveNo, 0) != FMM_OK)
        {
            std::cout << "Function(FAS_ServoEnable) was failed." << std::endl;
            return false;
        }

        do
        {
            usleep(1 * 1000);

            if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(AxisStatus.dwValue)) != FMM_OK)
            {
                std::cout << "Function(FAS_GetAxisStatus) was failed." << std::endl;
                return false;
            }

            if (!AxisStatus.FFLAG_SERVOON)
                std::cout << "Servo OFF" << std::endl;
        } while (AxisStatus.FFLAG_SERVOON);
    }
    else
    {
        std::cout << "Servo is already OFF" << std::endl;
    }

    return true;
}
