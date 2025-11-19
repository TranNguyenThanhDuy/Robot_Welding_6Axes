// Simple example: connect, servo ON, wait, servo OFF, close
#include <iostream>
#include <string>
#include <unistd.h>
#include "DriverConnection.h"

int main()
{
    const wchar_t* sPort = L"ttyUSB0";
    unsigned int dwBaudRate = 115200;
    int nPortID = 0;
    unsigned char iSlaveNo = 0;

    if (!Connect(sPort, dwBaudRate, nPortID))
    {
        printf("Connection Fail!\n");
        return 1;
    }

    if (!CheckDriveInfo(sPort, nPortID, iSlaveNo))
    {
        printf("CheckDriveInfo Fail!\n");
        FAS_Close(nPortID);
        return 1;
    }

    if (!ServoOn(nPortID, iSlaveNo))
    {
        printf("ServoOn Fail!\n");
        FAS_Close(nPortID);
        return 1;
    }

    // Keep servo ON for a short duration
    usleep(1000 * 1000);

    if (!ServoOff(nPortID, iSlaveNo))
    {
        printf("ServoOff Fail!\n");
        FAS_Close(nPortID);
        return 1;
    }

    FAS_Close(nPortID);
    return 0;
}

