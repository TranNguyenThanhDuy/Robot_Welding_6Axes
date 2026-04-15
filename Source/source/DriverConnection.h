#pragma once

#include <string>
#include "../lib/FAS_EziMOTIONPlusR.h"

// Function prototypes
bool Connect(const wchar_t* sPort, unsigned int dwBaudRate, int nPortID);
bool CheckDriveInfo(const wchar_t* sPort, int nPortID, unsigned char iSlaveNo);
bool Driver_Connection(const wchar_t* sPort, unsigned int dwBaudRate, int nPortID,
                       unsigned char iSlaveNo);
bool ServoOn(int nPortID, unsigned char iSlaveNo);
bool ServoOff(int nPortID, unsigned char iSlaveNo);
