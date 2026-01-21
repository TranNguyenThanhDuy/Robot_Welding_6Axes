# ConnectionExam

Notification before use
-------------------------------------------------------
Depending on the type of product you are using, the definitions of 'Parameter', 'IO Logic', 'AxisStatus', etc. may be different.
This example is based on 'Ezi-SERVO', so please apply the appropriate value depending on the product you are using.

```
Example)

FM_EZISERVO_PARAM			// Parameter enum when using 'Ezi-SERVO'	
FM_EZIMOTIONLINK_PARAM		// Parameter enum when using 'Ezi-MOTIONLINK'
```

## 0. Program scenario
[EN]  
1. Connect a device.
2. Get product information.
3. Close connection.

[KR]  
1. 장치 연결.
2. 제품 종류 확인.
3. 연결 해제.

## 1. Connect
``` c++
const wchar_t* sPort = L"ttyUSB0";
unsigned int dwBaudRate = 115200;
int nPortID = 0;

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
```
[EN]  
'FAS_Connect' is a function for RS-485 connection through PortNo and BaudRate.
If the function returns 'TRUE', the connection is successful.

[KR]  
'FAS_Connect'는 PortNo와 BaudRate를 통해 RS-485 통신 연결을 하는 함수입니다.
함수가 'TRUE'를 리턴하면 연결이 성공한 것입니다.

## 2. Get product info
``` c++ 
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

```
[EN]  
After successful connection, a function 'FAS_GetSlaveInfo' was used to check product information. 
This function is not required for connection, but to just show an example of using a function after successful connection.

[KR]  
연결 성공 후 제품의 정보를 확인하는 함수(FAS_GetSlaveInfo)를 사용하였습니다.
이 함수는 연결에 필요한 함수가 아닙니다. 연결 후 임의의 함수를 호출한 것입니다.

## 3. Connection close
``` c++
// Connection Close
FAS_Close(nPortID);
```
[EN]  
Use 'FAS_Close' to close the connection.

[KR]  
'FAS_Close()'를 통해 연결을 해제할 수 있습니다.
