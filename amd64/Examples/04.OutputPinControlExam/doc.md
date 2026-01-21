# OutputPinControlExam

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
2. Configure output pin setting.
3. Check the output pin setting.
4. Control the output signals (ON / OFF) for a certain period of time (1 minute).
5. Close connection.

[KR]  
1. 장치 연결.
2. Output PIN 설정.
3. Output PIN 설정 확인.
4. 일정 시간(1분) 동안 출력PIN 제어(ON / OFF).
5. 연결 해제.

## 1. Check IO Map
``` c++
unsigned char byLevel = 0;
unsigned int dwLogicMask = 0;

// Check OutputPin Status
for (int i = 0; i < OUTPUTPIN; i++)
{
	if (FAS_GetIOAssignMap(nPortID, iSlaveNo, INPUTPIN + i, &dwLogicMask, &byLevel) != FMM_OK)
	{
		printf("Function(FAS_GetIOAssignMap) was failed.\n");
		return false;
	}

	if (dwLogicMask != IN_LOGIC_NONE)
		printf("Output Pin[%d] : Logic Mask 0x%08x (%s) \n", i, dwLogicMask, (byLevel == LEVEL_LOW_ACTIVE) ? "Low Active" : "High Active");
	else
		printf("Output Pin[%d] : Not Assigned \n", i);
}
```
[EN]  
You can use the FAS_GetIOAssignMap() function to check the function and operation level status values ​​for a specific pin of the drive's I/O connector (CN1).

[KR]  
FAS_GetIOAssignMap() 함수를 사용하여 드라이브의 I/O 커넥터(CN1)의 특정 핀에 대한 기능 및 동작 Level 상태값을 확인 할 수 있다.

### 1.1 Check logic value
[EN]  
Please check the header file (MOTION_DEFINE.h) for the definition of comparison data to check the validity of the Logic value of a specific pin.

[KR]  
특정 핀의 Logic값 유효성을 확인할 비교 데이터는 헤더파일(MOTION_DEFINE.h)에 정의되어 있습니다.

## 2. Output PIN Control
``` c++
// Set Output pin Value.
unsigned char byPinNo = 3;
unsigned char byLevel = LEVEL_HIGH_ACTIVE;
unsigned int dwOutputMask = SERVO_OUT_BITMASK_USEROUT0;

if (FAS_SetIOAssignMap(nPortID, iSlaveNo, INPUTPIN + byPinNo, dwOutputMask, byLevel) != FMM_OK)
{
	printf("Function(FAS_SetIOAssignMap) was failed.\n");
	return false;
}
```
[EN]  
You can use the FAS_SetIOAssignMap() function to set the Logic value and operation Level value for a specific pin of CN1.

[KR]  
FAS_SetIOAssignMap()함수를 사용하여 CN1의 특정핀에 대한 Logic값 및 동작 Level 값을 설정 할 수 있습니다.

### 2.1 Check Bit Mask value
[EN]  
The BIT MASK value for the output PIN can be found in the header file (MOTION_EziSERVO_DEFINE.h).

[KR]  
출력 PIN에 매칭될 BIT MASK값은 헤더파일(MOTION_EziSERVO_DEFINE.h)에서 확인하실 수 있습니다.

## 3.Output Pin control
``` c++
unsigned int dwOutputMask = SERVO_OUT_BITMASK_USEROUT0;

// Control output signal on and off for 60 seconds
do
{
	usleep(1000);
	// USEROUT0: ON
	if (FAS_SetIOOutput(nPortID, iSlaveNo, dwOutputMask, 0) != FMM_OK)
	{
		printf("Function(FAS_SetIOOutput) was failed.\n");
		return false;
	}

	usleep(1000);
	// USEROUT0: OFF
	if (FAS_SetIOOutput(nPortID, iSlaveNo, 0, dwOutputMask) != FMM_OK)
	{
		printf("Function(FAS_SetIOOutput) was failed.\n");
		return false;
	}
} while (WaitSeconds(60));
```
[EN]  
You can control (Set & Clear) the output status of a specific pin using the FAS_SetIOOutput() function.

[KR]  
FAS_SetIOOutput() 함수를 사용하여 특정 핀의 출력 상태값을 제어(Set & Clear)할 수 있습니다.

## 4. Etc
[EN]  
1. Please refer to the ConnectionExam project documentation for a description of the functions related to connecting and disconnecting devices.

[KR]  
1. 장치 연결 및 해제에 관한 기능 설명은 ConnectionExam 프로젝트 문서를 참고하시기 바랍니다.
