#PushStopExam

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
2. Check the drive error.
3. Enable Servo.
4. Operate the motor with Push Mode (Stop Mode).
5. Operate the motor with the return operation.
6. Connection close.

[KR]  
1. 장치 연결.
2. 드라이브 에러 체크.
3. Servo Enable.
4. Push Mode(Stop Mode) 운전 실시.
5. 복귀 운전 실시.
6. 연결 해제.

## 1. Check drive error
``` c++
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
```
[EN]  
You can check the current drive's operating status using the FAS_GetAxisStatus() function. You can reset the current drive's alarm status using the FAS_ServoAlarmReset() function.

[KR]  
FAS_GetAxisStatus() 함수를 사용하여 현재 드라이브의 운전 상태를 확인 할 수 있습니다. FAS_ServoAlarmReset() 함수를 사용하여 현재 드라이브의 알람상태를 리셋 할 수 있습니다.

### 1.1 Axis status
[EN]  
EZISERVO_AXISSTATUS is a structure that organizes drive status values ​​and can be checked in the header file (MOTION_EziSERVO_DEFINE.h).

[KR]  
EZISERVO_AXISSTATUS 는 드라이브 상태값이 정리된 구조체이며 헤더파일 (MOTION_EziSERVO_DEFINE.h)에서 확인하실 수 있습니다.

## 2. Servo Enable
``` c++
if (FAS_ServoEnable(nPortID, iSlaveNo, 1) != FMM_OK)
{
	printf("Function(FAS_ServoEnable) was failed.\n");
	return false;
}
```
[EN]  
You can set the Servo Enable signal of the drive using the FAS_ServoEnable() function.

[KR]  
FAS_ServoEnable() 함수를 사용하여 드라이브의 Servo Enable 신호를 설정할 수 있습니다.

## 3. Move push mode
``` c++
EZISERVO_AXISSTATUS AxisStatus;
unsigned int dwStartSpd, dwMoveSpd;
unsigned short wAccel, wDecel;
int lPosition;

unsigned int dwPushSpd;
unsigned short wPushRate;
unsigned short wPushMode;
int lEndPosition;

// normal position motion
dwStartSpd = 1;
dwMoveSpd = 50000;
wAccel = 500;
wDecel = 500;
lPosition = 500000;

// push motion
dwPushSpd = 2000;
wPushRate = 50;
wPushMode = 0;	// Stop Mode Push
lEndPosition = lPosition + 10000;

printf("---------------------------\n");

if (FAS_MovePush(nPortID, iSlaveNo, dwStartSpd, dwMoveSpd, lPosition, wAccel, wDecel, wPushRate, dwPushSpd, lEndPosition, wPushMode) != FMM_OK)
{
	printf("Function(FAS_MovePush) was failed.\n");
	return false;
}
```
[EN]  
With FAS_MovePush() function, you can operate the motor to a desired position with a specified force.
For your information, there are two operation parts in 'push mode'.
The first one is position control before applying a specified force, and the second one is the force control with the specified force.
The position control part can set [start speed, maximum speed, acceleration/deceleration time, absolute position].
The force control part can set [push force ratio, push operation speed, push operation absolute position, push operation mode].

[KR]  
FAS_MovePush() 함수를 사용하여 특정 위치에서 지정된 힘을 유지하며 이동할 수 있다.
더하여 PUSH MODE 운전은 힘을 가하기 이전 움직임에 대한 설정 [시작속도, 최대속도, 가감속 시간, 절대 위치]을 할 수 있으며, 
힘을 가하는 운전에 대한 설정 [PUSH 힘 비율, Push 운전 속도, PUSH 운전 절대 위치, PUSH 운전 모드]또한 할 수 있습니다.

## 4. Check IO Output
``` c++
unsigned int dwOutput = 0;
EZISERVO_OUTLOGIC outLogic;

if (FAS_GetIOOutput(nPortID, iSlaveNo, &dwOutput) != FMM_OK)
{
	printf("Function(FAS_GetIOOutput) was failed. \n");
	return false;
}
else
{
	outLogic.dwValue = dwOutput;

	if (outLogic.BIT_RESERVED0 == true)
		printf("Work Detected! \n");
	else
		printf("Work Not Detected! \n");
}
```
[EN]  
You can check the status of the IO output pin using the FAS_GetIOOutput() function.

[KR]  
FAS_GetIOOutput() 함수를 통하여 IO 출력핀의 상태를 확인 할 수 있습니다.

### 4.1 Output logic
[EN]  
EZISERVO_OUTLOGIC is a structure that organizes the meaning of IO output status, and when an obstacle is detected in force control mode, the BIT_RESERVED0 signal is output. 
You can check EZISERVO_OUTLOGIC in the header file (MOTION_EziSERVO_DEFINE.h).

[KR]  
EZISERVO_OUTLOGIC는 IO 출력 상태 의미를 정리한 구조체이며, PUSH 운전에서 장애물 감지시 BIT_RESERVED0 신호를 출력하게 됩니다.
EZISERVO_OUTLOGIC는 헤더파일(MOTION_EziSERVO_DEFINE.h)에서 확인하실 수 있습니다.

## 5. Watch drive status
``` c++
EZISERVO_AXISSTATUS AxisStatus;

// Check the Axis status until motor stops and the FFLAG_MOTIONING value is released.
do
{
	usleep(1 * 1000);

	if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(AxisStatus.dwValue)) != FMM_OK)
	{
		printf("Function(FAS_GetAxisStatus) was failed.\n");
		return false;
	}

} while (AxisStatus.FFLAG_MOTIONING || !(AxisStatus.FFLAG_INPOSITION));
```
[EN]  
The FAS_GetAxisStatus() function indicates the drive status value.
The user can use it to check the status and wait until a specific status value is confirmed.
In addition, the user can check whether the push mode operation is completed with the values ​​FFLAG_MOTIONING ('0').

[KR]  
드라이브의 운전 상태값을 나타내는 FAS_GetAxisStatus() 함수를 사용하여 사용자가 원하는 특정상태 값이 확인 될 때까지 대기할 수 있습니다.
더하여 PUSH 모드 동작 완료 상태는 FFLAG_MOTIONING ('0')값으로 확인 할 수 있습니다.

## 6. Etc
[EN]  
1. For function descriptions on device connection and disconnection, please refer to the [01.ConnectionExam] project document. 
2. For function descriptions on relative position movement operation, please refer to the [07.MoveAbsIncPosExam] project document.

[KR]  
1. 장치 연결 및 해제에 대한 함수 설명은 [01.ConnectionExam] 프로젝트 문서를 참고하시기 바랍니다.
2. 상대 위치 이동 운전에 대한 함수 설명은 [07.MoveAbsIncPosExam] 프로젝트 문서를 참고하시기 바랍니다.
