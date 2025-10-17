# JogMovementExam

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
4. Operate the motor clockwise for a certain period of time (5 seconds) and then stops.
5. Operate the motor counterclockwise for a certain period of time (5 seconds) and then stops.
6. Close connection.

[KR]  
1. 장치 연결.
2. 드라이브 에러 체크.
3. Servo Enable.
4. 일정 시간동안(5초) 시계방향으로 모터 동작후 정지.
5. 일정 시간동안(5초) 반시계방향으로 모터 동작후 정지.
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

### 1.1 Axis Status
[EN]  
EZISERVO_AXISSTATUS is a structure that organizes drive status values.
It can be checked in the header file (MOTION_EziSERVO_DEFINE.h).

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

## 3. Watch drive status
``` c++
EZISERVO_AXISSTATUS AxisStatus;

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
```
[EN]  
The FAS_GetAxisStatus() function indicates the drive status value.
The user can use it to check the status and wait until a specific status value is confirmed.

[KR]  
드라이브의 운전 상태값을 나타내는 FAS_GetAxisStatus() 함수를 사용하여 사용자가 원하는 특정상태 값이 확인 될 때까지 대기할 수 있습니다.

## 4. Motor motion test
``` c++
int nTargetVeloc = 10000;
int nDirect = nDir;
// #define CW  1
// #define CCW 0

if (FAS_MoveVelocity(nPortID, iSlaveNo, nTargetVeloc, nDirect) != FMM_OK)
{
	printf("Function(FAS_MoveVelocity) was failed.\n");
	return false;
}
```
[EN]  
The FAS_MoveVelocity() function allows the user to operate the motor at the desired [speed, direction].

[KR]  
FAS_MoveVelocity() 함수를 사용하여 사용자가 원하는 모터의 [속도, 방향]으로 동작할 수 있습니다.

## 5. Stop motor
``` c++
if (FAS_MoveStop(nPortID, iSlaveNo) != FMM_OK)
{
	printf("Function(FAS_MoveStop) was failed.\n");
	return false;
}
```
[EN]  
You can stop a running motor using the FAS_MoveStop() function.

[KR]  
FAS_MoveStop() 함수를 사용하여 동작중인 모터를 정지 할 수 있습니다.

## 6. Etc
[EN]  
1. Please refer to the [01.ConnectionExam] project document for description of functions related to connecting and disconnecting devices.

[KR]  
1. 장치 연결 및 해제에 관한 기능 설명은 [01.ConnectionExam] 프로젝트 문서를 참고하시기 바랍니다.
