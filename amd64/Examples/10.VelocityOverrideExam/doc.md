# VelocityOverrideExam

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
4. Operate the motor with incremental position, and when it reaches a specific point, change the motor speed and drive the motor.
5. Operate the motor with jog motion, and after a certain period of time, change the motor speed and drive the motor.
6. Close connection.

[KR]  
1. 장치 연결.
2. 드라이브 에러 체크.
3. Servo Enable.
4. 상대 위치 이동 운전으로 모터를 구동시키며 사용자가 지정한 특정 지점에 도달하게 된다면 기존의 모터 속도를 변경하여 구동시킵니다.
5. 기본 모터 구동 운전을 진행하며 일정 시간이 지나면 사용자가 지정한 속도로 변화시키며 운전을 진행합니다.
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

## 3. Get actual position
``` c++
int lActualPos = 0;

if (FAS_GetActualPos(nPortID, iSlaveNo, &lActualPos) != FMM_OK)
{
	print("Function(FAS_GetActualPos) was failed.\n");
	return false;
}
```
[EN]  
You can check the current absolute position of the motor using the FAS_GetActualPos() function.

[KR]  
FAS_GetActualPos() 함수를 사용하여 현재 모터의 절대위치값을 파악할 수 있습니다.

## 4. Get actual velocity
``` c++
int lActualVelocity = 0;

if (FAS_GetActualVel(nPortID, iSlaveNo, &lActualVelocity) != FMM_OK)
{
	printf("Function(FAS_GetActualVel) was failed. \n");
	return false;
}
```
[EN]  
You can check the current motor speed using the FAS_GetActualVel() function.

[KR]  
FAS_GetActualVel() 함수를 사용하여 현재 모터의 속값을 파악할 수 있습니다.

## 5. Velocity override
``` c++
int lVelocity = 20000;
int lChangeVelocity = 0;

lChangeVelocity = lVelocity * 2;
if (FAS_VelocityOverride(nPortID, iSlaveNo, lChangeVelocity) != FMM_OK)
{
	printf("Function(FAS_VelocityOverride) was failed. \n");
	return false;
}
```
[EN]  
The FAS_VelocityOverride() function can be used to change the speed value during operation.

[KR]  
FAS_VelocityOverride() 함수를 사용하여 모터에 이미 설정된 속도값을 변경하는데 사용할 수 있습니다.

## 6. Stop Motor
``` c++
FAS_MoveStop(nPortID, iSlaveNo);
printf("FAS_MoveStop! \n");
```
[EN]  
You can stop a running motor using the FAS_MoveStop() function.

[KR]  
FAS_MoveStop() 함수를 사용하여 동작중인 모터를 정지 할 수 있습니다.

## 7. Etc
[EN]  
1. For function descriptions on device connection and disconnection, please refer to the [01.ConnectionExam] project document.
2. For function descriptions on basic motor drive operation, please refer to the [05.JogMovementExam] project document.
3. For function descriptions on relative position movement operation, please refer to the [07.MoveAbsIncPosExam] project document.

[KR]  
1. 장치 연결 및 해제에 대한 함수 설명은 [01.ConnectionExam] 프로젝트 문서를 참고하시기 바랍니다.
2. 기본 모터 구동 운전에 대한 함수 설명은 [05.JogMovementExam] 프로젝트 문서를 참고하시기 바랍니다.
3. 상대 위치 이동 운전에 대한 함수 설명은 [07.MoveAbsIncPosExam] 프로젝트 문서를 참고하시기 바랍니다.
