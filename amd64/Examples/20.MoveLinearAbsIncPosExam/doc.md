# MoveLinearAbsIncPosExam

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
4. Operate the motor with Linear interpolation move (incremental position).
5. Operate the motor with Linear interpolation move (absolute position).
6. Close connection.

[KR]  
1. 장치 연결.
2. 드라이브 에러 체크.
3. Servo Enable.
4. 직선 보간 이송 (상대 좌표).
5. 직선 보간 이송 (절대 좌표).
6. 연결 해제.

## 1. Check drive error
``` c++
// Check Drive's Error
EZISERVO_AXISSTATUS AxisStatus;

if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(AxisStatus.dwValue)) != FMM_OK)
{
	printf("[Slave : %d] Function(FAS_GetAxisStatus) was failed.\n", iSlaveNo);
	return false;
}

if (AxisStatus.FFLAG_ERRORALL)
{
	// if Drive's Error was detected, Reset the ServoAlarm
	if (FAS_ServoAlarmReset(nPortID, iSlaveNo) != FMM_OK)
	{
		printf("[Slave : %d] Function(FAS_ServoAlarmReset) was failed.\n", iSlaveNo);
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

## 2. Linear interpolation move (relative coordinates).
``` c++
// Move Linear IncPos
int lIncPos[2] = { 370000, 370000 };
unsigned int dwVelocity = 0;
unsigned short wAccelTime = 100;

printf("---------------------------\n");
// Increase the motor by 370000 pulse (target position : Relative position)
dwVelocity = 40000;

printf("[Linear Inc Mode] Move Motor ! \n");

if (FAS_MoveLinearIncPos2(nPortID, SLAVE_CNT, iSlaveNo, lIncPos, dwVelocity, wAccelTime) != FMM_OK)
{
	printf("Function(FAS_MoveLinearIncPos2) was failed.\n");
	return false;
}
```
[EN]  
You can perform incremental position linear interpolation movement using the FAS_MoveLinearIncPos2() function.
Meaning of each argument in the function is as follows sequentially:
'Number of drives to linearly interpolate', 'ID array of drives', 'Coordinate array', 'Speed', 'Acceleration/deceleration time'

[KR]  
FAS_MoveLinearIncPos2() 함수를 사용하여 상대 좌표 직선 보간 이송을 수행할 수 있습니다.
해당 함수의 각 인자는 순차적으로 다음을 의미합니다.
'직선 보간할 Slave 들의 Port번호', '직선 보간할 드라이브의 수', '드라이브들의 SlaveNo배열', '좌표 배열', '속도', '가감속 시간'

## 3. Linear interpolation move (absolute coordinates).
``` c++
// Move Linear AbsPos
int lAbsPos[2] = { 0, 0 };
unsigned int dwVelocity = 0;
unsigned short wAccelTime = 100;

printf("---------------------------\n");

// Move the motor by 0 pulse (target position : Absolute position)
dwVelocity = 40000;

printf("[Linear Abs Mode] Move Motor ! \n");

if (FAS_MoveLinearAbsPos2(nPortID, SLAVE_CNT, iSlaveNo, lAbsPos, dwVelocity, wAccelTime) != FMM_OK)
{
	printf("Function(FAS_MoveLinearAbsPos2) was failed.\n");
	return false;
}
```
[EN]  
You can perform absolute position linear interpolation using the FAS_MoveLinearIncPos2() function. 
It has same argument as 'Incremental position linear interpolation'.

[KR]  
FAS_MoveLinearIncPos2() 함수를 사용하여 절대 좌표 직선 보간 이송을 수행할 수 있습니다.
해당 함수의 각 인자는 '상대 좌표 직선 보간 이송'과 동일합니다.

## 4. Etc
[EN]  
1. For function descriptions on device connection and disconnection, please refer to the [01.ConnectionExam] project document. 
2. For function descriptions on driver error check, please refer to the [05.JogMovementExam] project document.

[KR]  
1. 장치 연결 및 해제에 대한 함수 설명은 [01.ConnectionExam] 프로젝트 문서를 참고하시기 바랍니다.
2. 드라이버 에러 체크에 대한 함수 설명은 [05.JogMovementExam] 프로젝트 문서를 참고하시기 바랍니다.
