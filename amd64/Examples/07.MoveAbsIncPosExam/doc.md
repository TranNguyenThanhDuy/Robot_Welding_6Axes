# MoveAbsIncPosExam

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
4. Operate the motor with incremental position.
5. Operate the motor with absolute position.
6. Close connection.

[KR]  
1. 장치 연결.
2. 드라이브 에러 체크.
3. Servo Enable.
4. 상대 위치 이동.
5. 절대 위치 이동.
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

## 3. Move increase position
``` c++
int lIncPos = 370000;
int lVelocity = 40000;

if (FAS_MoveSingleAxisIncPos(nPortID, iSlaveNo, lIncPos, lVelocity) != FMM_OK)
{
	printf("Function(FAS_MoveSingleAxisIncPos) was failed.\n");
	return false;
}
```
[EN]  
You can use the FAS_MoveSingleAxisIncPos() function to operate the motor with [incremental position, speed].

[KR]  
FAS_MoveSingleAxisIncPos() 함수를 사용하여 모터의 [상대 위치, 속도]으로 동작시킬 수 있습니다.

## 4. Move absolute position
``` c++
int lAbsPos = 0;
int lVelocity = 40000;

// Move the motor by 0 pulse (target position : Absolute position)
if (FAS_MoveSingleAxisAbsPos(nPortID, iSlaveNo, lAbsPos, lVelocity) != FMM_OK)
{
	printf("Function(FAS_MoveSingleAxisAbsPos) was failed.\n");
	return false;
}
```
[EN]  
You can use the FAS_MoveSingleAxisAbsPos() function to operate the motor with [absolute position, speed].

[KR]  
FAS_MoveSingleAxisAbsPos() 함수를 사용하여 모터의 [절대위치, 속도]으로 동작시킬 수 있습니다.

## 5. Watch drive status
``` c++
EZISERVO_AXISSTATUS AxisStatus;

// Check the Axis status until motor stops and the Inposition value is checked
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

[KR]  
드라이브의 운전 상태값을 나타내는 FAS_GetAxisStatus() 함수를 사용하여 사용자가 원하는 특정상태 값이 확인 될 때까지 대기할 수 있습니다.

### 5.1 Axis status
[EN]  
EZISERVO_AXISSTATUS is a structure that organizes the drive status values ​​and can be checked in the header file (MOTION_EziSERVO_DEFINE.h).
In addition, the user can check whether the motor operation is completed with the values ​​FFLAG_MOTIONING ('0') and FFLAG_INPOSITION ('1').

[KR]  
EZISERVO_AXISSTATUS 는 드라이브 상태값이 정리된 구조체이며 헤더파일 (MOTION_EziSERVO_DEFINE.h)에서 확인하실 수 있습니다.
더하여 모터의 동작 후 정지완료 상태는 FFLAG_MOTIONING ('0') 및 FFLAG_INPOSITION('1') 값으로 확인 할 수 있습니다.

## 6. Etc
[EN]  
1. Please refer to the [01.ConnectionExam] project document for description of functions related to connecting and disconnecting devices.

[KR]  
1. 장치 연결 및 해제에 관한 기능 설명은 [01.ConnectionExam] 프로젝트 문서를 참고하시기 바랍니다.
