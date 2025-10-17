# MoveAbsIncPosExExam

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
4. Operate the motor with incremental position by setting the acceleration/deceleration parameters.
5. Operate the motor with absolute position by setting the acceleration/deceleration structure variables.
6. Close connection.

[KR]  
1. 장치 연결.
2. 드라이브 에러 체크.
3. Servo Enable.
4. 가감속 파라미터 설정을 통한 모터 상대 위치 이동 동작 실시.
5. 가감속 구조체 변수 설정을 통한 모터 상대 위치 이동 동작 실시.
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

## 3. Move Increase position1
``` c++
int lIncPos = nDistance;
int lVelocity = 40000;

// Move the motor by [nDistance] pulse (target position : Absolute position)
printf("Move Motor! \n");
if (FAS_MoveSingleAxisIncPos(nPortNo, iSlaveNo, lIncPos, lVelocity) != FMM_OK)
{
	printf("Function(FAS_MoveSingleAxisIncPos) was failed.\n");
	return false;
}
```
[EN]  
You can use the FAS_MoveSingleAxisIncPos() function to operate the motor with [incremental position, speed].

[KR]  
FAS_MoveSingleAxisIncPos() 함수를 사용하여 모터의 [상대 위치, 속도]으로 동작시킬 수 있습니다.

## 4. Move Increase position2
``` c++
MOTION_OPTION_EX opt = { 0 };
int lIncPos = nDistance;
int lVelocity = 40000;

opt.flagOption.BIT_USE_CUSTOMACCEL = 1;
opt.flagOption.BIT_USE_CUSTOMDECEL = 1;
opt.wCustomAccelTime = nAccDecTime;
opt.wCustomDecelTime = nAccDecTime;

printf("Move Motor! [Ex]\n");
if (FAS_MoveSingleAxisIncPosEx(nPortID, iSlaveNo, lIncPos, lVelocity, &opt) != FMM_OK)
{
	printf("Function(FAS_MoveSingleAxisIncPosEx) was failed.\n");
	return false;
}
```
[EN]  
You can use the FAS_MoveSingleAxisIncPosEx() function to operate the motor with [incremental position, speed, acceleration/deceleration].

[KR]  
FAS_MoveSingleAxisIncPosEx() 함수를 사용하여 모터의 [상대 위치, 속도, 가감속]으로 동작시킬 수 있습니다.

### 4.1 Motion option
[EN]  
MOTION_OPTION_EX is a structure variable that can set the acceleration/deceleration time when the motor is operating. Information about the structure variable can be found in the header file (MOTION_DEFINE.h).

[KR]  
MOTION_OPTION_EX는 모터 동작시 가감속 시간을 설정할 수 있는 구조체 변수입니다. 구조체 변수에 대한 정보는 헤더파일(MOTION_DEFINE.h)에서 확인하실 수 있습니다.

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
In addition, the user can check whether the motor operation is completed with the values ​​FFLAG_MOTIONING ('0') and FFLAG_INPOSITION ('1').

[KR]  
드라이브의 운전 상태값을 나타내는 FAS_GetAxisStatus() 함수를 사용하여 사용자가 원하는 특정상태 값이 확인 될 때까지 대기할 수 있습니다.
더하여 모터의 동작 후 정지완료 상태는 FFLAG_MOTIONING ('0') 및 FFLAG_INPOSITION('1') 값으로 확인 할 수 있습니다.

### 5.1 Axis status
[EN]  
EZISERVO_AXISSTATUS is a structure that organizes the drive status values ​​and can be checked in the header file (MOTION_EziSERVO_DEFINE.h). 
In addition, the stop completion status after motor operation can be checked with the FFLAG_MOTIONING ('0') and FFLAG_INPOSITION ('1') values.

[KR]  
EZISERVO_AXISSTATUS 는 드라이브 상태값이 정리된 구조체이며 헤더파일 (MOTION_EziSERVO_DEFINE.h)에서 확인하실 수 있습니다. 
더하여 모터의 동작 후 정지완료 상태는 FFLAG_MOTIONING ('0') 및 FFLAG_INPOSITION('1') 값으로 확인 할 수 있습니다.

## 6. Etc
[EN]  
1. For function descriptions on connecting and disconnecting devices, please refer to the [01.ConnectionExam] project document. 
2. For function descriptions on setting parameters, please refer to the [02.ParameterTestExam] project document.

[KR]  
1. 장치 연결 및 해제에 대한 함수 설명은 [01.ConnectionExam] 프로젝트 문서를 참고하시기 바랍니다.
2. 파라미터 설정에 대한 함수 설명은 [02.ParameterTestExam] 프로젝트 문서를 참고하시기 바랍니다.
