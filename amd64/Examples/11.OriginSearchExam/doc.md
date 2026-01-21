#OriginSearchExam

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
4. Configure the parameters related to the origin search.
5. Search the motor origin.
6. Close connection.

[KR]  
1. 장치 연결.
2. 드라이브 에러 체크.
3. Servo Enable.
4. 원점 탐색 관련 파라미터 설정.
5. 모터 원점 탐색 실시.
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

## 3. Set parameter value
``` c++
if (FAS_SetParameter(nPortID, iSlaveNo, SERVO_ORGSPEED, nOrgSpeed) != FMM_OK)
{
	printf("Function(FAS_SetParameter[SERVO_ORGSPEED]) was failed.\n");
	return false;
}

if (FAS_SetParameter(nPortID, iSlaveNo, SERVO_ORGSEARCHSPEED, nOrgSearchSpeed) != FMM_OK)
{
	printf("Function(FAS_SetParameter[SERVO_ORGSEARCHSPEED]) was failed.\n");
	return false;
}

if (FAS_SetParameter(nPortID, iSlaveNo, SERVO_ORGACCDECTIME, nOrgAccDecTime) != FMM_OK)
{
	printf("Function(FAS_SetParameter[SERVO_ORGACCDECTIME]) was failed.\n");
	return false;
}

if (FAS_SetParameter(nPortID, iSlaveNo, SERVO_ORGMETHOD, nOrgMethod) != FMM_OK)
{
	printf("Function(FAS_SetParameter[SERVO_ORGMETHOD]) was failed.\n");
	return false;
}

if (FAS_SetParameter(nPortID, iSlaveNo, SERVO_ORGDIR, nOrgDir) != FMM_OK)
{
	printf("Function(FAS_SetParameter[SERVO_ORGDIR]) was failed.\n");
	return false;
}

if (FAS_SetParameter(nPortID, iSlaveNo, SERVO_ORGOFFSET, nOrgOffset) != FMM_OK)
{
	printf("Function(FAS_SetParameter[SERVO_ORGOFFSET]) was failed.\n");
	return false;
}

if (FAS_SetParameter(nPortID, iSlaveNo, SERVO_ORGPOSITIONSET, nOrgPositionSet) != FMM_OK)
{
	printf("Function(FAS_SetParameter[SERVO_ORGPOSITIONSET]) was failed.\n");
	return false;
}

if (FAS_SetParameter(nPortID, iSlaveNo, SERVO_ORGTORQUERATIO, nOrgTorqueRatio) != FMM_OK)
{
	printf("Function(FAS_SetParameter[SERVO_ORGTORQUERATIO]) was failed.\n");
	return false;
}
```
[EN]  
You can change the parameter value using the 'FAS_SetParameter' function.
The parameter value takes effect immediately after it is set.

[KR]  
FAS_SetParameter 함수를 사용하여 파라미터의 값을 변경할 수 있습니다.
파라미터의 값은 설정하는 즉시 적용됩니다.

### 3.1 Origin method
[EN]  
There are the various parameter numbers about origin search, such as SERVO_ORGSPEED, SERVO_ORGSEARCHSPEED, SERVO_ORGACCDECTIME, etc. 
You can check those parameter numbers in the corresponding header file (MOTION_SERVO_DEFINE.h for Ezi-SERVO Plus-R). 
Or, refer to the number displayed on the Parameter List screen of the Ezi-MOTION Plus-R program.

[KR]  
SERVO_ORGSPEED, SERVO_ORGSEARCHSPEED, SERVO_ORGACCDECTIME... 다양한 원점 탐색에 대한 설정값 파라미터 번호입니다.
설정하고자 하는 파라미터의 번호는 해당하는 header 파일(Ezi-SERVO Plus-R의 경우 MOTION_SERVO_DEFINE.h)에서 확인하실 수 있습니다. 
혹은, Ezi-MOTION Plus-R 프로그램의 Parameter List 화면에 표시되는 번호를 참고하시기 바랍니다.

## 4. Move origin
``` c++
// Origin Search Start
if (FAS_MoveOriginSingleAxis(nPortID, iSlaveNo) != FMM_OK)
{
	printf("Function(FAS_MoveOriginSingleAxis) was failed.\n");
	return false;
}
```
[EN]  
FAS_MoveOriginSingleAxis() function will start origin search with the configured search mode.

[KR]  
FAS_MoveOriginSingleAxis() 함수를 사용하여 사용자가 설정한 탐색 모드값에 따라 탐색이 시작됩니다.

## 5. Watch drive status
``` c++
EZISERVO_AXISSTATUS AxisStatus;

// Check the Axis status until OriginReturning value is released.
do
{
	usleep(1 * 1000);

	if (FAS_GetAxisStatus(nPortID, iSlaveNo, &(AxisStatus.dwValue)) != FMM_OK)
	{
		printf("Function(FAS_GetAxisStatus) was failed.\n");
		return false;
	}

} while (AxisStatus.FFLAG_ORIGINRETURNING);

if (AxisStatus.FFLAG_ORIGINRETOK)
{
	printf("Origin Search Success! \n");
	return true;
}
```
[EN]  
The FAS_GetAxisStatus() function indicates the drive status value.
The user can use it to check the status and wait until a specific status value is confirmed.
In addition, the user can check whether the motor origin search is completed with the values ​​FFLAG_MOTIONING ('0') and FFLAG_INPOSITION ('1').

[KR]  
드라이브의 운전 상태값을 나타내는 FAS_GetAxisStatus() 함수를 사용하여 사용자가 원하는 특정상태 값이 확인 될 때까지 대기할 수 있습니다.
더하여 모터의 원점 탐색 완료 상태는 FFLAG_ORIGINRETURNING ('0') 및 FFLAG_ORIGINRETOK('1') 값으로 확인 할 수 있습니다.

## 6. Etc
[EN]  
1. For function descriptions on connecting and disconnecting devices, please refer to the [01.ConnectionExam] project document. 
2. For function descriptions on setting parameters, please refer to the [02.ParameterTestExam] project document.

[KR]  
1. 장치 연결 및 해제에 대한 함수 설명은 [01.ConnectionExam] 프로젝트 문서를 참고하시기 바랍니다.
2. 파라미터 설정에 대한 함수 설명은 [02.ParameterTestExam] 프로젝트 문서를 참고하시기 바랍니다.
