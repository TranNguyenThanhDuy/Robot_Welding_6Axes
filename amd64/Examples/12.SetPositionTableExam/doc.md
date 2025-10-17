#SetPositionTableExam

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
4. Configure and check the position table.
5. Operate the motor with the position table.
6. Close connection.

[KR]  
1. 장치 연결.
2. 드라이브 에러 체크.
3. Servo Enable.
4. 포지션 테이블 설정 및 확인 실시.
5. 포지션 테이블 실행.
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

## 3. Set position table
``` c++
// Sets values in the position table.
ITEM_NODE nodeItem;
unsigned short wItemNo = 1;

// Overwrite part of node's data
nodeItem.dwMoveSpd = 50000;
nodeItem.lPosition = 250000;
nodeItem.wBranch = 3;
nodeItem.wContinuous = 1;

// Set node data in Position Table
if (FAS_PosTableWriteItem(nPortID, iSlaveNo, wItemNo, &nodeItem))
{
	printf("Function(FAS_PosTableWriteItem) was failed.\n");
	return false;
}
```
[EN]  
The FAS_PosTableWriteItem() function configures various properties on the indexes in the position table.

[KR]  
FAS_PosTableWriteItem() 함수를 사용하여 포지션 테이블을 구성하는 인덱스에 다양한 속성을 설정할 수 있습니다.

### 3.1 Position table item node
[EN]  
ITEM_NODE is a structure that organizes the attribute values ​​of the indexes in the position table.
It can be checked in the header file (MOTION_DEFINE.h).

[KR]  
ITEM_NODE는 포지션 테이블을 구성하는 인덱스의 속성값이 정리된 구조체이며 헤더파일(MOTION_DEFINE.h)에서 확인하실 수 있습니다.

## 4. Read position table
``` c++
ITEM_NODE nodeItem;
unsigned short wItemNo = 1;

// Gets the data values of that node.
if (FAS_PosTableReadItem(nPortID, iSlaveNo, wItemNo, &nodeItem) != FMM_OK)
{
	printf("Function(FAS_PosTableReadItem) was failed.\n");
	return false;
}
```
[EN]  
FAS_PosTableReadItem() function checks the table properties stored at a specific index in the positions table.

[KR]  
FAS_PosTableReadItem() 함수를 사용하여 포지션 테이블내 특정 인덱스에 저장된 테이블 속성을 확인 할 수 있습니다.

## 5. Run position table
``` c++
unsigned short wItemNo = 1;

if (FAS_PosTableRunItem(nPortID, iSlaveNo, wItemNo) != FMM_OK)
{
	printf("Function(FAS_PosTableRunItem) was failed.\n");
	return false;
}
```
[EN]  
FAS_PosTableRunItem() function operates the motor according to the specific index in the position table.

[KR]  
FAS_PosTableRunItem() 함수를 사용하여 포지션 테이블내 특정 인덱스의 설정값에 맞게 동작시킬수 있습니다.

## 6. Watch drive status
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

	if (AxisStatus.FFLAG_PTSTOPPED == true)
		printf("Position Table Run Stop! \n");
} while (!AxisStatus.FFLAG_PTSTOPPED);
```
[EN]  
The FAS_GetAxisStatus() function indicates the drive status value.
The user can use it to check the status and wait until a specific status value is confirmed.
In addition, the user can check whether the position table operation is completed with the values ​​FFLAG_MOTIONING ('0') and FFLAG_INPOSITION ('1').

[KR]  
드라이브의 운전 상태값을 나타내는 FAS_GetAxisStatus() 함수를 사용하여 사용자가 원하는 특정상태 값이 확인 될 때까지 대기할 수 있습니다.
더하여 포지션 테이블 동작 완료 상태는 FFLAG_ORIGINRETURNING ('0')값으로 확인 할 수 있습니다.

## 7. Etc
[EN]  
1. Please refer to the [01.ConnectionExam] project document for function descriptions on connecting and disconnecting devices.

[KR]  
1. 장치 연결 및 해제에 대한 함수 설명은 [01.ConnectionExam] 프로젝트 문서를 참고하시기 바랍니다.
