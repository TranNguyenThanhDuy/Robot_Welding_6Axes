# TriggerExam

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
2. Configure the trigger.
3. Run the trigger.
4. Monitor trigger count.
5. Close connection.

[KR]  
1. 장치 연결.
2. Trigger 설정.
3. Trigger 시작.
4. Trigget 횟수 감지.
5. 연결 해제.

## 1. Set Trigger
``` c++
TRIGGER_INFO Trg_Info = {0};
unsigned char cOutputNo;

// Set Trigger value.
cOutputNo = 0;			// Output pin #0
Trg_Info.wCount = 20;	// Set the number of Trigger outputs
Trg_Info.wOnTime = 250;	// On Time Setting : 250 [ms]
Trg_Info.wPeriod = 500;	// Set Trigger period : 500 [ms]

if (FAS_SetTrigger(nPortID, iSlaveNo, cOutputNo, &Trg_Info) != FMM_OK)
{
	printf("Function(FAS_SetTrigger) was failed.\n");
	return false;
}
```
[EN]  
You can configure the Trigger using the FAS_SetTrigger() function.

[KR]  
FAS_SetTrigger() 함수를 사용하여 Trigger에 대한 설정을 할 수 있습니다.

### 1.1 TRIGGER_INFO
[EN]  
TRIGGER_INFO is a structure that contains information about the Trigger settings.

The meaning of each member variable of TRIGGER_INFO is as follows.
wCount is the number of times the Trigger is output.
wOnTime is the length of Trigger On in millisecond.
wPeriod is the period of Trigger On in millisecond.
The above source code turns the Trigger On 20 times for 250ms with a cycle of 500ms.

TRIGGER_INFO is declared in 'MOTION_DEFINE.h'.

[KR]  
TRIGGER_INFO는 Trigger 설정에 대한 정보를 담은 구조체입니다.

TRIGGER_INFO의 각 멤버 변수는 다음을 의미합니다.
wCount는 Trigger 출력 횟수
wOnTime은 Trigger On 시간 설정
wPeriod은 Trigger On 주기
위 소스 코드는 500ms 주기로 250ms동안 20회 Trigger가 On이 됩니다.

TRIGGER_INFO는 MOTION_DEFINE.h에 선언되어 있습니다.

### 1.2 Bitmask logic of Fastech IO product 

||0x0001|0x0002|0x0004|0x0008|0x0010|0x0020|0x0040|0x0080|0x0100|0x0200|0x0400|0x0800|...|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|Ezi-IO-I16|IN0|IN1|IN2|IN3|IN4|IN5|IN6|IN7|IN8|IN9|IN10|IN11|...|
|Ezi-IO-O16|OUT0|OUT1|OUT2|OUT3|OUT4|OUT5|OUT6|OUT7|OUT8|OUT9|OUT10|OUT11|...	|
|Ezi-IO-I8O8|IN0|IN1|IN2|IN3|IN4|IN5|IN6|IN7|OUT0|OUT1|OUT2|OUT3|...|


||0x0001|0x0002|0x0004|0x0008|...|0x10000|0x20000|0x40000|0x80000|...|
|---|---|---|---|---|---|---|---|---|---|---|
|Ezi-IO-I32|IN0|IN1|IN2|IN3|...|IN16|IN17|IN18|...|
|Ezi-IO-O32|OUT0|OUT1|OUT2|OUT3|...|OUT16|OUT17|OUT18|...|
|Ezi-IO-I16O16|IN0|IN1|IN2|IN3|...|OUT0|OUT1|OUT2|...||

## 2. Run trigger
``` c++
unsigned int uRunMask;
unsigned int uStopMask;

// Run Output
uRunMask = (0x01 << cOutputNo); // Run Output Pin #0
uStopMask = 0x00000000;
if (FAS_SetRunStop(nPortID, iSlaveNo, uRunMask, uStopMask) != FMM_OK)
{
	printf("Function(FAS_SetRunStop) was failed.\n");
	return false;
}
```
[EN]  
You can use the FAS_SetRunStop() function to Run/Stop the Trigger.
Trigger run requests are written sequentially from LSB to MSB of uRunMask.
For example, trigger run request 0 is written in the LSB, and trigger run request 31 is written in the MSB.
Trigger stop requests are written sequentially from LSB to MSB of uStopMask.
For example, trigger stop request 0 is written in the LSB, and trigger stop request 31 is written in the MSB.

[KR]  
FAS_SetRunStop() 함수를 사용하여 Trigger를 Run/Stop 시킬 수 있습니다.
uRunMask의 LSB에 0번 Trigger Run 요청부터 MSB에 31번 Trigger Run 요청이 순차적으로 쓰여집니다.
uStopMask의 LSB에 0번 Trigger Stop 요청부터 MSB에 31번 Trigger Stop 요청이 순차적으로 쓰여집니다.

## 3. Monitor trigger count
``` c++
unsigned char cOutputNo = 0;			// Output pin #0
unsigned int uCount = 0;

do
{
	usleep(100 * 1000);

	if (FAS_GetTriggerCount(nPortID, iSlaveNo, cOutputNo, &uCount) != FMM_OK)
	{
		printf("Function(FAS_GetTriggerCount) was failed.\n");
		return false;
	}
	else
	{
		printf("Get Trigger [%d] count \n", uCount);
	}
} while (uCount < Trg_Info.wCount);
```
[EN]  
You can check how many times trigger occurred for a specific Output Pin using the FAS_GetTriggerCount() function.

[KR]  
FAS_GetTriggerCount() 함수를 통하여 특정 Output Pin의 Trigger 발생 횟수를 읽어올 수 있습니다.

## 4. Etc
[EN]  
1. Please refer to the [01.ConnectionExam] project document for function descriptions on connecting and disconnecting devices.

[KR]  
1. 장치 연결 및 해제에 대한 함수 설명은 [01.ConnectionExam] 프로젝트 문서를 참고하시기 바랍니다.
