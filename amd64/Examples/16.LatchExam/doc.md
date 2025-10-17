# LatchExam

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
2. Read Input/Latch status for 60 seconds, Read Latch Count of Pin 0.
3. Clear Latch status of Pin 0.
4. Read Input/Latch status.
5. Read total Latch Count.
6. Clear Latch Count of Pin 0.
5. Close connection.

[KR]  
1. 장치 연결.
2. 60초간 Input / Latch 상태 읽기, 0번 Pin의 Latch Count 읽기.
3. 0번 Pin의 Latch 상태 초기화.
4. Input / Latch 상태 읽기.
5. 전체 Latch Count 읽기.
6. 0번 Pin의 Latch Count 초기화.
7. 연결 해제.

## 1. Read latch Count
``` c++
unsigned int uInput = 0;
unsigned int uLatch = 0;

unsigned char cInputNo = 0;	// Pin 0
unsigned int nLatchCount = 0;

// Monitor the specific pin input result value.
do
{
	// Latch status
	if (FAS_GetInput(nPortID, iSlaveNo, &uInput, &uLatch) != FMM_OK)
	{
		printf("Function(FAS_GetInput) was failed.\n");
		return false;
	}

	// Latch count
	if (FAS_GetLatchCount(nPortID, iSlaveNo, cInputNo, &nLatchCount) != FMM_OK)
	{
		printf("Function(FAS_GetLatchCount) was failed.\n");
		return false;
	}

	printf("Pin %d is %s and %s (latch count %d) \n", cInputNo, ((uInput & (0x01 << cInputNo)) ? "ON" : "OFF"), ((uLatch & (0x01 << cInputNo)) ? "latched" : "not latched"), nLatchCount);
} while (WaitSeconds(60));
```
[EN]  
You can read the Latch Count of a specific pin using the FAS_GetLatchCount() function. 
The above source code repeats 'Read Input / Latch Status' and 'Read Latch Count' for 60 seconds.

[KR]  
FAS_GetLatchCount() 함수를 사용하여 특정 핀의 Latch Count를 읽을 수 있습니다.
위 소스 코드는 60초간 'Input / Latch 상태 읽기'와 'Latch Count 읽기'를 반복합니다.

### 1.1 Read Input/Latch status
[EN]  
You can read information about the Input bit and Latch using the FAS_GetInput() function. 
For more information, please refer to the [15.GetInputExam] project document.

[KR]  
FAS_GetInput() 함수를 사용하여 Input bit와 Latch에 대한 정보를 읽을 수 있습니다.
자세한 내용은 [15.GetInputExam] 프로젝트 문서를 참고하시기 바랍니다.

## 2. Clear latch
``` c++
// Clear the specific pin's Latch status
uLatchMask = (0x01 << cInputNo);
if (FAS_ClearLatch(nPortID, iSlaveNo, uLatchMask) != FMM_OK)
{
	printf("Function(FAS_ClearLatch) was failed.\n");
	return false;
}
```
[EN]  
You can clear the latch state using the FAS_ClearLatch() function.

[KR]  
FAS_ClearLatch() 함수를 사용하여 Latch 상태를 초기화 할 수 있습니다.

### 2.1 Send data logic
[EN]  
The request for Latch state 0 is sequentially written to the LSB of uLatchMask (32 bits) and the request for Latch state 31 is sequentially written to the MSB. 
If the value of each bit is 0, the current state is maintained, and if it is 1, the Latch is cleared.

[KR]  
uLatchMask(32bit)의 LSB에 0번 Latch 상태에 대한 요청부터 MSB에 31번 Latch 상태에 대한 요청이 순차적으로 쓰여집니다.
각 bit의 값이 0인 경우 현재 상태 유지, 1인 경우 Latch를 Clear합니다.

## 3. Read latch count all (16 Input)
``` c++
unsigned int cntLatchAll[16] = { 0 };

// Get latch counts of all inputs (16 inputs)
if (FAS_GetLatchCountAll(nPortID, iSlaveNo, cntLatchAll) != FMM_OK)
{
	printf("Function(FAS_GetLatchCountAll) was failed.\n");
	return false;
}
else
{
	for (int i = 0; i < 16; i++)
	{
		printf("[FAS_GetLatchCountAll]  Pin[%d] : [%d]count \n", i, cntLatchAll[i]);
	}
}
```
[EN]  
Use the FAS_GetLatchCountAll() function to read the total Latch Count for 16 Inputs. 
For a request for 32 Inputs, use FAS_GetLatchCountAll32().

[KR]  
FAS_GetLatchCountAll() 함수를 사용하여 16 Input에 대해 전체 Latch Count를 읽습니다.
32 Input에 대한 요청은 FAS_GetLatchCountAll32()를 사용합니다.

## 4. Clear latch count
``` c++
unsigned int uLatchMask;
uLatchMask = (0x01 << cInputNo);

// Clear the latch count of the specific pin
if (FAS_ClearLatchCount(nPortID, iSlaveNo, uLatchMask) != FMM_OK)
{
	printf("Function(FAS_ClearLatchCount) was failed.\n");
	return false;
}
```
[EN]  
Initialize the Latch Count of a specific bit using the FAS_ClearLatchCount() function.

[KR]  
FAS_ClearLatchCount() 함수를 통하여 특정 bit의 Latch Count를 초기화 합니다.

## 5. Etc
[EN]  
1. Please refer to the [01.ConnectionExam] project document for function descriptions on connecting and disconnecting devices.

[KR]  
1. 장치 연결 및 해제에 대한 함수 설명은 [01.ConnectionExam] 프로젝트 문서를 참고하시기 바랍니다.
