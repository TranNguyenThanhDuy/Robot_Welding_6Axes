# GetInputExam

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
2. Read Input/Latch status.
3. Show Input bit.
4. Close connection.

[KR]  
1. 장치 연결.
2. Input/Latch 상태 읽기.
3. Input bit 출력.
4. 연결 해제.

## 1. Read Input/Latch status
``` c++
unsigned int uInput = 0;
unsigned int uLatch = 0;
bool bON;

if (FAS_GetInput(nPortID, iSlaveNo, &uInput, &uLatch) != FMM_OK)
{
	printf("Function(FAS_GetInput) was failed.\n");
	return false;
}
```
[EN]  
You can read information about the Input bit and Latch using the FAS_GetInput() function.

[KR]  
FAS_GetInput() 함수를 사용하여 Input bit와 Latch에 대한 정보를 읽을 수 있습니다.

## 2. Print input bit
``` c++
for (int i = 0; i < 16; i++)
{
	bON = ((uInput & (0x01 << i)) != 0);
	printf("Input bit %d is %s.\n", i, ((bON) ? "ON" : "OFF"));
}
```
[EN]  
Show input bits 0 to 15 using the data stored in uInput.

[KR]  
uInput에 저장된 데이터를 통하여 0~15번의 Input bit를 출력합니다.

### 2.1 Input/Latch logic
[EN]  
From the LSB to the MSB of uInput(32bit), Input bit 0~31 will be written sequentially.
From the LSB to the MSB of uLatch(32bit), Latch 0~31 will be written sequentially.

[KR]  
uInput(32bit)의 LSB에 0번 Input bit부터 MSB에 31번 Input bit가 순차적으로 쓰여집니다.
uLatch(32bit)의 LSB에 0번 Latch부터 MSB에 31번 Latch가 순차적으로 쓰여집니다.

## 3. Etc
[EN]  
1. Please refer to the [01.ConnectionExam] project document for function descriptions on connecting and disconnecting devices.

[KR]  
1. 장치 연결 및 해제에 대한 함수 설명은 [01.ConnectionExam] 프로젝트 문서를 참고하시기 바랍니다.
