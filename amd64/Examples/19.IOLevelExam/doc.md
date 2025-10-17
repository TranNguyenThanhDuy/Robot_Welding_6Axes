# IOLevelExam

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
2. Check IO Level information.
3. Configure IO Level.
4. Close connection.

[KR]  
1. 장치 연결.
2. IO Level 정보 읽기.
3. IO Level 설정.
4. 연결 해제.

## 1. Get IO level
``` c++
bool GetIOLevel(int nPortID, unsigned char iSlaveNo)
{
	unsigned int uIOLevel = 0;
	bool bLevel;

	printf("---------------------------------- \n");
	// [Before] Check IO Level Status
	if (FAS_GetIOLevel(nPortID, iSlaveNo, &uIOLevel) != FMM_OK)
	{
		printf("Function(FAS_GetIOLevel) was failed.\n");
		return false;
	}
	printf("Load IO Level Status : 0x%08x \n", uIOLevel);

	for (int i = 0; i < 16; i++)
	{
		bLevel = ((uIOLevel & (0x01 << i)) != 0);
		printf("I/O pin %d : %s\n", i, (bLevel) ? "High Active" : "Low Active");
	}

	return true;
}
```
[EN]  
With FAS_GetIOLevel() function, you can read the level information for each IO Pin.
IO level information is written sequentially from LSB to MSB of uIOLevel.
For example, IO level information of pin 0 is written in the LSB, and IO level information of pin 31 is written in the MSB.
0: Low Active Level / 1: High Active Level

[KR]  
FAS_GetIOLevel() 함수를 사용하여 각 IO Pin에 대해 설정된 Level을 읽을 수 있습니다.
uIOLevel의 LSB에 0번 Pin의 Level부터 MSB에 31번 31번 Pin의 Level이 순차적으로 쓰여집니다.
0: Low Active Level / 1: High Active Level을 의미합니다.

### 1.1 Bitmask logic of Fastech IO product 

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

## 2. Set IO level
``` c++
bool SetIOLevel(int nPortID, unsigned char iSlaveNo)
{
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	~~ Fastech's product which has Input (or Output) only (Ezi-IO-I16, Ezi-IO-O16, Ezi-IO-I32, Ezi-IO-O32...), BitMask of Input (or Output) starts from 0x01,	~~
	~~ However, for Input+Output mixed products (Ezi-IO-I8O8, Ezi-IO-I16O16, etc.), the BitMask of Output is allocated after the BitMasks of Input.				~~
	~~ Ezi-IO-I8O8, for example, the BitMask of Input 0 is 0x0001, and the BitMask of Output 0 is 0x0100.														~~
	~~ For detailed allocation methods, please refer to Section 1.3 of the "doc.md" file of this example.														~~
	~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	~~ Fastech의 Input/Output 단일 제품 (Ezi-IO-I16, Ezi-IO-O16, Ezi-IO-I32, Ezi-IO-O32 등)은 Input과 Output의 BitMask가 각각 0x01부터 시작하지만,					~~
	~~ Input+Output 혼합 제품 (Ezi-IO-I8O8, Ezi-IO-I16O16 등)은 Input의 BitMask를 전부 할당한 뒤, Output의 BitMask가 이어서 할당되어있습니다.						~~
	~~ 예를들어 Ezi-IO-I8O8의 Input 0번의 BitMask는 0x0001, Output 0번의 BitMask는 0x0100입니다.																	~~
	~~ 자세한 할당 방식은 본 예제의 "doc.md"파일 1.3절을 참고 부탁드립니다.																							~~
	~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	unsigned int uIOLevel = 0x0000ff00;

	printf("---------------------------------- \n");
	// Set IO Level Status
	if (FAS_SetIOLevel(nPortID, iSlaveNo, uIOLevel) != FMM_OK)
	{
		printf("Function(FAS_SetIOLevel) was failed.\n");
	}
	else
	{
		printf("Set IO Level Status : 0x%08x \n", uIOLevel);
	}

	return true;
}
```
[EN]  
You can set the level for each IO Pin using the FAS_SetIOLevel() function.
IO level information is written sequentially from LSB to MSB of uIOLevel.
For example, IO level information of pin 0 is written in the LSB, and IO level information of pin 31 is written in the MSB.
0: Low Active Level / 1: High Active Level.
The source code above sets Pins 0 to 7 to Low Active Level and Pins 8 to 15 to High Active Level. (16 Pins)

[KR]  
FAS_SetIOLevel() 함수를 사용하여 각 IO Pin에 대해 Level을 설정할 수 있습니다.
uIOLevel의 LSB에 0번 Pin의 Level부터 MSB에 31번 31번 Pin의 Level을 순차적으로 입력합니다.
0: Low Active Level / 1: High Active Level을 의미합니다.
위 소스 코드는 0~7번 Pin을 Low Active Level로, 8~15번 Pin을 High Active Level로 설정합니다. (16 Pin)

## 3. Etc
[EN]  
1. Please refer to the [01.ConnectionExam] project document for function descriptions on connecting and disconnecting devices.

[KR]  
1. 장치 연결 및 해제에 대한 함수 설명은 [01.ConnectionExam] 프로젝트 문서를 참고하시기 바랍니다.
