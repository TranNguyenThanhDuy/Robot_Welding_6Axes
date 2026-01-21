# InputPinControlExam 

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
2. Configure input pin setting.
3. Monitor the input pin signals for a minute.
4. Close connection.

[KR]  
1. 장치 연결.
2. Input Pin 설정.
3. 일정 시간(1분) 동안 입력핀 신호 감시.
4. 연결 해제.

## 1. Input Pin setting
``` c++
// Set Input pin Value.
byPinNo = 3;
byLevel = LEVEL_LOW_ACTIVE;
dwInputMask = SERVO_IN_BITMASK_USERIN0;

if (FAS_SetIOAssignMap(nPortID, iSlaveNo, byPinNo, dwInputMask, byLevel) != FMM_OK)
{
	printf("Function(FAS_SetIOAssignMap) was failed.\n");
	return false;
}
```

### ※ Function detail
[EN]  
> FAS_SetIOAssignMap(unsigned char nPortNo, unsigned char iSlaveNo, unsigned char iIOPinNo, unsigned int dwIOLogicMask, unsigned char bLevel)
> - nPortNo: Port number for FAS_Connect function.
> - iSlaveNo: Slave number of the board.
> - iIOPinNo: I/O Pin number to read.
> - dwIOLogicMask: Logic Mask value to assign to the corresponding pin.
> - bLevel: Active Level value to set for the corresponding pin.

The source code above uses the 'FAS_SetIOAssignMap()' function to set the Logic Mask (SERVO_IN_Bitmask_PJOG) and operation signal Level value for pin 3.
Accordingly, if a signal is input to pin 3, the motor will perform a Positive JOG Move operation. 
As above, the Logic Mask signal uses the Bit Mask value defined for various motor operations [Origin / Limit- / Limit+ / PT Start, etc.], and can be checked in the model's header file ('MOTION_EziSERVO_DEFINE.h' for Ezi-SERVO Plus-R).
Therefore, the 'FAS_SetIOAssignMap()' function assigns the meaning of the corresponding signal by matching the Logic Mask and operation signal level value to the input/output pins, and the user can perform the desired motor operation through the input/output signals to the corresponding pins.
In addition, when the 'FAS_SetIOAssignMap()' function is used, the Logic Mask value of the corresponding pin is stored in RAM.

[KR]  
> FAS_SetIOAssignMap(unsigned char nPortNo, unsigned char iSlaveNo, unsigned char iIOPinNo, unsigned int dwIOLogicMask, unsigned char bLevel)
> - nPortNo : FAS_Connect 함수에서 연결한 Port 번호.
> - iSlaveNo : 해당 보드의 Slave 번호.
> - iIOPinNo : 읽어올 I/O Pin 번호.
> - dwIOLogicMask : 해당 핀에 할당할 Logic Mask 값.
> - bLevel : 해당 핀에 설정할 Active Level 값.

위 소스코드는 FAS_SetIOAssignMap() 함수를 사용하여 3번 핀에 Logic Mask(SERVO_IN_BITMASK_USERIN0) 및 동작 신호 Level 값을 설정합니다.
그에 따라 3번 핀으로 신호가 입력된다면 모터는 Positive JOG Move 동작을 실시하게 됩니다.
위와같이 Logic Mask 신호는 [Origin / Limit- / Limit+ / PT Start 등..]모터의 다양한 동작을 위해 정의된 Bit Mask 값이 사용되며, 모델의 header 파일(Ezi-SERVO Plus-R의 경우 MOTION_EziSERVO_DEFINE.h)에서 확인하실 수 있습니다.
따라서 FAS_SetIOAssignMap() 함수는 입출력핀에 Logic Mask 및 동작 신호 Level 값의 매칭을 통하여 해당 신호의 의미를 부여하게 되며, 의미가 부여된 핀에 입출력 신호를 통하여 사용자가 원하는 모터 동작을 진행할 수 있습니다.
더하여, FAS_SetIOAssignMap() 함수 사용하시 해당 핀에 매칭되는 Logic Mask값은 RAM에 저장되게 됩니다.

## 2. Check IO Map
``` c++
unsigned int dwLogicMask = 0;

// Show Input pins status
for (int i = 0; i < INPUTPIN; i++)
{
	if (FAS_GetIOAssignMap(nPortID, iSlaveNo, i, &dwLogicMask, &byLevel) != FMM_OK)
	{
		printf("Function(FAS_SetIOAssignMap) was failed.\n");
		return false;
	}

	if (dwLogicMask != IN_LOGIC_NONE)
		printf("Input PIN[%d] : Logic Mask 0x%08x (%s) \n", i, dwLogicMask, (byLevel == LEVEL_LOW_ACTIVE) ? "Low Active" : "High Active");
	else
		printf("Input Pin[%d] : Not Assigned \n", i);
}
```

### ※ Function detail
[EN]  
> FAS_GetIOAssignMap(unsigned char nPortNo, unsigned char iSlaveNo, unsigned char iIOPinNo, unsigned int* dwIOLogicMask, unsigned char* bLevel)
> - nPortNo: Port number for FAS_Connect function.
> - iSlaveNo: Slave number of the board.
> - iIOPinNo: I/O Pin number to read.
> - dwIOLogicMask: Pointer variable where the Logic Mask value assigned to the corresponding pin will be stored.
> - bLevel: Pointer variable where the Active Level value of the corresponding pin will be stored.

The source code above use the FAS_GetIOAssignMap() function to read the Logic Mask (Bit Mask) and Level values ​​for each pin set by the FAS_SetIOAssignMap() function.
The Logic Mask and Level values ​​of each pin are printed on the screen.

[KR]  
> FAS_GetIOAssignMap(unsigned char nPortNo, unsigned char iSlaveNo, unsigned char iIOPinNo, unsigned int* dwIOLogicMask, unsigned char* bLevel)
> - nPortNo : FAS_Connect 함수에서 연결한 Port 번호.
> - iSlaveNo : 해당 보드의 Slave 번호.
> - iIOPinNo : 읽어올 I/O Pin 번호.
> - dwIOLogicMask : 해당 Pin에 할당된 Logic Mask값이 저장될 포인터 변수.
> - bLevel : 해당 핀의 Active Level 값이 저장될 포인터 변수.

위 소스코드는 FAS_SetIOAssignMap() 함수를 통하여 설정한 각 핀에 대한 Logic Mask (Bit Mask) 및 Level 값을 FAS_GetIOAssignMap() 함수를 통하여 읽어들입니다.
읽어들인 각 핀의 Logic Mask 및 Level 값을 화면에 출력합니다.

## 3. Check Input Pin
``` c++
unsigned int dwInput;
unsigned int dwInputMask = SERVO_IN_BITMASK_USERIN0;

printf("----------------------------------------------------------------------------- \n");

// When the value SERVO_IN_BITMASK_USERIN0 is entered with the set PinNo, an input confirmation message is displyed.
// Monitoring input signal for 60 seconds
do
{
	if (FAS_GetIOInput(nPortNo, iSlaveNo, &dwInput) == FMM_OK)
	{
		if (dwInput & dwInputMask)
		{
			printf("INPUT PIN DETECTED.\n");
		}
	}
	else
		return false;
} while (WaitSeconds(60));
```
[EN]  
The above source code checks the status of all pins using the FAS_GetIOInput() function for 1 minute.
Also, it compares the Logic Mask (Bit Mask) value for Positive Jog and if the input of the corresponding bit is confirmed, the phrase "INPUT PIN DETECTED" is displayed on the screen.

[KR]  
위 소스코드는 1분동안 FAS_GetIOInput() 함수를 사용하여 핀 전체의 상태값을 확인합니다.
특히 Positive Jog에 대한 Logic Mask(Bit Mask)값을 비교하여 해당 비트의 입력이 확인된다면 "INPUT PIN DETECTED" 문구를 화면해 현시합니다.

### ※ Function detail
[EN]  
> FAS_GetIOInput(unsigned char nPortNo, unsigned char iSlaveNo, unsigned int* dwIOInput)
> - nPortNo: Port number for FAS_Connect function.
> - iSlaveNo: Slave number of the board.
> - dwIOInput: Pointer variable where the input value will be stored.

[KR]  
> FAS_GetIOInput(unsigned char nPortNo, unsigned char iSlaveNo, unsigned int* dwIOInput)
> - nPortNo : FAS_Connect 함수에서 연결한 Port 번호.
> - iSlaveNo : 해당 보드의 Slave 번호.
> - dwIOInput : Input 값이 저장될 포인터 변수.

## 4. Etc
[EN]  
1. Please refer to the ConnectionExam project documentation for functional descriptions of device connection and disconnection.

[KR]  
1. 장치 연결 및 해제에 대한 기능 설명은 ConnectionExam 프로젝트 문서를 참고하시기 바랍니다.