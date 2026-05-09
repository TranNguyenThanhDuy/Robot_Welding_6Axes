#ifndef _COMM_SETTING_AVR128_H_
#define _COMM_SETTING_AVR128_H_
//===========================================================================
//	File Name	: Communication_Setting_AVR128.h
//	Description : Serial Communications for AVR128
//===========================================================================

//===========================================================================
//	Include Files
//===========================================================================

// Include definition files.

//===========================================================================
//	Serial Communication Functions
//===========================================================================

// switching tx/rx
#define _SERIAL_TX_ENABLE()         do{PORTE.6 = 1; PORTE.7 = 1;}while(0)
#define _SERIAL_RX_ENABLE()         do{PORTE.7 = 0; PORTE.6 = 0;}while(0)

// send/recv
#define _SERIAL_SEND_BYTE(c)        do{while((UCSR0A&0x20) == 0x00); UDR0 = c; delay_us(100);}while(0)
#define _SERIAL_RECV_BYTE()         (UDR0)

// check rx buffer
#define _SERIAL_IS_RXBUFF_EMPTY()   ((UCSR0A&0x80) == 0x00)

#endif	// _COMM_SETTING_AVR128_H_