#ifndef _COMM_SETTING_H_
#define _COMM_SETTING_H_
//===========================================================================
//	File Name	: Communication_Setting.h
//	Description : Serial Communications for NONE
//===========================================================================

//===========================================================================
//	Serial Communication Functions
//===========================================================================

#define _SERIAL_TX_ENABLE()			do{ }while(0)
#define _SERIAL_RX_ENABLE()			do{ }while(0)

// send/recv
#define _SERIAL_SEND_BYTE(c)		do{ }while(0)
#define _SERIAL_RECV_BYTE()			(0)

// check rx buffer
#define _SERIAL_IS_RXBUFF_EMPTY()	(0)

#endif	// _COMM_SETTING_H_