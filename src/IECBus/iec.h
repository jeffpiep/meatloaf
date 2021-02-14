#ifndef IEC_H
#define IEC_H

#include <Arduino.h>
//#include "global_defines.h"
#include "global_defines.h"
#include "cbmdefines.h"
#include "Petscii.h"

class IEC
{
public:
	enum IECState
	{
		noFlags = 0,
		eoiFlag = (1 << 0),	 // might be set by iec_receive
		atnFlag = (1 << 1),	 // might be set by iec_receive
		errorFlag = (1 << 2) // If this flag is set, something went wrong and
	};

	// Return values for checkATN:
	enum ATNCheck
	{
		ATN_IDLE = 0,		// Nothing recieved of our concern
		ATN_CMD = 1,		// A command is recieved
		ATN_CMD_LISTEN = 2, // A command is recieved and data is coming to us
		ATN_CMD_TALK = 3,	// A command is recieved and we must talk now
		ATN_ERROR = 4,		// A problem occoured, reset communication
		ATN_RESET = 5		// The IEC bus is in a reset state (RESET line).
	};

	// IEC ATN commands:
	enum ATNCommand
	{
		ATN_CODE_GLOBAL = 0x00,	  // 0x00 + cmd (global command)
		ATN_CODE_LISTEN = 0x20,	  // 0x20 + device_id (LISTEN)
		ATN_CODE_UNLISTEN = 0x3F, // 0x3F (UNLISTEN)
		ATN_CODE_TALK = 0x40,	  // 0x40 + device_id (TALK)
		ATN_CODE_UNTALK = 0x5F,	  // 0x5F (UNTALK)
		ATN_CODE_DATA = 0x60,	  // 0x60 + channel (SECOND)
		ATN_CODE_CLOSE = 0xE0,	  // 0xE0 + channel (CLOSE)
		ATN_CODE_OPEN = 0xF0	  // 0xF0 + channel (OPEN)
	};

	// ATN command struct maximum command length:
	enum
	{
		ATN_CMD_MAX_LENGTH = 40
	};

	typedef struct _tagATNCMD
	{
		byte code;
		byte command;
		byte channel;
		byte device;
		byte str[ATN_CMD_MAX_LENGTH];
		byte strLen;
	} ATNCmd;

	IEC();
	~IEC() {}

	// Initialise iec driver
	boolean init();

	// Checks if CBM is sending an attention message. If this is the case,
	// the message is recieved and stored in atn_cmd.
	ATNCheck checkATN(ATNCmd &atn_cmd);

	// Checks if CBM is sending a reset (setting the RESET line high). This is typicall
	// when the CBM is reset itself. In this case, we are supposed to reset all states to initial.
	boolean checkRESET();

	// Sends a byte. The communication must be in the correct state: a load command
	// must just have been recieved. If something is not OK, FALSE is returned.
	boolean send(byte data);

	// Same as IEC_send, but indicating that this is the last byte.
	boolean sendEOI(byte data);

	// A special send command that informs file not found condition
	boolean sendFNF();

	// Recieves a byte
	byte receive();

	// Enabled Device Bit Mask
	uint32_t enabledDevices;
	bool isDeviceEnabled(const byte deviceNumber);
	void enableDevice(const byte deviceNumber);
	void disableDevice(const byte deviceNumber);

	IECState state() const;

	boolean readATN()
	{
		return readPIN(IEC_PIN_ATN);
	}

	boolean readCLOCK()
	{
		return readPIN(IEC_PIN_CLOCK);
	}

	boolean readDATA()
	{
		return readPIN(IEC_PIN_DATA);
	}

	boolean readSRQ()
	{
		return readPIN(IEC_PIN_SRQ);
	}

	boolean readRESET()
	{
		return readPIN(IEC_PIN_RESET);
	}

private:
	// IEC Bus Commands
	ATNCheck deviceListen(ATNCmd &atn_cmd);	  // 0x20 + device_id 	Listen, device (0–30)
	ATNCheck deviceUnListen(ATNCmd &atn_cmd); // 0x3F 				Unlisten, all devices
	ATNCheck deviceTalk(ATNCmd &atn_cmd);	  // 0x40 + device_id 	Talk, device
	ATNCheck deviceUnTalk(ATNCmd &atn_cmd);	  // 0x5F 				Untalk, all devices
	ATNCheck deviceReopen(ATNCmd &atn_cmd);	  // 0x60 + channel		Reopen, channel (0–15)
	ATNCheck deviceClose(ATNCmd &atn_cmd);	  // 0xE0 + channel		Close, channel
	ATNCheck deviceOpen(ATNCmd &atn_cmd);	  // 0xF0 + channel		Open, channel

	byte timeoutWait(byte waitBit, boolean whileHigh);
	byte receiveByte(void);
	boolean sendByte(byte data, boolean signalEOI);
	boolean turnAround(void);
	boolean undoTurnAround(void);

	// false = LOW, true == HIGH
	boolean readPIN(byte pinNumber)
	{
		// To be able to read line we must be set to input, not driving.
		pinMode(pinNumber, INPUT);
		return digitalRead(pinNumber) ? true : false;
	}

	// true == PULL == HIGH, false == RELEASE == LOW
	void writePIN(byte pinNumber, boolean state)
	{
		pinMode(pinNumber, state ? OUTPUT : INPUT);
		digitalWrite(pinNumber, state ? LOW : HIGH);
	}

	void writeATN(boolean state)
	{
		writePIN(IEC_PIN_ATN, state);
	}

	void writeCLOCK(boolean state)
	{
		writePIN(IEC_PIN_CLOCK, state);
	}

	void writeDATA(boolean state)
	{
		writePIN(IEC_PIN_DATA, state);
	}

	void writeSRQ(boolean state)
	{
		writePIN(IEC_PIN_SRQ, state);
	}

	// communication must be reset
	byte m_state;
};

#endif
