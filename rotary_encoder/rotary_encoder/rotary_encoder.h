#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include "interrupt_handler.h"
#include "Arduino.h"

class RotaryEncoder: Interruptable{

public:

	//TODO: add pp macros to determine board and interrupt pins
	enum interruptPinOptions {INTERRUPT_A = 2, INTERRUPT_B = 3};

	RotaryEncoder(int, enum interruptPinOptions, bool);
	long read();
	long last_read();

protected:

	virtual void onInterrupt();

private:

	volatile long pulses;
	volatile long lastPulses;
	int pollingChannelPin;
	bool clockwiseFwd;

	void pulseInc();
	void pulseDec();
};

#endif
