#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include "interruptable.h"
#include "Arduino.h"

class RotaryEncoder: Interruptable{

public:

	RotaryEncoder(int, Interruptable::interruptPins , bool);
	long read();
	long last_read();

protected:

	void onInterrupt();

private:

	volatile long pulses;
	volatile long lastPulses;
	int pollingChannelPin;
	bool clockwiseFwd;

	void pulseInc();
	void pulseDec();
};

#endif
