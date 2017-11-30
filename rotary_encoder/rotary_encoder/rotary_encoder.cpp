#include "rotary_encoder.h"
#include "Arduino.h"

RotaryEncoder::RotaryEncoder(int pollingChannelPin, Interruptable::interruptPins leadingChannelPin, bool clockwiseFwd)
: Interruptable(leadingChannelPin, Interruptable::INTERRUPT_HIGH)
{

	this->pollingChannelPin = pollingChannelPin;
	this->clockwiseFwd = clockwiseFwd;

	pulses = 0;
	lastPulses = 0;

	pinMode(pollingChannelPin, INPUT);
}

long RotaryEncoder::read(){

	lastPulses = 0;
	return pulses;
}

long RotaryEncoder::last_read(){

	long displayValue = lastPulses;
	lastPulses = 0;
	return displayValue;
}

void RotaryEncoder::onInterrupt(){

	//TODO: put more thought into this logic, pretty sure it's more complicated than this
	if(digitalRead(pollingChannelPin) == HIGH && clockwiseFwd)
		pulseInc();
	else
		pulseDec();
}

void RotaryEncoder::pulseInc(){

	pulses++;
	lastPulses++;
}

void RotaryEncoder::pulseDec(){

	pulses--;
	lastPulses--;
}
