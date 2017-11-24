#include "rotary_encoder.h"
#include "Arduino.h"

RotaryEncoder::RotaryEncoder(int pollingChannelPin, RotaryEncoder::interruptPinOptions leadingChannelPin, bool clockwiseFwd)
: Interruptable(leadingChannelPin, Interruptable::CHANGE)
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

void RotaryEncoder::onRotaryTransition(){

	//TODO: put more thought into this logic, pretty sure it's more complicated than this
	if(digitalRead(instance->pollingChannelPin) == HIGH && instance->clockwiseFwd)
		instance->pulseInc();
	else
		instance->pulseDec();
}

void RotaryEncoder::pulseInc(){

	pulses++;
	lastPulses++;
}

void RotaryEncoder::pulseDec(){

	pulses--;
	lastPulses--;
}
