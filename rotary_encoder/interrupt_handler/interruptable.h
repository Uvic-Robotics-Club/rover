#ifndef INTERRUPTABLE_H
#define INTERRUPTABLE_H

#include "interrupt_handler.h"
#include "Arduino.h"

class InterruptHandler;

class Interruptable{

friend class InterruptHandler;

public:

	enum interruptModes {INTERRUPT_LOW = LOW, INTERRUPT_HIGH = HIGH,
		INTERRUPT_RISING = RISING, INTERRUPT_FALLING = FALLING, INTERRUPT_CHANGE = CHANGE};

	Interruptable(int, interruptModes);
	~Interruptable();
	int getInterruptPin();
	interruptModes getInterruptMode();

protected:

	virtual void onInterrupt();

private:

	static InterruptHandler* interruptHandler;
	int interruptPin;
	interruptModes interruptMode;
};

#endif
