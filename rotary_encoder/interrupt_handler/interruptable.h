#ifndef INTERRUPTABLE_H
#define INTERRUPTABLE_H

#include "interrupt_handler.h"
#include "Arduino.h"

class Interruptable{

friend class InterruptHandler;

public:

	enum interruptModes {INTERRUPT_LOW = LOW, INTERRUPT_HIGH = HIGH,
		INTERRUPT_RISING = RISING, INTERRUPT_FALLING = FALLING};
	enum interruptPins {INTERRUPT_A = 2, INTERRUPT_B = 3};

	Interruptable(interruptPins, interruptModes);
	~Interruptable();
	int getInterruptPin();
	int getInterruptMode();

protected:

	virtual void onInterrupt();

private:

	int interruptPin;
	int interruptMode;
};

#endif
