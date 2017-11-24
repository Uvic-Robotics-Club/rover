#ifndef INTERRUPT_HANDER_H
#define INTERRUPT_HANDER_H

#include "interruptable.h"
#include "Arduino.h"

class Interruptable;

class InterruptHandler{

friend class Interruptable;

private:

	//TODO: add pp macros to determine board and interrupt pins
	//enum used more for readability than functionality
	enum interruptPinCountIndexes {INTERRUPT_A = 0, INTERRUPT_B = 1};
	int lowPinCounts[2] = {0};
	int highPinCounts[2] = {0};
	int fallingPinCounts[2] = {0};
	int risingPinCounts[2] = {0};
	int changePinCounts[2] = {0};

	Interruputable* interruptables[100] = {0};

	Interruptable();
	bool addInterruptable(Interruptable* newInterruptable);
	bool removeInterruptable(Interruptable* removeInterruptable);
	void allInterrupts();
};

#endif
