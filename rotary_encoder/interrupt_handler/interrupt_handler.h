#ifndef INTERRUPT_HANDER_H
#define INTERRUPT_HANDER_H

#include "interruptable.h"
#include "Arduino.h"

class Interruptable;

class InterruptHandler{

public:

	static InterruptHandler* getInstance();
	bool addInterruptable(Interruptable*);
	bool removeInterruptable(Interruptable*);

private:

	//TODO: add pp macros to determine board and interrupt pins
	//enum used more for readability than functionality
	static InterruptHandler* instance;
	enum interruptPinCountIndexes {INTERRUPT_A = 0, INTERRUPT_B = 1};
	int lowPinCounts[2] = {0};
	int highPinCounts[2] = {0};
	int fallingPinCounts[2] = {0};
	int risingPinCounts[2] = {0};
	Interruptable* interruptables[100] = {0};

	InterruptHandler();
	static void allInterrupts();
};

#endif
