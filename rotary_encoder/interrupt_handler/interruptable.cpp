#include "interruptable.h"
#include "Arduino.h"

InterruptHandler* Interruptable::interruptHandler = 0;

Interruptable::Interruptable(int interruptPin, Interruptable::interruptModes interruptMode){

	this->interruptPin = interruptPin;
	this->interruptMode = interruptMode;

	//if interrupthandler doesn't exist, create one
	//always add this interruptable to the handler
	if(!interruptHandler)
		interruptHandler = new InterruptHandler();

	interruptHandler->addInterruptable(this);
}

Interruptable::~Interruptable(){

	//remove this from the interrupt handler
	interruptHandler->removeInterruptable(this);
}

int Interruptable::getInterruptPin(){

	return interruptPin;
}

Interruptable::interruptModes Interruptable::getInterruptMode(){

	return interruptMode;
}
