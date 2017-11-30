#include "interruptable.h"
#include "Arduino.h"

Interruptable::Interruptable(interruptPins interruptPin, interruptModes interruptMode){

	this->interruptPin = interruptPin;
	this->interruptMode = interruptMode;

	//add to the interrupt handler
	InterruptHandler* interruptHandler = InterruptHandler::getInstance();
	interruptHandler->addInterruptable(this);
}

Interruptable::~Interruptable(){

	//remove this from the interrupt handler
	InterruptHandler* interruptHandler = InterruptHandler::getInstance();
	interruptHandler->removeInterruptable(this);
}

int Interruptable::getInterruptPin(){

	return interruptPin;
}

int Interruptable::getInterruptMode(){

	return interruptMode;
}

void Interruptable::onInterrupt(){


}
