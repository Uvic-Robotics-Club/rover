#include "interrupt_handler.h"
#include "Arduino.h"

Interruptable::Interruptable(){

	//empty constructor for private declaration
}

bool InterruptHandler::addInterruptable(Interruptable* newInterruptable){

	//add to interruptables array
	//if first interruptable for this pin/event, attach an interrupt listener for this pin/event
	bool success = false;
	for(Interruptable* existingInterruptable: interruptables){

		//overrwrite the first unused (0) pointer
		if(existingInterruptable == 0){

			existingInterruptable = newInterruptable;
			success = true;
			break;
		}
	}

	if(success){

		int pinCountIndex = 0;
		switch(newInterruptable->interruptPin){

			//TODO: add pp macros to determine board and interrupt pins
			case Interruptable::INTERRUPT_A:
				pinCountIndex = INTERRUPT_A;
				break;

			case Interruptable::INTERRUPT_B:
				pinCountIndex = INTERRUPT_B;
				break;

			default:
				return false;
				break;
		}

		int pinCounts[] = 0;
		switch(newInterruptable->interruptMode){

			case Interruptable::INTERRUPT_LOW:
				pinCounts = lowPinCounts;
				break;

			case Interruptable::INTERRUPT_HIGH:
				pinCounts = highPinCounts;
				break;

			case Interruptable::INTERRUPT_FALLING:
				pinCounts = fallingPinCounts;
				break;

			case Interruptable::INTERRUPT_RISING:
				pinCounts = risingPinCounts;
				break;

			case Interruptable::INTERRUPT_CHANGE:
				pinCounts = changePinCounts;
				break;

			default:
				return false;
				break;
		}

		//attach interrupt if first
		if(pinCounts[pinCountIndex] == 0)
			attachInterrupt(digitalPinToInterrupt(newInterruptable->interruptPin),
				allInterrupts, newInterruptable->interruptMode);
		pinCounts[pinCountIndex]++;

		return true;
	}
	else
		return false;
}

bool InterruptHandler::removeInterruptable(Interruptable* removeInterruptable){

	//iterate through interruptables and erase this one
	//if this pin/event is empty after removing, detach the interrupt
	bool isFound = false;
	for(Interruptable* interruptable: interruptables){

		if(interruptable == removeInterruptable){

			isFound = true;
			break;
		}
	}

	if(isFound){

		int pinCountIndex = 0;
		switch(newInterruptable->interruptPin){

			//TODO: add pp macros to determine board and interrupt pins
			case Interruptable::INTERRUPT_A:
				pinCountIndex = INTERRUPT_A;
				break;

			case Interruptable::INTERRUPT_B:
				pinCountIndex = INTERRUPT_B;
				break;

			default:
				return false;
				break;
		}

		int pinCounts[] = 0;
		switch(newInterruptable->interruptMode){

			case Interruptable::INTERRUPT_LOW:
				pinCounts = lowPinCounts;
				break;

			case Interruptable::INTERRUPT_HIGH:
				pinCounts = highPinCounts;
				break;

			case Interruptable::INTERRUPT_FALLING:
				pinCounts = fallingPinCounts;
				break;

			case Interruptable::INTERRUPT_RISING:
				pinCounts = risingPinCounts;
				break;

			case Interruptable::INTERRUPT_CHANGE:
				pinCounts = changePinCounts;
				break;

			default:
				return false;
				break;
		}

		//negate this interruptable, reduce the pin count, and if empty remove listener
		pinCounts[pinCountIndex]--;
		if(pinCounts[pinCountIndex] == 0)
			detachInterrupt(digitalPinToInterrupt(removeInterruptable->interruptPin));
		removeInterruptable = 0;

		return true;
	}
	else
		return false;
}

void InterruptHandler::allInterrupts(){

	//call 'onInterrupt' for each interruptable element at their selected mode
	for(Interruptable* interruptable: interruptables){

		interruptable->onInterrupt();
	}
}
