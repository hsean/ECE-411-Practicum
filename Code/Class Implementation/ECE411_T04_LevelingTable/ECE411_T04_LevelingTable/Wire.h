/*******************************************************************************
* Author: Sean Hendrickson
* Date: 12/04/2015
* File: Wire.h
* Description:  This file models the methods needed to alter pin
*               configurations on an ATMEGA328 microcontroller
*******************************************************************************/
#pragma once
#include <cstdint>
#define HIGH 1
#define LOW 0

//************************************************
// Class CWire: simplifies pin configuration
//************************************************
class CWire
{
public:
	CWire();
	~CWire();
	
	//*******************************************************
	// Desc: sets the passed pin as an OUTPUT for ATMEGA328
	// Param: pinNumber - pin to configure
	//        logicOut - 1 is output HIGH
	//                   0 is output LOW
	// Ret: -1 if pin out of bounds,
	//       0 if successful
	//*******************************************************
	int pinOutput(uint8_t pinNumber, uint8_t logicOut);

	//*******************************************************
	// Desc: sets the passed pin as an INPUT for ATMEGA328
	// Param: pinNumber - pin to configure
	// Ret: -1 if pin out of bounds,
	//       0 if successful
	//*******************************************************
	int pinInput(uint8_t pinNumber);

private:
	enum Port{PORTB, PORTC, PORTD, ERROR};

	// Return PORT of passed pin. These pins are
	// specific to the ATMEGA328 28pin DIP layout
	uint8_t getPort(uint8_t pinNumber);

	// The following methods configure pins based on
	// the pin's PORT
	void bPortOut(uint8_t pin, uint8_t outputLevel);
	void cPortOut(uint8_t pin, uint8_t outputLevel);
	void dPortOut(uint8_t pin, uint8_t outputLevel);
	void bPortIn(uint8_t pin);
	void cPortIn(uint8_t pin);
	void dPortIn(uint8_t pin);
};

