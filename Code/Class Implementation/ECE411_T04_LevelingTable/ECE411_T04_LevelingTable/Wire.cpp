/*******************************************************************************
* Author: Sean Hendrickson
* Date: 12/04/2015
* File: Wire.cpp
* Description:  This file implements the methods needed to alter pin 
*               configurations on an ATMEGA328 microcontroller
*******************************************************************************/
#include "Wire.h"


CWire::CWire()
{
}


CWire::~CWire()
{
}


//*******************************************************
// Desc: sets the passed pin as an OUTPUT for ATMEGA328
// Param: pinNumber - pin to configure
//        logicOut - 1 is output HIGH
//                   0 is output LOW
// Ret: -1 if pin out of bounds,
//       0 if successful
//*******************************************************
int CWire::pinOutput(uint8_t pinNumber, uint8_t logicOut)
{}


//*******************************************************
// Desc: sets the passed pin as an INPUT for ATMEGA328
// Param: pinNumber - pin to configure
// Ret: -1 if pin out of bounds,
//       0 if successful
//*******************************************************
int CWire::pinInput(uint8_t pinNumber)
{}


// Return PORT of passed pin. These pins are
// specific to the ATMEGA328 28pin DIP layout
uint8_t CWire::getPort(uint8_t pinNumber)
{
	// find and output port
	switch (pinNumber)
	{
	case 1: return Port::PORTC;
	case 2: return Port::PORTD;
	case 3: return Port::PORTD;
	case 4: return Port::PORTD;
	case 5: return Port::PORTD;
	case 6: return Port::PORTD;
	case 9: return Port::PORTB;
	case 10: return Port::PORTB;
	case 11: return Port::PORTD;
	case 12: return Port::PORTD;
	case 13: return Port::PORTD;
	case 14: return Port::PORTB;
	case 15: return Port::PORTB;
	case 16: return Port::PORTB;
	case 17: return Port::PORTB;
	case 18: return Port::PORTB;
	case 19: return Port::PORTB;
	case 23: return Port::PORTC;
	case 24: return Port::PORTC;
	case 25: return Port::PORTC;
	case 26: return Port::PORTC;
	case 27: return Port::PORTC;
	case 28: return Port::PORTC;
	default: return Port::ERROR;  // pin not found
	}
}


// sets pin as output by writing 1 to DDRB register
// and using PORTB to set logic level
void CWire::bPortOut(uint8_t pin, uint8_t outputLevel)
{
	// set pin as output and Logic Level
	switch (pin)
	{
	case 9: // (PIN9 = PB6)
	{	DDRB |= (1 << PORTB6);  // set as output
		(outputLevel == HIGH) ?      // if HIGH, set output HIGH, else set LOW
		PORTB |= (1 << DDB6) : PORTB &= ~(1 << DDB6);
		break; }
	case 10: // (PIN10 = PB7)
	{	DDRB |= (1 << PORTB7);  // set as output
		(level == HIGH) ?      // if HIGH, set output HIGH, else set LOW
		PORTB |= (1 << DDB7) : PORTB &= ~(1 << DDB7);
		break; }
	case 14: // (PIN14 = PB0)
	{	DDRB |= (1 << PORTB0);  // set as output
		(level == HIGH) ?      // if HIGH, set output HIGH, else set LOW
		PORTB |= (1 << DDB0) : PORTB &= ~(1 << DDB0);
		break; }
	case 15: // (PIN15 = PB1)
	{	
		DDRB |= (1 << PORTB1);  // set as output
		(level == HIGH) ?      // if HIGH, set output HIGH, else set LOW
		PORTB |= (1 << DDB1) : PORTB &= ~(1 << DDB1);
		break; }
	case 16: // (PIN16 = PB2)
	{	DDRB |= (1 << PORTB2);  // set as output
	(level == HIGH) ?      // if HIGH, set output HIGH, else set LOW
		PORTB |= (1 << DDB2) : PORTB &= ~(1 << DDB2);
	break; }
	case 17: // (PIN17 = PB3)
	{	DDRB |= (1 << PORTB3);  // set as output
	(level == HIGH) ?      // if HIGH, set output HIGH, else set LOW
		PORTB |= (1 << DDB3) : PORTB &= ~(1 << DDB3);
	break; }
	case 18: // (PIN18 = PB4)
	{	DDRB |= (1 << PORTB4);  // set as output
	(level == HIGH) ?      // if HIGH, set output HIGH, else set LOW
		PORTB |= (1 << DDB4) : PORTB &= ~(1 << DDB4);
	break; }
	case 19: // (PIN19 = PB5)
	{	DDRB |= (1 << PORTB5);  // set as output
	(level == HIGH) ?      // if HIGH, set output HIGH, else set LOW
		PORTB |= (1 << DDB5) : PORTB &= ~(1 << DDB5);
	break; }
	}
}


void CWire::cportOut(uint8_t pin, uint8_t outputLevel)
{}


void CWire::dportOut(uint8_t pin, uint8_t outputLevel)
{}


void CWire::bportIn(uint8_t pin)
{}


void CWire::cportIn(uint8_t pin)
{}


void CWire::dportIn(uint8_t pin)
{}




