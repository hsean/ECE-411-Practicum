/*******************************************************************************
 * Author: Sean Hendrickson
 * Date: 11/23/2015
 * File: LEDStrip.h
 * Description:  This file models an array of LEDs for our ECE 411 
 *               self leveling table
 *******************************************************************************/
#pragma once
#include "LED.h"
#define DEFAULT_NUM_LEDS 20
#define DEFAULT_COLOR 0
#define DEFAULT_BRIGHTNESS 0

class CLEDStrip
{
public:
	CLEDStrip(void);
	CLEDStrip(int numLEDs, int iniColor, int iniBrightness);
	~CLEDStrip(void);

	//*************************************************
	// Desc: Sets the address of the LED
	// Param: LEDAddress - position of LED in array
	// Return: 0 on success, -1 if error
	//*************************************************
	int setAddress(int arrayPos, int LEDAddress);

	//*************************************************
	// Desc: Sets the color of the LED
	// Param: LEDAddress - position of LED in array
	// Return: 0 on success, -1 if error
	//*************************************************
	int setColor(int LEDAddress, int color);

	//*************************************************
	// Desc: Sets the brightness of the LED
	// Param: LEDAddress - position of LED in array
	// Return: 0 on success, -1 if error
	//*************************************************
	int setBrightness(int LEDAddress, int brightness);

	//*************************************************
	// Desc: returns the color of an LED
	// Param: LEDAddress - position of LED in array
	// Return: value of color 
	//*************************************************
	int getColor(int LEDAddress);

	//*************************************************
	// Desc: returns the brightness of an LED
	// Param: LEDAddress - position of LED in array
	// Return: value of brightness
	//*************************************************
	int getBrightness(int LEDAddress);

private:
	//*************************************************
	// Read data from LED strip
	//*************************************************
	int read();

	//*************************************************
	// Write too LED strip
	//*************************************************
	int write();

	CLED mLEDArray[DEFAULT_NUM_LEDS];  // array of LED objects
};

