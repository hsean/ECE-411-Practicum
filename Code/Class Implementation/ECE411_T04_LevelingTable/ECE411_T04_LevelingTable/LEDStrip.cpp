/*******************************************************************************
 * Author: Sean Hendrickson
 * Date: 11/23/2015
 * File: LEDStrip.cpp
 * Description:  This file models an array of LEDs for our ECE 411 
 *               self leveling table
 *******************************************************************************/
#include "LEDStrip.h"


CLEDStrip::CLEDStrip(void)
{
	mLEDArray[DEFAULT_NUM_LEDS];

	// fill array with default values
	for (int i = 0; i < DEFAULT_NUM_LEDS; ++i)
	{
		mLEDArray[i].setAddress(i);
		mLEDArray[i].setColor(DEFAULT_COLOR);
		mLEDArray[i].setBrightness(DEFAULT_BRIGHTNESS);
	}
}


CLEDStrip::CLEDStrip(int numLEDs, int iniColor, int iniBrightness)
{
	mLEDArray[numLEDs];

	// fill array with default values
	for (int i = 0; i < numLEDs; ++i)
	{
		mLEDArray[i].setAddress(i);
		mLEDArray[i].setColor(iniColor);
		mLEDArray[i].setBrightness(iniBrightness);
	}
}


CLEDStrip::~CLEDStrip()
{}


//***************************************
// Desc: Sets the address of the LED
// Param: LEDAddress - position of LED in array
// Return: 0 on success, -1 if error
//***************************************
int CLEDStrip::setAddress(int arrayPos, int LEDAddress)
{
	mLEDArray[arrayPos].setAddress(LEDAddress);
	return 0;
}


//***************************************
// Desc: Sets the address of the LED
// Param: led - pointer to LED object
// Return: 0 on success, -1 if error
//***************************************
int CLEDStrip::setColor(int LEDAddress, int color)
{
	mLEDArray[LEDAddress].setColor(color);
	return 0;
}


//***************************************
// Desc: Sets the address of the LED
// Param: address - int
// Return: 0 on success, -1 if error
//***************************************
int CLEDStrip::setBrightness(int LEDAddress, int brightness)
{
	mLEDArray[LEDAddress].setBrightness(brightness);
	return 0;
}


//*************************************************
// Desc: returns the color of an LED
// Param: LEDAddress - position of LED in array
// Return: value of color 
//*************************************************
int CLEDStrip::getColor(int LEDAddress)
{
	return mLEDArray[LEDAddress].getColor();
}


//*************************************************
// Desc: returns the brightness of an LED
// Param: LEDAddress - position of LED in array
// Return: value of brightness
//*************************************************
int CLEDStrip::getBrightness(int LEDAddress)
{
	return mLEDArray[LEDAddress].getBrightness();
}

