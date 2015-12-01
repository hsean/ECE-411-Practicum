/*******************************************************************************
 * Author: Sean Hendrickson
 * Date: 11/23/2015
 * File: LED.cpp
 * Description:  This file implements an LED for our ECE 411 self leveling table
 *******************************************************************************/

#include "LED.h"


CLED::CLED(void)
{
	mAddress = 0;
	mColor = 0;
	mBrightness = 0;
}

CLED::CLED(int address, int color, int brightness)
{
	mAddress = address;
	mColor = color;
	mBrightness = brightness;
}

CLED::~CLED(void)
{
	mAddress = 0;
	mColor = 0;
	mBrightness = 0;
}

//***************************************
// Desc: Sets the address of the LED
// Param: address - int
// Return: 0 on success, -1 if error
//***************************************
int CLED::setAddress(int address)
{
	mAddress = address;
	return 0;
}

//***************************************
// Desc: Sets the color of the LED
// Param: color - int
// Return: 0 on success, -1 if error
//***************************************
int CLED::setColor(int color)
{
	mColor = color;
	return 0;
}

//***************************************
// Desc: Sets the brightness of the LED
// Param: brightness - int
// Return: 0 on success, -1 if error
//***************************************
int CLED::setBrightness(int brightness)
{
	mBrightness = brightness;
	return 0;
}

//***************************************
// Desc: Gets the address of the LED
// Return: LED address
//***************************************
int CLED::getAddress()
{
	return mAddress;
}


//***************************************
// Desc: Gets the color of the LED
// Return: LED color
//***************************************
int CLED::getColor()
{
	return mColor;
}


//***************************************
// Desc: Gets the brightness of the LED
// Return: LED brightness
//***************************************
int CLED::getBrightness()
{
	return mBrightness;
}


