/*******************************************************************************
 * Author: Sean Hendrickson
 * Date: 11/23/2015
 * File: LED.h
 * Description:  This file models an LED for our ECE 411 self leveling table
 *******************************************************************************/
#pragma once

class CLED
{
public:
	CLED(void);
	CLED(int address, int color, int brightness);
	~CLED(void);

	//***************************************
	// Desc: Sets the address of the LED
	// Param: address - int
	// Return: 0 on success, -1 if error
	//***************************************
	int setAddress(int address);  

	//***************************************
	// Desc: Sets the color of the LED
	// Param: color - int
	// Return: 0 on success, -1 if error
	//***************************************
	int setColor(int color);

	//***************************************
	// Desc: Sets the brightness of the LED
	// Param: brightness - int
	// Return: 0 on success, -1 if error
	//***************************************
	int setBrightness(int brightness);

private:
	int mAddress;
	int mColor;
	int mBrightness;
};

