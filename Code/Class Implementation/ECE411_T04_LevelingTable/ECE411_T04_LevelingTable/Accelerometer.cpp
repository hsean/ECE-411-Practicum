/*******************************************************************************
* Author: Sean Hendrickson
* Date: 11/24/2015
* File: Accelerometer.cpp
* Description:  This file implements an accelerometer
*               for our ECE 411 self leveling table
*******************************************************************************/
#include "Accelerometer.h"


CAccelerometer::CAccelerometer(void)
{
	// initialize data
	mX = 0;
	mY = 0;
	mZ = 0;

	// TODO: Initialize accelerometer
	// TODO: get data from accelerometer
}


CAccelerometer::~CAccelerometer(void)
{
	// clean up data
	mX = 0;
	mY = 0;
	mZ = 0;
}

//*******************************************************
// retrieve data from accelerometer over I2C and
// store captured data
// Post-Cond: Data returned through x, y, z parameters
// Param: x - int to hold x-axis data in quarter ms
//        y - int to hold y-axis data in quarter ms
//        z - int to hold z-axis data in quarter ms
// Return: Returns 0 on success, else returns -1 if error
//*******************************************************
int CAccelerometer::getData(int &x, int &y, int &z)
{
	// TODO: get data from accelerometer and place in data array

	// return data
	x = mX;
	y = mY;
	z = mZ;

	return 0;
}


//*******************************************************
// Set data for x, y, and z-axis.  This function is 
// intended for testing purposes
// Param: x - x-axis data in quarter ms
//        y - y-axis data in quarter ms
//        z - z-axis data in quarter ms
// Return: 0 on success, -1 if error
//*******************************************************
int CAccelerometer::setData(int x, int y, int z)
{
	// set data 
	mX = x;
	mY = y;
	mZ = z;

	return 0;
}

