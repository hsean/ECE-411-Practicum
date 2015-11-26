/*******************************************************************************
* Author: Sean Hendrickson
* Date: 11/24/2015
* File: Accelerometer.h
* Description:  This file models an accelerometer
*               for our ECE 411 self leveling table
*******************************************************************************/
#pragma once
#define ACCEL_ARRAY_SIZE 3

class CAccelerometer
{
public:
	CAccelerometer(void);
	~CAccelerometer(void);

	//*******************************************************
	// retrieve data from accelerometer over I2C and
	// store captured data
	// Post-Cond: Data returned through x, y, z parameters
	// Param: x - int to hold x-axis data in quarter ms
	//        y - int to hold y-axis data in quarter ms
	//        z - int to hold z-axis data in quarter ms
	// Return: Returns 0 on success, else returns -1 if error
	//*******************************************************
	int getData(int &x, int &y, int &z);

	//*******************************************************
	// Set data for x, y, and z-axis.  This function is 
	// intended for testing purposes
	// Param: x - x-axis data in quarter ms
	//        y - y-axis data in quarter ms
	//        z - z-axis data in quarter ms
	// Return: 0 on success, -1 if error
	//*******************************************************
	int setData(int x, int y, int z);
private:
	//*******************************************************
	// Read data from the accelerometer
	//*******************************************************
	int read();

	//*******************************************************
	// Write data to the accelerometer
	//*******************************************************
	int write();

	int mX;  // x-axis data
	int mY;  // y-axis data
	int mZ;  // z-axis data
};

