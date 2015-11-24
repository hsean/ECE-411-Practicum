/*******************************************************************************
* Author: Sean Hendrickson
* Date: 11/23/2015
* File: SystemManager.h
* Description:  This file models a pololu mini maestro servo controller
*               for our ECE 411 self leveling table
*******************************************************************************/
#pragma once
#include "Accelerometer.h"
#include "ServoController.h"
#include "LEDStrip.h"

#define NUM_STORED_DATA 5

//******************************************
// Struct AccelData - stores data from 
// accelerometer as angles.
//******************************************
struct AccelData
{
	int x;
	int y;
	int z;
};


//******************************************
// Struct ServoTargets - stores data for
// calculated targets for servos based on 
// accelerometer data.
//******************************************
struct ServoTargets
{
	int servo1Target;
	int servo2Target;
	int servo3Target;
};


class CSystemManager
{
public:
	CSystemManager(void);
	~CSystemManager(void);

private:
	CAccelerometer* mAccelerometer;
	CServoController* mServoController;
	CLEDStrip* mLEDStrip;
	CAccelerometer* mAccelerometer;
	AccelData* currentAccelData[NUM_STORED_DATA];
	AccelData* previousAccelData[NUM_STORED_DATA];
	AccelData* newAccelAvg;
	AccelData* prevAccelAvg;

};

