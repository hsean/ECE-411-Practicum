/*******************************************************************************
* Author: Sean Hendrickson
* Date: 11/23/2015
* File: ServoController.cpp
* Description:  This file implements a pololu mini maestro servo controller
*               for our ECE 411 self leveling table
*******************************************************************************/
#include "ServoController.h"


CServoController::CServoController(void)
{
	mServo1Pos = DEFAULT_SERVO_ANGLE;
	mServo2Pos = DEFAULT_SERVO_ANGLE;
	mServo3Pos = DEFAULT_SERVO_ANGLE;
	mAcceleration = DEFAULT_ACCELERATION;
	mSpeed = DEFAULT_SPEED;

	// TODO: Write targets to servo controller

}


CServoController::CServoController(int iniTarget1, int iniTarget2, 
	int iniTarget3, int iniAcceleration, int iniSpeed)
{
	mServo1Pos = iniTarget1;
	mServo2Pos = iniTarget2;
	mServo3Pos = iniTarget3;
	mAcceleration = iniAcceleration;
	mSpeed = iniSpeed;

	// TODO: Write targets to servo controller

}


CServoController::~CServoController(void)
{
	mServo1Pos = 0;
	mServo2Pos = 0;
	mServo3Pos = 0;
	mAcceleration = 0;
	mSpeed = 0;

	// TODO: Write targets to servo controller
}


//**************************************************
// This function sets a target for a single servo
// Param: channel - physical servo hookup (0 to 2)
//        target - position to move servo too (0 to 180 degrees)
// Returns: 0 on success, -1 if errors
//**************************************************
int CServoController::setTarget(int channel, int target)
{
	switch (channel)
	{
	case 0:
	{
		mServo1Pos = target;
		break;
	}
	case 1:
	{
		mServo2Pos = target;
	}
	case 2:
	{
		mServo3Pos = target;
	}
	default:
	{
		// no valid case entered, return error
		return -1;
	}
	}
	// TODO: write targets to servo controller

	return 0;
}


//**************************************************
// This function sets targets for all servos
// Param: target1 - servo 1 position
//        target2 - servo 2 position
//        target3 - servo 3 position
// Pre-Cond: assumes servos are in channels 0, 1, and 2.
//           targets correspond to equivalent channel
// Returns: 0 on success, -1 if errors
//**************************************************
int CServoController::setAllTargets(int target1, int target2, int target3)
{
	mServo1Pos = target1;
	mServo2Pos = target2;
	mServo3Pos = target3;

	// TODO: write targets to servo controller

	return 0;
}


//**************************************************
// This function sets the acceleration of all servos
// Param: accel - int (0 to 255)
// Returns: 0 on success, -1 if errors
//**************************************************
int CServoController::setAcceleration(int accel)
{
	// check for out of bounds
	if ((accel < MIN_ACCELERATION) || (accel > MAX_ACCELERATION))
	{
		return -1;
	}

	mAcceleration = accel;

	// TODO: write targets to servo controller
	return 0;
}


//**************************************************
// This function sets the speed of all servos
// Param: speed - int (0 to 255)
// Returns: 0 on success, -1 if errors
//**************************************************
int CServoController::setSpeed(int speed)
{
	// check for out of bounds error
	if ((speed < MIN_SPEED) || (speed > MAX_SPEED))
	{
		return -1;
	}

	mSpeed = speed;

	// TODO: write targets to servo controller
	return 0;
}


//**************************************************
// This function gets target of servo1
// Returns: target of servo1
//**************************************************
int CServoController::getServo1()
{
	return mServo1Pos;
}


//**************************************************
// This function gets target of servo2
// Returns: target of servo2
//**************************************************
int CServoController::getServo2()
{
	return mServo2Pos;
}


//**************************************************
// This function gets target of servo3
// Returns: target of servo3
//**************************************************
int CServoController::getServo3()
{
	return mServo3Pos;
}


//**************************************************
// This function gets the acceleration
// Returns: acceleration
//**************************************************
int CServoController::getAcceleration()
{
	return mAcceleration;
}


//**************************************************
// This function gets the speed of all servos
// Returns: speed
//**************************************************
int CServoController::getSpeed()
{
	return mSpeed;
}


