/*******************************************************************************
* Author: Sean Hendrickson
* Date: 11/23/2015
* File: ServoController.h
* Description:  This file models a pololu mini maestro servo controller
*               for our ECE 411 self leveling table
*******************************************************************************/
#pragma once
#define DEFAULT_SERVO_ANGLE 45
#define DEFAULT_ACCELERATION 0
#define DEFAULT_SPEED 0
#define MIN_ACCELERATION 0
#define MAX_ACCELERATION 255
#define MIN_SPEED 0
#define MAX_SPEED 255


class CServoController
{
public:
	CServoController(void);
	CServoController(int iniTarget1, int iniTarget2, int iniTarget3,
		             int iniAcceleration, int iniSpeed);
	~CServoController(void);

	//**************************************************
	// This function sets a target for a single servo
	// Param: channel - physical servo hookup (0 to 5)
	//        target - position to move servo too (0 to 180 degrees)
	// Returns: 0 on success, -1 if errors
	//**************************************************
	int setTarget(int channel, int target);

	//**************************************************
	// This function sets targets for all servos
	// Param: target1 - servo 1 position
	//        target2 - servo 2 position
	//        target3 - servo 3 position
	// Pre-Cond: assumes servos are in channels 0, 1, and 2.
	//           targets correspond to equivalent channel
	// Returns: 0 on success, -1 if errors
	//**************************************************
	int setAllTargets(int target1, int target2, int target3);

	//**************************************************
	// This function sets the acceleration of all servos
	// Param: accel - int (0 to 255)
	// Returns: 0 on success, -1 if errors
	//**************************************************
	int setAcceleration(int accel);

	//**************************************************
	// This function sets the speed of all servos
	// Param: speed - int (0 to 255)
	// Returns: 0 on success, -1 if errors
	//**************************************************
	int setSpeed(int speed);

	//**************************************************
	// This function gets target of servo1
	// Returns: target of servo1
	//**************************************************
	int getServo1();

	//**************************************************
	// This function gets target of servo2
	// Returns: target of servo2
	//**************************************************
	int getServo2();

	//**************************************************
	// This function gets target of servo3
	// Returns: target of servo3
	//**************************************************
	int getServo3();

	//**************************************************
	// This function gets the acceleration
	// Returns: acceleration
	//**************************************************
	int getAcceleration();

	//**************************************************
	// This function gets the speed of all servos
	// Returns: speed
	//**************************************************
	int getSpeed();

private:
	//**************************************************
	// Read from servo controller
	//**************************************************
	int read();

	//**************************************************
	// Write too servo controller
	//**************************************************
	int write();

	int mServo1Pos;
	int mServo2Pos;
	int mServo3Pos;
	int mAcceleration;
	int mSpeed;
};

