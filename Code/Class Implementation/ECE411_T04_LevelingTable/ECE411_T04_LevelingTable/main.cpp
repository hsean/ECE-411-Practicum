/*******************************************************************************
 * Author: Sean Hendrickson
 * Date: 11/26/2015
 * File: main.cpp
 * Desc: This is the main application file for ECE 411 T04 Leveling Table.
 *       This file will contain tests for the different functions and classes
 *       used for this application
 *******************************************************************************/
#include "SystemManager.h"
#include <iostream>
#include <stdlib.h>
#include <time.h>
#define DEFAULT_ADDRESS 0
#define WINDOWS
#define VERBOSE
#define NUM_ACCEL_TEST_RUNS 5
#define NUM_SERVOCONTROLLER_RUNS 5
using std::cout;
using std::endl;

/* PROCEDURE PROTOTYPES */
void LEDTest();
void LEDStripTest();
void accelTest();
void servoControllerTest();
void printAccel(CAccelerometer* accel);
void printLEDStrip(CLEDStrip* LEDStrip, int numLEDs);
void printAccel(CAccelerometer* accel);
void printServoController(CServoController* myServos);

int main()
{
	LEDTest();  // run LED class tests

#ifdef VERBOSE
	#ifdef WINDOWS
		system("PAUSE");
	#endif
#endif

	LEDStripTest();  // run LEDStrip class tests

#ifdef VERBOSE
	#ifdef WINDOWS
		system("PAUSE");
	#endif
#endif

	accelTest();  // run Accelerometer class tests

#ifdef VERBOSE
#ifdef WINDOWS
	system("PAUSE");
#endif
#endif

	servoControllerTest();  // run servoController class tests

	// exit program
	cout << "All tests complete" << endl;
#ifdef WINDOWS
	system("PAUSE");
#endif

	return 0;
}

/* PROCEDURE IMPLEMENTATION */
//**************************************************
// This function tests the CLED class 
//**************************************************
void LEDTest()
{
	CLED myLED(DEFAULT_ADDRESS,
			   DEFAULT_COLOR,
			   DEFAULT_BRIGHTNESS);    // LED object
	int address, color, brightness;
	bool error = false;

	// check that all values initialized to 0
	address = myLED.getAddress();
	color = myLED.getColor();
	brightness = myLED.getBrightness();

	if ((address != DEFAULT_ADDRESS) || 
		(color != DEFAULT_COLOR) || 
		(brightness != DEFAULT_BRIGHTNESS))
	{
		cout << "ERROR: LED defaults not set" << endl;
		error = true;
	}

	// noew change LED values and check that it worked
	address = 1;
	color = 2;
	brightness = 3;

	myLED.setAddress(address);
	myLED.setColor(color);
	myLED.setBrightness(brightness);

	if ((address != myLED.getAddress()) ||
		(color != myLED.getColor()) ||
		(brightness != myLED.getBrightness()))
	{
		cout << "ERROR: LED values did not change" << endl;
		error = true;
	}

	// output text if no errors found
	if (!error)
	{
		cout << "CLED test SUCCESS" << endl;
	}

}


//**************************************************
// This function tests the CLEDList class 
//**************************************************
void LEDStripTest()
{
	int numLEDs = 15;    // define number of LEDs in strip
	int color = 0;       // holds color
	int brightness = 0;  // holds brightness
	int val = 0;         // hold return values for error check
	int randVal = 0;     // holds rand() function return
	bool error = false;  // final error check

	CLEDStrip LEDStrip(numLEDs, color, brightness);
	srand(time(NULL));   // initialize random seed

#ifdef VERBOSE
	// print current state of LED Strip
	printLEDStrip(&LEDStrip, numLEDs); 
#endif

	// changes values of all LEDs
	for (int i = 0; i < numLEDs; ++i)
	{
		LEDStrip.setColor(i, i + 1);
		LEDStrip.setBrightness(i, i + 1);

		// check write
		if ((val = LEDStrip.getColor(i)) != i + 1)
		{
			error = true;
		}
		if ((val = LEDStrip.getBrightness(i)) != i + 1)
		{
			error = true;
		}
	}

#ifdef VERBOSE
	// print out changes
	printLEDStrip(&LEDStrip, numLEDs);
#endif

	// changes values of all LEDs
	for (int i = 0; i < numLEDs; ++i)
	{
		// set and check color
		LEDStrip.setColor(i, randVal = rand() % 256);  // RN: 0 & 255)
		if ((val = LEDStrip.getColor(i)) != randVal)
		{
			error = true;
		}

		// set and check brightness
		LEDStrip.setBrightness(i, randVal = rand() % 256);  // RN: 0 & 255)
		if ((val = LEDStrip.getBrightness(i)) != randVal)
		{
			error = true;
		}
	}

#ifdef VERBOSE
	// print out changes after random function
	printLEDStrip(&LEDStrip, numLEDs);
#endif

	// output text if no errors found
	if (!error)
	{
		cout << "CLEDStrip test SUCCESS" << endl;
	}
	else
	{
		cout << "CLEDStrip test ERRORS FOUND" << endl;
	}
}

void printLEDStrip(CLEDStrip* LEDStrip, int numLEDs)
{
	for (int i = 0; i < numLEDs; ++i)
	{
		cout << "LED: " << i << 
			    ", c: " << LEDStrip->getColor(i) << 
				", b: " << LEDStrip->getBrightness(i) << endl;
	}
}


//**************************************************
// This function tests the CAccelerometer class 
//**************************************************
void accelTest()
{
	int randVal1, randVal2, randVal3;  // hold rand() results
	int x, y, z;  // coordinates from accelerometer
	bool error = false;  // used for test output

	CAccelerometer myAccel;  // initialize accelerometer object
	srand(time(NULL));   // initialize random seed

	// pass in some random data between 0 and 180 and check results
	for (int i = 0; i < NUM_ACCEL_TEST_RUNS; ++i)
	{
		// generate random values
		randVal1 = rand() % 181;
		randVal2 = rand() % 181;
		randVal3 = rand() % 181;

		// set values
		myAccel.setData(randVal1, randVal2, randVal3);

		// get data
		myAccel.getData(x, y, z);

		// check data
		if ((randVal1 != x) || (randVal2 != y) || (randVal3 != z))
		{
			error = true;
		}

#ifdef VERBOSE
		printAccel(&myAccel);
#endif
	}

	// output text if no errors found
	if (!error)
	{
		cout << "CAccelerometer test SUCCESS" << endl;
	}
	else
	{
		cout << "CAccelerometer test ERRORS FOUND" << endl;
	}
}

void printAccel(CAccelerometer* accel)
{
	int x, y, z;  // hold output from accelerometer
	accel->getData(x, y, z);
	cout << "X:" << x << 
		  ", Y:" << y << 
		  ", Z:" << z << endl;
}

//**************************************************
// This function tests the CServoController class 
//**************************************************
void servoControllerTest()
{
	bool error = false;  // determine test success
	int servo1 = DEFAULT_SERVO_ANGLE;
	int servo2 = DEFAULT_SERVO_ANGLE;
	int servo3 = DEFAULT_SERVO_ANGLE;
	int speed = DEFAULT_SPEED;
	int accel = DEFAULT_ACCELERATION;
	int rand1, rand2, rand3, rand4, rand5;  // capture random numbers

	CServoController myServos;
	srand(time(NULL));   // initialize random seed

	for (int i = 0; i < NUM_SERVOCONTROLLER_RUNS; ++i)
	{
		// set data
		myServos.setTarget(0, rand1 = rand() % 181);
		myServos.setTarget(1, rand2 = rand() % 181);
		myServos.setTarget(2, rand3 = rand() % 181);
		myServos.setSpeed(rand4 = rand() % 256);
		myServos.setAcceleration(rand5 = rand() % 256);

		// get data
		servo1 = myServos.getServo1();
		servo2 = myServos.getServo2();
		servo3 = myServos.getServo3();
		speed = myServos.getSpeed();
		accel = myServos.getAcceleration();

		// check data
		if ((servo1 != rand1) ||
			(servo2 != rand2) ||
			(servo3 != rand3) ||
			(speed != rand4) ||
			(accel != rand5))
		{
			error = true;
		}

#ifdef VERBOSE
		printServoController(&myServos);
#endif
	}

	// output text if no errors found
	if (!error)
	{
		cout << "CServoController test SUCCESS" << endl;
	}
	else
	{
		cout << "CServoController test ERRORS FOUND" << endl;
	}
}

void printServoController(CServoController* myServos)
{
	cout << "S1:" << myServos->getServo1()
		<< ", S2:" << myServos->getServo2()
		<< ", S3:" << myServos->getServo3()
		<< ", SP:" << myServos->getSpeed()
		<< ", AC:" << myServos->getAcceleration() << endl;
}

 