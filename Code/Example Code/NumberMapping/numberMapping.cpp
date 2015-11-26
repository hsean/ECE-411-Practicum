// Author: Sean Hendrickson
// This program maps user input from degrees 0 to 180 
// to PWM values from 600 to 2400

#include <iostream>

#define PWM_MAX 2400
#define PWM_MIN 600
#define MAX_ANGLE 180
#define MIN_ANGLE 0

using std::cout;
using std::endl;

int numberMap(int input);
int numberMap(int input, int inputMin, int inputMax,
              int ouputMin, int outputMax);

int main()
{
	int output = 0;  // hold function output
	int inputCheck = -6;  // input that is out of bounds
	/*
	for(int i = MIN_ANGLE; i <= MAX_ANGLE; ++i)
	{   // brute force possible ranges
		if((output = numberMap(i)) < 0)
		{
			cout << "ERROR: input out of bounds" << endl;
		}
		cout << "input = " << i 
		     << ", output = " << output << endl;
	}
	*/
	
	// checking if error works
        if((output = numberMap(inputCheck, MIN_ANGLE, MAX_ANGLE, PWM_MIN, PWM_MAX)) < 0)
	{
	    cout << "ERROR: input out of bounds" << endl;
	}
	    cout << "input = " << inputCheck 
	         << ", output = " << output << endl;

	for(int i = MIN_ANGLE; i <= MAX_ANGLE; ++i)
	{   // brute force possible ranges
		if((output = numberMap(i, MIN_ANGLE, MAX_ANGLE, PWM_MIN, PWM_MAX)) < 0)
		{
			cout << "ERROR: input out of bounds" << endl;
		}
		cout << "input = " << i 
		     << ", output = " << output << endl;
	}
	
	return 0;
}


int numberMap(int x)
{
	if((MAX_ANGLE - MIN_ANGLE) == 0)
	{   // divide by 0 check 
		return -1;
	}
	
	if((x < MIN_ANGLE) || (x > MAX_ANGLE))
	{  // input out of bounds
		return -1;
	}
	
	return (x - MIN_ANGLE) * (PWM_MAX - PWM_MIN) / (MAX_ANGLE - MIN_ANGLE) + PWM_MIN;
}

/* This function maps one range of integers to another range of integers
 * Pre-cond: No parameters can be less than 0
 * return: -1 if error, else returns mapped output
 */
int numberMap(int input, int inputMin, int inputMax,
              int outputMin, int outputMax)
{
	int error = -1;  // error value to be returned
	
	// check if any parameters are bellow 0
	if((input < 0) || (inputMin < 0) || (inputMax < 0) 
		       || (outputMin < 0) || (outputMax < 0))
	{
		return error;
	}
	
	// check if input is out of bounds
	if((input < inputMin) || (input > inputMax))
	{
		return error;
	}
	
	// check for divide by 0 error
	if((inputMax - inputMin) == 0)
	{
		return error;
	}
	
	// return new mapping
	return (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;
}
