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

int main()
{
	int output;  // hold function output
	
	for(int i = MIN_ANGLE; i <= MAX_ANGLE; ++i)
	{   // brute force possible ranges
		if((output = numberMap(i)) < 0)
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

