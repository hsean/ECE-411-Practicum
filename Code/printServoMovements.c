#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>


#define FOSC 8000000
#define BAUD 9600
#define MYUBRR ((FOSC / (16 * BAUD)) - 1)
#define MIN_ANGLE 0                                 //minimum servo angle
#define MAX_ANGLE 180                               //maximum servo angle
#define PWM_MIN 800                                 //minimum servo pulse width modulation value
#define PWM_MAX 2400 

//Prototypes
void passByPointer(int * x);
void arrayFun(int * x);
float dToR(int deg);
void PositionSetup(float heightMap[]);
int sort(float target, float heightMap[]);
unsigned short AngleToPWM(int angle);
void servoFix(int x, int y, float table[]);


int main(int argc, char * argv[])
{
    float array[180];
    int i;

    PositionSetup(array);

    //for(i = 0; i < 180; ++i)
    //    printf("array[%d] = %f\n", i, array[i]);
    

    //float derp; 
    //for(derp = 1; derp <= 9; derp += .1)
    //    printf("target = %f, sort returned %d, sort value = %f\n", derp, sort(derp, array), array[sort(derp, array)]);


    servoFix(atoi(argv[1]), atoi(argv[2]), array);


    //int a = 90;

    //printf("SHOW US WHAT YOU GOT: %d\n", (unsigned short)AngleToPWM(a));
   // printf("SHOW US WHAT YOU GOT: %f\n", AngleToPWM(a));
   // printf("SHOW US WHAT YOU GOT: %u\n", AngleToPWM(a));
    
    return 0;
}


void servoFix(int x, int y, float table[])
{
	float xDist;
	float yDist;
	int home = 68;
	
	float s0, s1;
	float s2;
	
	int s0angle;
	int s1angle, s2angle;
	
	
	xDist = 6.7 * sin(x);
	yDist = 6.7 * sin(y);
	
	s0 = table[home] + (7.7365 * tan(dToR(y)));
	s1 = table[home] - (3.8683 * tan(dToR(y))) - (6.7 * tan(dToR(x)));
	s2 = table[home] - (3.8683 * tan(dToR(y))) + (6.7 * tan(dToR(x)));

	//printf("x = %d, y = %d\n",x,y);
	printf("table[%d] = %f\n",home,table[home]);
        //printf("7.7365 * tan(y) = (%f, %d)\n",(7.7365 * tan(dToR(y))),(7.7365 * tan(dToR(y))));	
        printf("s0 = %f, s1 = %f, s2 = %f\n",s0,s1,s2);
	
	s0angle = sort(s0, table);
	s1angle = sort(s1, table);
	s2angle = sort(s2, table);
	
	
	printf("s0angle = %d\n",s0angle);
	printf("s1angle = %d\n",s1angle);
        printf("s2angle = %d\n",s2angle);

	//USARTSetTarget(s0angle, 0x00);
	
	
	//USARTSetTarget(s1angle, 0x01);
	//USARTSetTarget(s2angle, 0x02);
	
	//_delay_ms(800);
	
	return;
}


unsigned short AngleToPWM(int angle)
{
	unsigned short PWM;     //pulse width modulation value to return
	int mina = MIN_ANGLE;
	int maxa = MAX_ANGLE;
	int minp = PWM_MIN;
	int maxp = PWM_MAX;
	
	//angle definitions incorrect or input out of bounds
	//if ((MAX_ANGLE <= MIN_ANGLE) || (angle < MIN_ANGLE) || (angle > MAX_ANGLE))
		//PWM = -1;
	
	//convert to pwm value
	//else
	
	PWM = (unsigned short) ((angle - mina) * (maxp - minp) / (maxa - mina) + minp);
	
	return PWM;
}

void PositionSetup(float heightMap[])
{
	int degree;
	float rad = 0;
	float x = 0;
	float y = 0;
	float y2 = 0;
	
	for (degree = 270; degree <= 450; ++degree)
	{
		//Convert to radians for math functions.
		rad = dToR(degree);
		
		// Find servo arm X pos.
		x = 2.34 * cos(rad);
		// Find servo arm y pos.
		y = 2.34 * sin(rad);
		// Find rod height.
		y2 = sqrt(56.25 - pow(4.0876 - x, 2));
		
		heightMap[degree - 270] = y2 + y;
		
		// Print to screen if needed.
		//printf ("Degree: %d\nX: %f\nY: %f\nY2: %f\nTotal Y: %f\n\n\n", degree, x, y, y2, (y + y2));
	}
}


// Convert from degrees to radians.
float dToR(int deg)
{
        //printf("dToR = %f\n",(2 * M_PI * deg) / 360);
	return (2 * M_PI * deg) / 360;
}


void passByPointer(int * x)
{
    (*x) = 4;
    (*x) *= 4;

    return;
}


void arrayFun(int * x)
{
    printf("x[0] = %d\n",x[0]);
    printf("x[1] = %d\n",x[1]);
    printf("x[2] = %d\n",x[2]);

    return;
}


int sort(float target, float heightMap[])
{
    int mid = 0;
    int min = 0;
    int max = 120;

    while (min <= max)
    {

        mid = (max - min) / 2 + min;
//	printf("mid = %d, min = %d, max = %d\n", mid, min, max);
	if (mid == min)
	    return mid + 1;

        if (heightMap[mid] == target)
            return mid;
        else if (heightMap[mid] < target)
            min = mid + 1;
        else
            max = mid - 1;
    }

    return min;
}

