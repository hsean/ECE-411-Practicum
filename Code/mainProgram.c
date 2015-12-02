/*
 * T04:
 *      Sean Hendrickson
 *      Waleed Alhaddad
 *      Adrian Steele
 *      Taylor Griffin
 *
 * Self Leveling Table Main Program
 */


#define F_CPU 8000000    // AVR clock frequency in Hz, used by util/delay.h
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>


//definitions
#define MIN_ANGLE 0                                 //minimum servo angle
#define MAX_ANGLE 180                               //maximum servo angle
#define PWM_MIN 800                                 //minimum servo pulse width modulation value
#define PWM_MAX 2400                                //maximum servo pulse width modulation value
#define ACCL_SLV_ADDR 0X3A                          //slave address of accelerometer
#define ACCL_CTRL_REG1 0X2A                         //address of accelerometer control register
#define ACCL_OUT_X_MSB 0x01                         //address of accelerometer x msb register
#define ACCL_NUM_SAMPLE 20                          //number of times to sample the accelerometer
#define USART_FOSC 8000000                          //frequency of the ATmega328 oscillator
#define USART_BAUD 9600                             //target baud rate
#define USART_MYUBRR ((USART_FOSC / (16 * USART_BAUD)) - 1)     //calculation of the value to input into the USART baud rate register
#define USART_NUM_SERVOS 0x03                       //number of servos
#define USART_START_SERVO_CHANNEL 0x00              //starting channel of servos (servos should be placed in successive channels)
#define SERVO_MV_DELAY_MS 5                         //delay to allow servos to move


//prototypes
void USARTinit();
void USARTtransmit(uint8_t data);
void USARTSetTarget(int servo_angle, uint8_t channelNum);
void USARTSetSpeed(uint16_t speed);
void USARTSetAcceleration(uint16_t acceleration);
void USARTSetMultipleTargets(uint16_t * servo_angle);
void TWIInit(void);
void TWIStart();
void TWIStop();
void TWIWrite(uint8_t data);
uint8_t TWIReadACK();
uint8_t TWIReadNACK();
void TWIGetStatus(int operation);
void GetData(uint8_t data);
int ConvertData(uint8_t dataByte);
void InitAcclerometer();
void InitServocontroller();
void SampleAccelerometer(int * x, int * y, int * z);
uint16_t AngleToPWM(int angle);


/*
 * Main function
 */
int main()
{
	int x, y, z;
	int * servoAnglePositions;
	
	//temporary stuff
	DDRD |= (1<<DDD5)|(1<<DDD7);          // set LED pin PD1 to output
	
    //Initialize USART and set servos to home position
	InitServocontroller();
	
	//UPDATE STATUS LEDS
	
	//Initialize accelerometer
	InitAcclerometer();

	//UPDATE STATUS LEDS
	
	while(1)
	{
		//Sample accelerometer
		SampleAccelerometer(&x, &y, &z);
		if(fabs(x) >= 45)
			PORTD |= (1<<PORTD5);   // drive PD1 high
		else
			PORTD &= ~(1<<PORTD5);  // drive PD1 low
		
		if(fabs(y) >= 45)
			PORTD |= (1<<PORTD7);   // drive PD1 high
		else
			PORTD &= ~(1<<PORTD7);  // drive PD1 low
		//UPDATE STATUS LEDS
		
		//SERVO POSITION CALCULATION

		//Send positions to servos
		for(i = 10; i < 90; i += 10)
		{
			USARTSetTarget(i,0x00);
			USARTSetTarget(i,0x01);
			USARTSetTarget(i,0x02);
			_delay_ms(20);     //servo movement delay
		}
		
		
		//UPDATE STATUS LEDS
		
		//UPDATE LED STRIPS
		
		//UPDATE STATUS LEDS
		
		_delay_ms(5);     //servo movement delay
	}
	
	return 0;	
}


/*
 * Initialize USART for ATmega328
 */
void USARTinit()
{
	uint16_t myubrr = USART_MYUBRR;     //the value to store into register UBRR (baud rate register)
	
	//set baud rate
	UBRR0H = (uint8_t) (myubrr>>8);     //get most significant byte
	UBRR0L = (uint8_t) myubrr;          //get least significant byte
	
	//USART control and status register: enable transmitter, disable receiver, write character size bit 2 to 0 (for 8 bit character size)
	UCSR0B = (1 << TXEN0) |      //write 1 to tx enable bit (enable it)
             (0 << RXEN0) |      //0 to rx enable bit (disable it)
             (0 << UCSZ02);      //character size bit 2 = 0;
	
	//set frame format: 1 stop bit, 8 data, parity mode disabled, asynchronous usart
	UCSR0C = (0 << USBS0) |                        //write 0 int stop bit select bit (1 stop bit)
             (1 << UCSZ00) | (1 << UCSZ01) |       //0x03 into character size bits (8-bit data size) bit 0 = 1, bit 1 = 1
             (0 << UPM00) | (0 << UPM01) |         //parity mode disabled: bit 0 = 0, bit 1 = 0
	         (0 << UMSEL01) | (0 << UMSEL00) |     //asynchronous USART mode of operation for USART bit 1 = 0, bit 0 = 0
             (0 << UCPOL0);                        //write 0 to clock parity bit (0 when asynchronous mode is used)

	return;
}


/*
 * Transmit a byte through USART
 * Input: 1 byte of data to transmit
 */
void USARTtransmit(uint8_t data)
{
	//wait for empty transmit buffer
	while(!(UCSR0A & (1<<UDRE0)))
	{
		//do nothing
	}
	
	//put data into buffer and send data
	UDR0 = data;
	
	return;
}


/*
 * Set the position of a single servo
 * Inputs: servo_angle - the angle that you want the servo to be in
 *         channelNum - the number of the channel that the servo is inserted into on the Pololu Micro Maestro
 */        
void USARTSetTarget(int servo_angle, uint8_t channelNum)
{
	uint16_t PWM;     //pulse width modulation value
	
	USARTtransmit(0xAA);     //send the baud rate indication byte to servo controller
	USARTtransmit(0x0C);
	USARTtransmit(0x04);     //set target command byte
	USARTtransmit(channelNum);     //channel number of servo to control
	
	PWM = AngleToPWM(servo_angle);     //convert angle to Pulse Width Modulation value
	
	PWM *= 4;     //wants position in quarter micro seconds
	USARTtransmit(PWM & 0x7F);     //send least significant byte of servo position
	USARTtransmit((PWM >> 7) & 0x7F);     //send most significant byte of servo position
	
	return;	
}


/*
 * The set the speed of the servos
 * Input: speed - the speed of the servos: should be 0 to 256, speed is given in units of (0.25 us) / (10 ms)
 */
void USARTSetSpeed(uint16_t speed)
{
	uint8_t i = USART_START_SERVO_CHANNEL;     //the lowest channel that a servo is plugged into on the Pololu Micro Maestro
	uint8_t max_channel = USART_START_SERVO_CHANNEL + USART_NUM_SERVOS;     //the highest channel that a servo is plugged into on the Pololu Micro Maestro (note: servos should be plugged in using successive channels)
	
	//repeat for each servo to set speed for (speed should be the same for each servo)
	for(i; i < max_channel; ++i)
	{
		USARTtransmit(0x87);     //speed command byte
		USARTtransmit(i);     //channel number of servo to control
		USARTtransmit(speed & 0x7F);     //send least significant byte of speed
		USARTtransmit((speed >> 7) & 0x7F);     //send most significant byte of speed
	}
	
	return;
}


/*
 * Set the acceleration limit for servos
 * Input: the acceleration entered should be a value from 0 to 255, in units of (0x25 us)/(10 ms)/(80 ms). A value of 0 corresponds to no acceleration limit
 */
void USARTSetAcceleration(uint16_t acceleration)
{
	uint8_t i = 0x00;
	
	for(i; i < 0x03; ++i)
	{
		USARTtransmit(0x89);     //acceleration command byte
		USARTtransmit(i);     //channel number of servo
		USARTtransmit(acceleration & 0x7F);     //send least significant byte of acceleration
		USARTtransmit((acceleration >> 7) & 0x7F);     //send most significant byte of acceleration
	}
	
	return;
}


/*
 * Set the positions of all servos (so they receive commands at same time and move in unison)
 * Inputs: servo_angle - array containing the angles of the servos to move, the order should be in ascending channel number order
 */
void USARTSetMultipleTargets(uint16_t * servo_angle)
{
	uint8_t channelNum = USART_START_SERVO_CHANNEL;
	uint16_t servoPWM;
	int i;
	
	USARTtransmit(0xAA);                 //send the baud rate indication byte to servo controller
	USARTtransmit(0x0C);                 //device number
	USARTtransmit(0x1F);                 //command byte: set multiple targets
	USARTtransmit(USART_NUM_SERVOS);     //number of targets
	USARTtransmit(channelNum);           //channel number of first servo
	
	for(i = 0; i < USART_NUM_SERVOS; ++i)
	{
		//convert angles to PWM, multiply value by 4 to get in quarter micro seconds
		servoPWM = AngleToPWM(servo_angle[i]) * 4;
		  
		//position of first servo
		USARTtransmit(servoPWM & 0x7F);     //send least significant byte of servo position
		USARTtransmit((servoPWM >> 7) & 0x7F);     //send most significant byte of servo position
	}
	
	return;
}


/*
 * Initialize TWI (I2C) in ATmega328
 */
void TWIInit(void)
{
	//set SCL to 400kHz
	TWSR = 0x00;
	TWBR = 0x18;
	
	//enable TWI
	TWCR = (1<<TWEN);
	
	return;
}


/*
 * Generate TWI start signal
 */
void TWIStart()
{
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);     //clear interrupt, send start condition, enable TWI
	
	while ((TWCR & (1<<TWINT)) == 0)     //wait until TWI operation finished
	{
		//do nothing
	}
	
	return;
}


/*
 * Send stop signal
 */
void TWIStop()
{
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);     //clear interrupt, send stop condition, enable TWI
	
	return;
}


/*
 * Send a byte through TWI
 * Input: data - the byte you want to transmit
 */
void TWIWrite(uint8_t data)
{
	TWDR = data;     //put the byte into the data register
	TWCR = (1<<TWINT)|(1<<TWEN);     //clear interrupt,  registers to transmit
	
	while ((TWCR & (1<<TWINT)) == 0)     //wait until TWI operation finished
	{
		//do nothing
	}
	
	return;
	
}


/*
 * Read a byte from the TWI, send an acknowledgment back
 * Return: byte received from TWI
 */
uint8_t TWIReadACK()
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);     //clear interrupt, enable TWI, send acknowledgment
	
	while ((TWCR & (1<<TWINT)) == 0)     //wait until TWI operation finished
	{
		//do nothing
	}
	
	return TWDR;     //return data received
}


/*
 * Read a byte from TWI, don't send acknowledgment
 * Return: byte received from TWI
 */
uint8_t TWIReadNACK()
{
	TWCR = (1<<TWINT)|(1<<TWEN);     //clear interrupt, enable TWI
	
	while ((TWCR & (1<<TWINT)) == 0)     //wait until TWI operation finished
	{
		//do nothing
	}
	
	return TWDR;     //return data received
}


/*
 * Check the status register of the TWI
 * Input: operation - the current TWI operation to check the status for (1 for master transmit, 0 for master receive)
 */
void TWIGetStatus(int operation)
{
	uint8_t status;
	
	//mask status
	status = TWSR & 0xF8;

	_delay_ms(500);

	//DO ERROR CHECK
	
	return;
}


/*
 * For testing purposes, output the value of a byte through two LEDs
 * REMOVE BEFORE FINAL COMMIT
 */
void GetData(uint8_t data)
{
	_delay_ms(500);

	int i = 7;
	int j = 128;
	for(i; i >= 0; --i)
	{
		if (data & j)
		{
			PORTD |= (1 << PORTD6);   // drive PD1 high
			_delay_ms(150);
			PORTD &= ~(1 << PORTD6);  // drive PD1 low
			_delay_ms(150);
		}
		else
		{
			PORTD |= (1 << PORTD5);   // drive PD1 high
			_delay_ms(150);
			PORTD &= ~(1 << PORTD5);  // drive PD1 low
			_delay_ms(150);
		}
		j /= 2;
	}
	
	return;
}


/*
 * Convert data received from accelerometer from their 2's complement value into an angle
 * Input: dataMSB - The most significant byte of the two data bytes received from accelerometer
 *        dataLSB - The least significant byte of the two data bytes received from accelerometer
 * Output: Returns the corresponding angle value
 */
int ConvertData(uint8_t dataByte)
{
	int data;     //container for merged MSB and LSB of the accelerometer data
	int angle;     //the converted angle value
	float gValue;     //the accelerometer data converted to G's

	data = dataByte;
	
	//convert from two's complement
	if(data > 0x7F)
	{
		data = ~(data | 0xFFFFFF00)+1;
		data *= -1;
	}

	gValue = (float) data * (float) .015625;     //convert to G's
	angle = (int) (gValue * 90);     //convert to angle
		
	return angle;
}


/*
 * Initialize everything required for accelerometer (initialize TWI in ATmega328, change settings in accelerometer)
 */
void InitAcclerometer()
{
	TWIInit();                   //initialize ATmega328 twi
	TWIStart();                  //send start condition
	TWIWrite(ACCL_SLV_ADDR);     //slave address of accelerometer with R/W bit set to 0 (write)
	TWIWrite(0x2A);              //register to write to: CTRL_REG1 (system control 1 register)
	
	/********* IF NOT USING FAST READ MODE ********/
	/**            TWIWrite(0x21)              **/
	/**********************************************/
	TWIWrite(0x23);       //(bit 5 = 1, bit 4 = 0, bit 3 = 0) - data rate to 50 Hz
	                      //(bit 1 = 1) - F_READ to fast read mode (ignore lsb registers)
						  //(bit 0 = 1) - Active mode (turn the accelerometer on)
						  
	TWIStop();          //send stop condition
}


/*
 * Initialize everything required for servo controller (initialize USART in ATmega328, change servo settings in servo controller, set servos to home position)
 */
void InitServocontroller()
{
	USARTinit();                                                   //initialize ATmega328 USART
	USARTSetAcceleration(0x00);                                    //max acceleration speed
	//USARTSetMutlipleTargets(/*servo1*/,/*servo2*/,/*servo3*/);     //move servos to home position
	_delay_ms(SERVO_MV_DELAY_MS);                                  //servo movement delay
}


/*
 * Sample the accelerometer for the current x y z angle values of the base
 * Input: x - the location to store the x angle value
 *        y - the location to store the y angle value
 *        z - the location to store the z angle value
 */
void SampleAccelerometer(int * x, int * y, int * z)
{
	int i;
	uint8_t xByte, yByte, zByte;
	
	//initialize x y z to 0
	(*x) = 0;
	(*y) = 0;
	(*z) = 0;
	
	//sample the accelerometer multiple times
	for(i = 0; i < ACCL_NUM_SAMPLE; ++i)
	{
		TWIStart();                           //send start condition
		TWIWrite(ACCL_SLV_ADDR);              //slave address with R/W bit set to 0 (write)
		TWIWrite(ACCL_OUT_X_MSB);             //register to read (OUT_X_MSB)
		TWIStart();                           //repeated start condition
		TWIWrite((ACCL_SLV_ADDR | 0x01));     //slave address with R/W bit set to 1 (read)
	
		xByte = TWIReadACK();     //read x y z data
		yByte = TWIReadACK();
		zByte = TWIReadNACK();
				
		TWIStop();     //send stop condition

		//Convert accelerometer data into angles
		(*x) += ConvertData(xByte);
		(*y) += ConvertData(yByte);
		(*z) += ConvertData(zByte);
	}
	
	//average the x y z values
	(*x) /= ACCL_NUM_SAMPLE;
	(*y) /= ACCL_NUM_SAMPLE;
	(*z) /= ACCL_NUM_SAMPLE;
	
	return;
}


/*
 * Convert angle to Pulse Width Modulation value
 * Input: angle - the angle to convert
 * Return: the pulse width modulation value stored in two bytes
 */
uint16_t AngleToPWM(int angle)
{
	int PWM;     //pulse width modulation value to return
	
	//angle definitions incorrect or input out of bounds
	if ((MAX_ANGLE <= MIN_ANGLE) || (angle < MIN_ANGLE) || (angle > MAX_ANGLE))
		PWM = -1;
	
	//convert to pwm value
	else
		PWM = (angle - MIN_ANGLE) * (PWM_MAX - PWM_MIN) / (MAX_ANGLE - MIN_ANGLE) + PWM_MIN;
	
	return (uint16_t) PWM;
}