#define F_CPU 8000000    // AVR clock frequency in Hz, used by util/delay.h
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>

//prototypes
void USARTinit();
void USARTtransmit(unsigned char data);
void USARTservoPos(unsigned short position, unsigned char channelNum);
void USARTservoSpeed(unsigned short speed);
void USARTacceleration(unsigned short acceleration);
void USARTsetMultiple(unsigned short position);
void TWIInit(void);
void TWIStart(void);
void TWIStop(void);
void TWIWrite(uint8_t u8data);
uint8_t TWIReadACK(void);
uint8_t TWIReadNACK(void);
void TWIGetStatus(void);
int ConvertData(uint8_t data,uint8_t data2);
void GetData(uint8_t data);


int main() {
	
	/*
	// Test LED
	DDRD |= (1<<DDD5);          // set LED pin PD1 to output
	
	while (1) {
		PORTD |= (1<<PORTD5);   // drive PD1 high
		_delay_ms(100);         // delay 500 ms
		PORTD &= ~(1<<PORTD5);  // drive PD1 low
		_delay_ms(100);         // delay 500 ms
	}
	*/
	
	/* Test UART
	int i = 800;
	
	USARTinit();
	USARTacceleration(0x00);
	/USARTservoSpeed(0x00);
	
	while(1)
	{
		for(i; i < 2400; i+=100)
		{
			USARTsetMultiple(i);
			_delay_ms(50);
		}
		for(i; i > 800; i-=100)
		{
			USARTsetMultiple(i);
			_delay_ms(50);
		}
	}
	*/
	
	
	uint8_t x, x2, y, y2, z, z2;
	
	DDRD |= (1<<DDD5)|(1<<DDD6);          // set LED pin PD1 to output
	TWIInit();               //initialize ATmega328 twi
	
	PORTD |= (1<<PORTD5);   // drive PD5 high
	PORTD |= (1<<PORTD6);   // drive PD6 high
	
	_delay_ms(500);
		
	PORTD &= ~(1<<PORTD5);  // drive PD5 low
	PORTD &= ~(1<<PORTD6);  // drive PD6 low	
	_delay_ms(500);
	
	
	//change to fast read mode and slow data rate
	TWIStart();         //start condition
	TWIWrite(0x3A);     //slave address  with R/W bit set to 0 (write)
	TWIWrite(0x2A);     //register to read: CTRL_REG1 (system control 1 register)
	TWIWrite(0x01);     //(bit 5 = 1, bit 4 = 0, bit 3 = 0) - data rate to 50 Hz
	                    //(bit 1 = 1) - F_READ to fast read mode (ignore lsb registers)
	TWIStop();
	
	
	
	while(1)
	{
		//Get xyz data
		TWIStart();              //start condition
		TWIWrite(0x3A);          //slave address with R/W bit set to 0 (write)
		TWIWrite(0x01);          //register to read (OUT_X_MSB)
		TWIStart();              //repeated start condition
		TWIWrite(0x3B);          //slave address with R/W bit set to 1 (read)
		
		x = TWIReadACK();    //read x y z data, change from G's to angle
		x2 = TWIReadACK();
		y = TWIReadACK();
		y2 = TWIReadACK();
		z = TWIReadACK();
		z2 = TWIReadNACK();
		
		TWIStop();               //send stop condition

		int xExt, yExt, zExt;
		xExt = ConvertData(x,x2);
		yExt = ConvertData(y,y2);
		zExt = ConvertData(z,z2);
		
		//GetData(x);
		//GetData(x2);
		
		if(fabs(xExt) >= 45 || fabs(yExt) >= 45)
			PORTD |= (1<<PORTD5);   // drive PD1 high
		else
			PORTD &= ~(1<<PORTD5);  // drive PD1 low
			
		if((fabs(xExt) >= 0 && fabs(xExt) < 45) || (fabs(yExt) >= 0 && fabs(yExt) < 45))
			PORTD |= (1<<PORTD6);   // drive PD1 high
		else
			PORTD &= ~(1<<PORTD6);  // drive PD1 low

	}
	

	return 0;
}


//initialize TWI (I2C)
void TWIInit(void)
{
	//set SCL to 400kHz
	TWSR = 0x00;
	TWBR = 0x18;
	//enable TWI
	TWCR = (1<<TWEN);
}
//TWIStart AND TWIStop functions to generate start and stop signals
void TWIStart(void)
{
	uint8_t status;

	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
}
//send stop signal
void TWIStop(void)
{
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

void TWIWrite(uint8_t u8data)
{
	TWDR = u8data;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
	//TWIGetStatus();
	
}

uint8_t TWIReadACK(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while ((TWCR & (1<<TWINT)) == 0);
	//TWIGetStatus();
	return TWDR;
}
//read byte with NACK
uint8_t TWIReadNACK(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
	//TWIGetStatus();
	return TWDR;
}

void TWIGetStatus(void)
{
	uint8_t status;
	//mask status
	status = TWSR & 0xF8;

	_delay_ms(500);

	int i = 7;
	int j = 128;
	for(i; i >= 0; --i)
	{
		if (status & j)
		{
			PORTD |= (1<<PORTD6);   // drive PD1 high
			_delay_ms(150);
			PORTD &= ~(1<<PORTD6);  // drive PD1 low
			_delay_ms(150);
		}
		else
		{
			PORTD |= (1<<PORTD5);   // drive PD1 high
			_delay_ms(150);
			PORTD &= ~(1<<PORTD5);  // drive PD1 low
			_delay_ms(150);
		}
		j /= 2;
	}
	
	
	return;
}

void GetData(uint8_t data)
{
	_delay_ms(500);

	int i = 7;
	int j = 128;
	for(i; i >= 0; --i)
	{
		if (data & j)
		{
			PORTD |= (1<<PORTD6);   // drive PD1 high
			_delay_ms(150);
			PORTD &= ~(1<<PORTD6);  // drive PD1 low
			_delay_ms(150);
		}
		else
		{
			PORTD |= (1<<PORTD5);   // drive PD1 high
			_delay_ms(150);
			PORTD &= ~(1<<PORTD5);  // drive PD1 low
			_delay_ms(150);
		}
		j /= 2;
	}
	
	
	return;
}


int ConvertData(uint8_t data,uint8_t data2)
{
	    int dataExt;
		float superAngle;

	    dataExt = (data << 8) | data2;
		dataExt = dataExt >> 4;

	    /*if(data > 0x7F)
			dataExt = ~(dataExt | 0xFFFF0000)+1;
			dataExt *= -1;
		*/
		superAngle = (float) dataExt / (float)(1 << 11) * (float)2;
		int angle = (int) (superAngle * 90);
		
		return angle;
}

/*
 * USART Interface
 *
 * Version 1.0
 * team: T04
 */ 


//initialize USART
void USARTinit()
{
	//unsigned int fosc = 8000000;     //8MHz clock system oscillator clock frequency ? correct or (20 MHz)? frequency should be multiples of 1.8432 MHz
	//unsigned int baud = 9600;       //Target baud rate 9600 bps
	//unsigned int myubrr = (fosc / (16 * baud)) - 1;     //value to enter into baud rate registers
	unsigned int myubrr = 51;
	//set baud rate
	UBRR0H = (unsigned char) (myubrr>>8);     //get most significant byte
	UBRR0L = (unsigned char) myubrr;          //get least significant byte
	
	//USART control and status register: enable transmitter, disable receiver, write character size bit 2 to 0 (for 8 bit character size)
	UCSR0B = (1 << TXEN0) |      //write 1 to tx enable bit (enable it)
             (0 << RXEN0) |      //0 to rx enable bit (disable it)
             (0 << UCSZ02);      //character size bit 2 = 0;
	
	//set frame format: 8 data, 1 stop bit, parity mode disabled, asynchronous usart
	UCSR0C = (0 << USBS0) |       //write 0 int stop bit select bit (1 stop bit)
             (1 << UCSZ00) |      //0x03 into character size bits (8-bit data size) bit 0 = 1
             (1 << UCSZ01) |      //                                             bit 1 = 1
             (0 << UPM00) |       //parity mode disabled: bit 0 = 0
             (0 << UPM01) |       //parity mode disabled: bit 1 = 0
	         (0 << UMSEL01) |     //asynchronous USART mode of operation for USART bit 1 = 0
             (0 << UMSEL00) |     //                                               bit 0 = 0
             (0 << UCPOL0);       //write 0 to clock parity bit (0 when asynchronous mode is used)

	return;
}




//transmit a byte through USART
void USARTtransmit(unsigned char data)
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


//position should be from 992 microseconds to 2000 microseconds where 992 is minimum, 2000 is maximum position, (1500 is neutral position)
void USARTservoPos(unsigned short position, unsigned char channelNum)
{
	USARTtransmit(0xAA);     //send the baud rate indication byte to servo controller
	USARTtransmit(0x0C);
	USARTtransmit(0x04);     //set target command byte
	USARTtransmit(channelNum);     //channel number of servo to control
	
	position *= 4;     //wants position in quarter micro seconds
	USARTtransmit(position & 0x7F);     //send least significant byte of servo position
	USARTtransmit((position >> 7) & 0x7F);     //send most significant byte of servo position
	
	return;	
}


void USARTsetMultiple(unsigned short position)
{
	unsigned char channelNum = 0x07;
	
	USARTtransmit(0xAA);     //send the baud rate indication byte to servo controller
	USARTtransmit(0x0C);     //device number
	USARTtransmit(0x1F);     //set target command byte
	USARTtransmit(0x03);     //number of servos
	USARTtransmit(channelNum);     //channel number of servo to control
	
	position *= 4;     //wants position in quarter micro seconds
      
        //position of first servo
	USARTtransmit(position & 0x7F);     //send least significant byte of servo position
	USARTtransmit((position >> 7) & 0x7F);     //send most significant byte of servo position
        //position of second servo
	USARTtransmit(position & 0x7F);     //send least significant byte of servo position
	USARTtransmit((position >> 7) & 0x7F);     //send most significant byte of servo position
        //position of third servo
	USARTtransmit(position & 0x7F);     //send least significant byte of servo position
	USARTtransmit((position >> 7) & 0x7F);     //send most significant byte of servo position
	return;
}


//0 to 256 speed is given in units of (0.25 us) / (10 ms)
void USARTservoSpeed(unsigned short speed)
{
	unsigned char i = 0x00;
	
	for(i; i < 0x03; ++i)
	{
		USARTtransmit(0x87);     //speed command byte
		USARTtransmit(i);     //channel number of servo to control
		USARTtransmit(speed & 0x7F);     //send least significant byte of speed
		USARTtransmit((speed >> 7) & 0x7F);     //send most significant byte of speed
	}
	
	return;
}


//acceleration limit is a value from 0 to 255 in units of (0x25 us)/(10 ms)/(80 ms). A value of 0 corresponds to no acceleration limit
void USARTacceleration(unsigned short acceleration)
{
	unsigned char i = 0x00;
	
	for(i; i < 0x03; ++i)
	{
		USARTtransmit(0x89);     //acceleration command byte
		USARTtransmit(i);     //channel number of servo
		USARTtransmit(acceleration & 0x7F);     //send least significant byte of acceleration
		USARTtransmit((acceleration >> 7) & 0x7F);     //send most significant byte of acceleration
	}
	
	return;
}


