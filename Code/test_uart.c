#define F_CPU 20000000    // AVR clock frequency in Hz, used by util/delay.h
#include <avr/io.h>
#include <util/delay.h>

//prototypes
void USARTinit();
void USARTtransmit(unsigned char data);
void USARTservoPos(unsigned short position, unsigned char channelNum);
void USARTservoSpeed(unsigned short speed);
void USARTacceleration(unsigned short acceleration);

int main() {
	//DDRD |= (1<<DDD1);          // set LED pin PD1 to output
	USARTinit();
	
	while (1) {
		//PORTD |= (1<<PORTD1);   // drive PD1 high
		USARTservoPos(1250,0x05);
		_delay_ms(500);         // delay 10 ms
		//PORTD &= ~(1<<PORTD1);  // drive PD1 low
		USARTservoPos(1750,0x05);
		_delay_ms(500);         // delay 10 ms
	}
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
	unsigned int fosc = 8000000;     //8MHz clock system oscillator clock frequency ? correct or (20 MHz)? frequency should be multiples of 1.8432 MHz
	unsigned int baud = 9600;       //Target baud rate 9600 bps
	unsigned int myubrr = (fosc / (16 * baud)) - 1;     //value to enter into baud rate registers
	
	//set baud rate
	UBRR0H = (unsigned char) (myubrr>>8);     //get most significant byte
	UBRR0L = (unsigned char) myubrr;          //get least significant byte
	
	//enable transmitter
	UCSR0B = (1 << TXEN0) | (0 << RXEN0);     //write 1 to tx enable bit, 0 to rx enable bit, in usart control and status register b
	
	//set frame format: 8 data, 1 stop bit
	UCSR0C = (0 << USBS0) | (3 << UCSZ00) | (0 << UPM00) | (0 << UPM01);     //write 0 into stop bit select bit (1 stop bit), 3 into character size bits (8-bit data size)
	
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
	USARTtransmit(0x84 & 0x7F);     //set target command byte
	USARTtransmit(channelNum);     //channel number of servo to control
	
	position *= 4;     //wants position in quarter micro seconds
	USARTtransmit(position & 0x7F);     //send least significant byte of servo position
	USARTtransmit((position >> 7) & 0x7F);     //send most significant byte of servo position
	
	return;	
}


//speed is given in units of (0.25 us) / (10 ms)
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


//initialize TWI (I2C)
void TWIInit(void)
{
	//set SCL to 400kHz
	TWSR = 0x00;
	TWBR = 0x0C;
	//enable TWI
	TWCR = (1<<TWEN);
}
//TWIStart AND TWIStop functions to generate start and stop signals
void TWIStart(void)
{
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
}

uint8_t TWIReadACK(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while ((TWCR & (1<<TWINT)) == 0);
	return TWDR;
}
//read byte with NACK
uint8_t TWIReadNACK(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
	return TWDR;
}

uint8_t TWIGetStatus(void)
{
	uint8_t status;
	//mask status
	status = TWSR & 0xF8;
	return status;
}
/*
{	else
	return FALSE;	//Error
	
}
*/