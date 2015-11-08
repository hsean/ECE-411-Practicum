#define F_CPU 20000000    // AVR clock frequency in Hz, used by util/delay.h
#include <avr/io.h>
#include <util/delay.h>

int main() {
	DDRD |= (1<<DDD1);          // set LED pin PD1 to output
	while (1) {
		PORTD |= (1<<PORTD1);   // drive PD1 high
		_delay_ms(10);         // delay 10 ms
		PORTD &= ~(1<<PORTD1);  // drive PD1 low
		_delay_ms(10);         // delay 10 ms
	}
}