/******************************************************************************
* Author: Sean Hendrickson
* Date: 12/09/2015
* File: Dotstar.h
* Desc: This libary contains code to control Adafruit's Dotstar LED strip.
******************************************************************************/
#ifndef __AVR_ATmega328P__
	#define __AVR_ATmega328P__
#endif

#include <stdint.h>
#include <avr/io.h>

#define NUM_PIXELS 15
#define NUM_SUBPIXELS 3
#define BRIGHTNESS 255      // leave brightness at 255
#define RED_SUBPIXEL 0      // subpixel position in array
#define GREEN_SUBPIXEL 1    // subpixel position in array
#define BLUE_SUBPIXEL 2     // subpixel position in array
#define MAX_COLOR 4F

#define DATA_PIN 15
#define CLOCK_PIN 14
#define START_FRAME 0x0    // SPI START Frame is 0x00
#define PIXEL_START 0xFF   // SPI START Code
#define END_FRAME 0xFF     // SPI END Frame is 0xFF


// pin ports enumeration
enum PORT { ePORTB = 0, ePORTC = 1, ePORTD = 2, eERROR = 3 };

// logic level of output
enum LogicLevel{eLOW = 0, eHIGH = 1};


//***************************************************
// Struct: Dotstar
//***************************************************
struct Dotstar
{
	uint16_t mNumPixels;                    // Number of pixels
	uint8_t mDataPin;                       // data pin #
	uint8_t mClockPin;                      // clock pin #
	uint8_t mBrightness;                    // global brightness setting
	uint8_t mPixels[NUM_PIXELS][NUM_SUBPIXELS]; // array of pixels [pixel][subpixel]
};


/* FUNCTION PROTOTYPES */
void begin(struct Dotstar* strip);          // initialize pins and SPI for output
void clear(struct Dotstar* strip);          // sets all pixel data to zero
void setBrightness(struct Dotstar* strip,   // set global brightness setting 
				   uint8_t brightness);
void setPixelColor(struct Dotstar* strip,   // set color of single pixel
			       uint16_t pixelPos,
	               uint32_t color);
void show(struct Dotstar* strip);           // send color data to strip
uint32_t packColor(uint8_t red,             // convert individual subpixel to packed form
	               uint8_t green,
	               uint8_t blue);
uint32_t getPixelColor(struct Dotstar* strip,  // get color of pixel in packed form
	                   uint16_t pixelPos);
uint16_t getNumPixels(struct Dotstar* strip);  // get number of pixels on strip
uint8_t getBrightness(struct Dotstar* strip);  // convert individual subpixel to packed form
uint8_t getPixels(struct Dotstar* strip,       // get color of specific subpixel
	              uint8_t pixel,
	              uint8_t subpixel);

void spiInit(struct Dotstar* strip);        // start software SPI communication
void spiOut(uint8_t pixelPos);              // send software SPI data
void spiEnd(struct Dotstar* strip);         // end software SPI communication


int pinOutput(uint8_t pinNumber, uint8_t logicOut);  // set output if pin
int pinInput(uint8_t pinNumber);  // change pin to input
uint8_t getPort(uint8_t pinNumber);    //Return PORT of passed pin
void bPortOut(uint8_t pin, uint8_t outputLevel);
void cPortOut(uint8_t pin, uint8_t outputLevel);
void dPortOut(uint8_t pin, uint8_t outputLevel);
void bPortIn(uint8_t pin);
void cPortIn(uint8_t pin);
void dPortIn(uint8_t pin);



/* FUNCTION IMPLEMENTATION */

//*************************************************
// Desc: initialize pins and SPI for output
//*************************************************
void begin(struct Dotstar* strip)
{
	strip->mNumPixels = NUM_PIXELS;
	strip->mDataPin = DATA_PIN;
	strip->mClockPin = CLOCK_PIN;
	strip->mBrightness = BRIGHTNESS;

	// set all colors to 0x0
	for (int i = 0; i < NUM_PIXELS; ++i)
	{
		for (int j = 0; j < NUM_SUBPIXELS; ++j)
		{
			strip->mPixels[i][j] = 0x0;
		}
	}

	// initialize SPI
	spiInit(&strip);
}


//*************************************************
// Desc: sets all pixel data to zero
//*************************************************
void clear(struct Dotstar* strip)
{
	// set all sub-pixel colors to 0x0
	for (int i = 0; i < NUM_PIXELS; ++i)
	{
		for (int j = 0; j < NUM_SUBPIXELS; ++j)
		{
			strip->mPixels[i][j] = 0x00;
		}
	}
	
	show(strip);
}


//*************************************************
// Desc: set global brightness setting
// Param: brightness - 8-bit value
//*************************************************
void setBrightness(struct Dotstar* strip, uint8_t brightness)
{
	strip->mBrightness = brightness;
}


//*****************************************************************
// Desc: set color of single pixel.  Each pixel is an RGB value 
//       from (0x000000 - 0xFFFFFF).  This packed form is then broken
//       down into 3 bytes.  1 byte for each subpixel
// Param: pixelPos - pos of pixel on strip
//        color - 32-bit packed form of color, e.g(0xFFFFFF)
//*****************************************************************
void setPixelColor(struct Dotstar* strip, uint16_t pixelPos, uint32_t color)
{
	// check that position is not out of bounds
	if ((pixelPos < strip->mNumPixels) && (pixelPos >= 0))
	{
		strip->mPixels[pixelPos][RED_SUBPIXEL] = (uint8_t)(color >> 16); // shift right 2 bytes
		strip->mPixels[pixelPos][GREEN_SUBPIXEL] = (uint8_t)(color >> 8);  // shift right 1 byte
		strip->mPixels[pixelPos][BLUE_SUBPIXEL] = (uint8_t)color;         // first byte is blue subpixel
	}
}


//*************************************************
// Desc: send color data to strip
//*************************************************
void show(struct Dotstar* strip)
{
	if (!strip->mPixels) return;    // check if pixels array exists

	uint16_t pixelCounter = strip->mNumPixels;    // counter 
	uint16_t b16 = (uint16_t)strip->mBrightness;  // Type-convert for pixed-point math


	// write data to strip using (bitbang) SPI
	for (int i = 0; i < 4; ++i)
	{
		spiOut(START_FRAME);
	} // start-frame marker

	// send color for each subpixel
	for (int i = 0; i < pixelCounter; ++i)
	{
		spiOut(PIXEL_START);    // pixel start
		for (int j = 0; j < NUM_SUBPIXELS; ++j)
		{
			spiOut(strip->mPixels[i][j]);  // RGB
		}
	}

	// send finished command
	for (int i = 0; i < 4; i++)
	{
		spiOut(END_FRAME);
	} // End-frame marker 
}


//*************************************************
// Desc: convert individual subpixel to packed form
// Param: red   - 8-bit sub-pixel color
//        green - 8-bit sub-pixel color
//        blue  - 8-bit sub-pixel color
// Ret: return packed pixel value
//*************************************************
uint32_t packColor(uint8_t red, uint8_t green, uint8_t blue)
{
	return ((uint32_t)red << 16) | ((uint32_t)green << 8) | blue;
}


//*************************************************
// Desc: convert individual subpixel to packed form
// Param: red   - 8-bit sub-pixel color
//        green - 8-bit sub-pixel color
//        blue  - 8-bit sub-pixel color
// Ret: 0 if out of bounds error, else
//      return packed 24-bit value
//*************************************************
uint32_t getPixelColor(struct Dotstar* strip, uint16_t pixelPos)
{
	// check for out of bounds
	if (pixelPos >= strip->mNumPixels)
	{
		return 0;
	}

	// get and pack 8-bit colors into single 32-bit variable
	uint8_t red = strip->mPixels[pixelPos][RED_SUBPIXEL];
	uint8_t green = strip->mPixels[pixelPos][GREEN_SUBPIXEL];
	uint8_t blue = strip->mPixels[pixelPos][BLUE_SUBPIXEL];

	return ((uint32_t)red << 16) |
		((uint32_t)green << 8) |
		(uint32_t)blue;
}


//*************************************************
// Desc: get number of pixels on strip
// Ret: number of pixels on strip
//*************************************************
uint16_t getNumPixels(struct Dotstar* strip)
{
	return strip->mNumPixels;
}


//*************************************************
// Desc: convert individual subpixel to packed form
// Ret: number of pixels on strip
//*************************************************
uint8_t getBrightness(struct Dotstar* strip)
{
	return strip->mBrightness;
}


//*************************************************
// Desc: get color of specific subpixel
// Param: pixels - pixel
//        subpixel - position of pixel in array
// Ret: color of subpixel as 8-bit value
//*************************************************
uint8_t getPixels(struct Dotstar* strip, uint8_t pixel, uint8_t subpixel)
{
	return strip->mPixels[pixel][subpixel];
}


// start software SPI communication
void spiInit(struct Dotstar* strip)
{
	// set data pin low
	pinOutput(DATA_PIN, eLOW);

	// set clock pin low
	pinOutput(CLOCK_PIN, eLOW);
}


// send software SPI data
void spiOut(uint8_t pixelPos)
{
	for (uint8_t i = 8; i--; pixelPos <<= 1)
	{
		if (pixelPos & 0x80)
		{  // set datapin HIGH
			pinOutput(DATA_PIN, eHIGH);
		}
		else
		{  // set datapin LOW
			pinOutput(DATA_PIN, eLOW);
		}
		// set clockpin HIGH
		pinOutput(CLOCK_PIN, eHIGH);

		// set clockpin LOW
		pinOutput(CLOCK_PIN, eLOW);
	}
}


// end software SPI communication
void spiEnd(struct Dotstar* strip)
{
	// set data and clock pins as inputs
	pinInput(DATA_PIN);
	pinInput(CLOCK_PIN);
}


//*******************************************************
// Desc: sets the passed pin as an OUTPUT for ATMEGA328
// Param: pinNumber - pin to configure
//        logicOut - 1 is output HIGH
//                   0 is output LOW
// Ret: -1 if pin out of bounds,
//       0 if successful
//*******************************************************
int pinOutput(uint8_t pinNumber, uint8_t logicOut)
{
	uint8_t pinPort = getPort(pinNumber);
	switch (pinPort)
	{
		case ePORTB:
		{
			bPortOut(pinNumber, logicOut);
			break;
		}
		case ePORTC:
		{
			cPortOut(pinNumber, logicOut);
			break;
		}
		case ePORTD:
		{
			dPortOut(pinNumber, logicOut);
			break;
		}
		defalut : return -1;
	}
}


//*******************************************************
// Desc: sets the passed pin as an INPUT for ATMEGA328
// Param: pinNumber - pin to configure
// Ret: -1 if pin out of bounds,
//       0 if successful
//*******************************************************
int pinInput(uint8_t pinNumber)
{
	uint8_t pinPort = getPort(pinNumber);
	switch (pinPort)
	{
		case ePORTB:
		{
				  bPortIn(pinNumber);
				  break;
		}
		case ePORTC:
		{
				  cPortIn(pinNumber);
				  break;
		}
		case ePORTD:
		{
				  dPortIn(pinNumber);
				  break;
		}
		defalut: return -1;
	}
}


// Return PORT of passed pin. These pins are
// specific to the ATMEGA328 28pin DIP layout
uint8_t getPort(uint8_t pinNumber)
{
	// find and output port
	switch (pinNumber)
	{
	case 1: return ePORTC;
	case 2: return ePORTD;
	case 3: return ePORTD;
	case 4: return ePORTD;
	case 5: return ePORTD;
	case 6: return ePORTD;
	case 9: return ePORTB;
	case 10: return ePORTB;
	case 11: return ePORTD;
	case 12: return ePORTD;
	case 13: return ePORTD;
	case 14: return ePORTB;
	case 15: return ePORTB;
	case 16: return ePORTB;
	case 17: return ePORTB;
	case 18: return ePORTB;
	case 19: return ePORTB;
	case 23: return ePORTC;
	case 24: return ePORTC;
	case 25: return ePORTC;
	case 26: return ePORTC;
	case 27: return ePORTC;
	case 28: return ePORTC;
	default: return eERROR;  // pin not found
	}
}


// sets pin as output by writing 1 to DDRB register
// and using PORTB to set logic level
void bPortOut(uint8_t pin, uint8_t level)
{
	// set pin as output and Logic Level
	switch (pin)
	{
	case 9: // (PIN9 = PB6)
	{	DDRB |= (1 << PORTB6);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTB |= (1 << DDB6)) : (PORTB &= ~(1 << DDB6));
	break; }
	case 10: // (PIN10 = PB7)
	{	DDRB |= (1 << PORTB7);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTB |= (1 << DDB7)) : (PORTB &= ~(1 << DDB7));
	break; }
	case 14: // (PIN14 = PB0)
	{	DDRB |= (1 << PORTB0);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTB |= (1 << DDB0)) : (PORTB &= ~(1 << DDB0));
	break; }
	case 15: // (PIN15 = PB1)
	{	DDRB |= (1 << PORTB1);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTB |= (1 << DDB1)) : (PORTB &= ~(1 << DDB1));
	break; }
	case 16: // (PIN16 = PB2)
	{	DDRB |= (1 << PORTB2);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTB |= (1 << DDB2)) : (PORTB &= ~(1 << DDB2));
	break; }
	case 17: // (PIN17 = PB3)
	{	DDRB |= (1 << PORTB3);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTB |= (1 << DDB3)) : (PORTB &= ~(1 << DDB3));
	break; }
	case 18: // (PIN18 = PB4)
	{	DDRB |= (1 << PORTB4);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTB |= (1 << DDB4)) : (PORTB &= ~(1 << DDB4));
	break; }
	case 19: // (PIN19 = PB5)
	{	DDRB |= (1 << PORTB5);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTB |= (1 << DDB5)) : (PORTB &= ~(1 << DDB5));
	break; }
	}
}


// sets pin as output by writing 1 to DDRC register
// and using PORTC to set logic level
void cPortOut(uint8_t pin, uint8_t level)
{
	// set pin as output and Logic Level
	switch (pin)
	{
	case 1: // (PIN1 = PC6)
	{	DDRC |= (1 << PORTC6);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTC |= (1 << DDC6)) : (PORTB &= ~(1 << DDC6));
	break; }
	case 23: // (PIN23 = PC0)
	{	DDRC |= (1 << PORTC0);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTC |= (1 << DDC0)) : (PORTB &= ~(1 << DDC0));
	break; }
	case 24: // (PIN24 = PC1)
	{	DDRC |= (1 << PORTC1);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTC |= (1 << DDC1)) : (PORTB &= ~(1 << DDC1));
	break; }
	case 25: // (PIN25 = PC2)
	{	DDRC |= (1 << PORTC2);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTC |= (1 << DDC2)) : (PORTB &= ~(1 << DDC2));
	break; }
	case 26: // (PIN26 = PC3)
	{	DDRC |= (1 << PORTC3);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTC |= (1 << DDC3)) : (PORTB &= ~(1 << DDC3));
	break; }
	case 27: // (PIN27 = PC4)
	{	DDRC |= (1 << PORTC4);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTC |= (1 << DDC4)) : (PORTB &= ~(1 << DDC4));
	break; }
	case 28: // (PIN28 = PC5)
	{	DDRC |= (1 << PORTC5);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTC |= (1 << DDC5)) : (PORTB &= ~(1 << DDC5));
	break; }
	}
}


// sets pin as output by writing 1 to DDRD register
// and using PORTD to set logic level
void dPortOut(uint8_t pin, uint8_t level)
{
	// set pin as output and Logic Level
	switch (pin)
	{
	case 2: // (PIN2 = PD0)
	{	DDRD |= (1 << PORTD0);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTD |= (1 << DDD0)) : (PORTB &= ~(1 << DDD0));
	break; }
	case 3: // (PIN3 = PD1)
	{	DDRD |= (1 << PORTD1);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTD |= (1 << DDD1)) : (PORTB &= ~(1 << DDD1));
	break; }
	case 4: // (PIN4 = PD2)
	{	DDRD |= (1 << PORTD2);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTD |= (1 << DDD2)) : (PORTB &= ~(1 << DDD2));
	break; }
	case 5: // (PIN5 = PD3)
	{	DDRD |= (1 << PORTD3);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTD |= (1 << DDD3)) : (PORTB &= ~(1 << DDD3));
	break; }
	case 6: // (PIN6 = PD4)
	{	DDRD |= (1 << PORTD4);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTD |= (1 << DDD4)) : (PORTB &= ~(1 << DDD4));
	break; }
	case 11: // (PIN11 = PD5)
	{	DDRD |= (1 << PORTD5);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTD |= (1 << DDD5)) : (PORTB &= ~(1 << DDD5));
	break; }
	case 12: // (PIN12 = PD6)
	{	DDRD |= (1 << PORTD6);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTD |= (1 << DDD6)) : (PORTB &= ~(1 << DDD6));
	break; }
	case 13: // (PIN13 = PD7)
	{	DDRD |= (1 << PORTD7);  // set as output
	(level == eHIGH) ?      // if HIGH, set output HIGH, else set LOW
		(PORTD |= (1 << DDD7)) : (PORTB &= ~(1 << DDD7));
	break; }
	}
}


// sets pin as input by writing 0 to DDRB register
void bPortIn(uint8_t pin)
{
	// set pin as input
	switch (pin)
	{
	case 9: // (PIN9 = PB6)
	{	DDRB &= ~(1 << PORTB6); break; } // set as input
	case 10: // (PIN10 = PB7)
	{	DDRB &= ~(1 << PORTB7); break; }// set as input
	case 14: // (PIN14 = PB0)
	{	DDRB &= ~(1 << PORTB0); break; } // set as input
	case 15: // (PIN15 = PB1)
	{	DDRB &= ~(1 << PORTB1); break; } // set as input
	case 16: // (PIN16 = PB2)
	{	DDRB &= ~(1 << PORTB2); break; } // set as input
	case 17: // (PIN17 = PB3)
	{	DDRB &= ~(1 << PORTB3); break; } // set as input
	case 18: // (PIN18 = PB4)
	{	DDRB &= ~(1 << PORTB4); break; } // set as input
	case 19: // (PIN19 = PB5)
	{	DDRB &= ~(1 << PORTB5); break; } // set as input
	}
}


// sets pin as input by writing 0 to DDRC register
void cPortIn(uint8_t pin)
{
	// set pin as input
	switch (pin)
	{
	case 1: // (PIN1 = PC6)
	{	DDRC &= ~(1 << PORTC6); break; } // set as input
	case 23: // (PIN23 = PC0)
	{	DDRC &= ~(1 << PORTC0); break; } // set as input
	case 24: // (PIN24 = PC1)
	{	DDRC &= ~(1 << PORTC1); break; } // set as input
	case 25: // (PIN25 = PC2)
	{	DDRC &= ~(1 << PORTC2); break; } // set as input
	case 26: // (PIN26 = PC3)
	{	DDRC &= ~(1 << PORTC3); break; } // set as input
	case 27: // (PIN27 = PC4)
	{	DDRC &= ~(1 << PORTC4); break; } // set as input
	case 28: // (PIN28 = PC5)
	{	DDRC &= ~(1 << PORTC5); break; } // set as input
	}
}


// sets pin as input by writing 0 to DDRD register
void dPortIn(uint8_t pin)
{
	// set pin as input
	switch (pin)
	{
	case 2: // (PIN2 = PD0)
	{	DDRD &= ~(1 << PORTD0); break; } // set as input
	case 3: // (PIN3 = PD1)
	{	DDRD &= ~(1 << PORTD1); break; } // set as input
	case 4: // (PIN4 = PD2)
	{	DDRD &= ~(1 << PORTD2); break; } // set as input
	case 5: // (PIN5 = PD3)
	{	DDRD &= ~(1 << PORTD3); break; } // set as input
	case 6: // (PIN6 = PD4)
	{	DDRD &= ~(1 << PORTD4); break; }  // set as input
	case 11: // (PIN11 = PD5)
	{	DDRD &= ~(1 << PORTD5); break; } // set as input
	case 12: // (PIN12 = PD6)
	{	DDRD &= ~(1 << PORTD6); break; } // set as input
	case 13: // (PIN13 = PD7)
	{	DDRD &= ~(1 << PORTD7); break; } // set as input
	}
}



