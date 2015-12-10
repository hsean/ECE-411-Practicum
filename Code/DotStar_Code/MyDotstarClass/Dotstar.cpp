/*******************************************************************************
* Author: Sean Hendrickson
* Date: 12/03/2015
* File: DotstarLEDStrip.cpp
* Description:  This file implements a Dotstar LED Strip from adafruit.com
*               for our ECE 411 self leveling table.  Our implementation
*               uses software SPI to control LEDs.  All code is based on
*               the Adafruit Dotstar Library for Arduino.  
*******************************************************************************/
#include "DotstarLEDStrip.h"


//*************************************************
// Desc: constructor for software SPI
// Param: numPixels - number of pixels on strip
//        data - data pin #
//        clock - clock pin #
//*************************************************
CDotstar::CDotstar(uint16_t numPixels, uint8_t data, uint8_t clock)
{
	mNumPixels = numPixels;
	mDataPin = data;
	mClockPin = clock;
	mBrightness = BRIGHTNESS;
	
	// set all colors to 0x0
	for(int i = 0; i < NUM_PIXELS; ++i)
	{
		for(int j = 0; j < NUM_SUBPIXELS; ++j)
		{
			mPixels[i][j] = 0x0;
		}
	}
}


CDotstar::~CDotstar(void)
{
	// set all variables to 0
	mNumPixels = 0;
	mDataPin = 0;
	mClockPin = 0;
	mRedOffset = 0;          
	mBlueOffset = 0;  
	mGreenOffset = 0; 

	// set all colors to 0x0
	for(int i = 0; i < NUM_PIXELS; ++i)
	{
		for(int j = 0; j < NUM_SUBPIXELS; ++j)
		{
			mPixels[i][j] = 0x0;
		}
	}

	// Stop using SPI
	spiEnd();
}


//*************************************************
// Desc: initialize pins and SPI for output
//*************************************************
void CDotstar::begin(void)
{
	spiInit();
}


//*************************************************
// Desc: sets all pixel data to zero
//*************************************************
void CDotstar::clear(void)
{
	// set all sub-pixel colors to 0x0
	for(int i = 0; i < NUM_PIXELS; ++i)
	{
		for(int j = 0; j < NUM_SUBPIXELS; ++j)
		{
			mPixels[i][j] = 0x00;
		}
	}
}


//*************************************************
// Desc: set global brightness setting
// Param: brightness - 8-bit value
//*************************************************
void CDotstar::setBrightness(uint8_t brightness)
{
	mBrightness = brightness;
}


//*****************************************************************
// Desc: set color of single pixel.  Each pixel is an RGB value 
//       from (0x000000 - 0xFFFFFF).  This packed for is then brocken
//       down into 3 bytes.  1 byte for each subpixel
// Param: pixelPos - pos of pixel on strip
//        color - 32-bit packed form of color, e.g(0xFFFFFF)
//*****************************************************************
void CDotstar::setPixelColor(uint16_t pixelPos, uint32_t color)
{
	// check that position is not out of bounds
	if (pixelPos < mNumPixels)
	{
		mPixels[pixelPos][RED_SUBPIXEL] = (uint8_t)(color >> 16); // shift right 2 bytes
		mPixels[pixelPos][GREEN_SUBPIXEL] = (uint8_t)(color >> 8);  // shift right 1 byte
		mPixels[pixelPos][BLUE_SUBPIXEL] = (uint8_t)color;         // first byte is blue subpixel
	}
}


//*************************************************
// Desc: set color of single pixel 
// Param: LEDPos - pos of pixel on strip
//        red   - 8-bit sub-pixel color
//        blue  - 8-bit sub-pixel color
//        green - 8-bit sub-pixel color
//*************************************************
void CDotstar::setPixelColor(uint16_t pixelPos, uint8_t red,
	uint8_t green, uint8_t blue)
{
	// check that position is not out of bounds
	if (pixelPos < mNumPixels)
	{
		mPixels[pixelPos][RED_SUBPIXEL] = red;
		mPixels[pixelPos][GREEN_SUBPIXEL] = green;
		mPixels[pixelPos][BLUE_SUBPIXEL] = blue;
	}
}


//*************************************************
// Desc: send color data to strip
// TODO: expand documentation to include SPI 
//       protocal
//*************************************************
void CDotstar::show(void)
{
	// TODO: Update this function to use double array
	if (!mPixels) return;    // check if pixels array exists

	uint16_t pixelCounter = mNumPixels;    // counter 
	uint16_t b16 = (uint16_t)mBrightness;  // Type-convert for pixed-point math


	// write data to strip using (bitbang) SPI
	for (i = 0; i < 4; i++)	{ spiOut(START_FRAME); } // start-frame marker
		do {
			spiOut(PIXEL_START);    // pixel start
			for (i = 0; i < NUM_SUBPIXELS; ++i) 
			{	
				spiOut(mPixels[pixelCounter][i]);  // RGB
			}
		} while(--pixelCounter);
	}
	for (i = 0; i < 4; i++) spiOut(END_FRAME);  // End-frame marker 
}


//*************************************************
// Desc: convert individual subpixel to packed form
// Param: red   - 8-bit sub-pixel color
//        green - 8-bit sub-pixel color
//        blue  - 8-bit sub-pixel color
// Ret: return packed pixel value
//*************************************************
uint32_t CDotstar::packColor(uint8_t red, uint8_t green, uint8_t blue)
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
uint32_t CDotstar::getPixelColor(uint16_t pixelPos) const
{
	// check for out of bounds
	if (pixelPos >= mNumPixels)
	{
		return 0;
	}

	// get and pack 8-bit colors into single 32-bit variable
	uint8_t red = mPixels[pixelPos][RED_SUBPIXEL];
	uint8_t green = mPixels[pixelPos][GREEN_SUBPIXEL];
	uint8_t blue = mPixels[pixelPos][BLUE_SUBPIXEL];
	
	return ((uint32_t)red << 16)  |
		   ((uint32_t)green << 8) |
		   (uint32_t)blue;
}


//*************************************************
// Desc: get number of pixels on strip
// Ret: number of pixels on strip
//*************************************************
uint16_t CDotstar::getNumPixels(void)
{
	return mNumPixels;
}


//*************************************************
// Desc: convert individual subpixel to packed form
// Ret: number of pixels on strip
//*************************************************
uint8_t CDotstar::getBrightness(void) const
{
	return mBrightness;
}


//*************************************************
// Desc: get pixel data pointer
// Ret: returns 8-bit pointer to pixel data
//*************************************************
uint8_t* CDotstar::getPixels(void) const
{
	return mPixels;
}


// start software SPI communication
void CDotstar::spiInit(void)
{
	// TODO: NOT IMPLEMENTED
	// set data and clock pins as outputs
	// write low to clock and data pins
	setDataLow();
	setClockLow();
}


// send software SPI data
void CDotstar::spiOut(uint8_t n)
{
	for (uint8_t i = 8; i--; n <<= 1)
	{
		if (n & 0x80)
		{  // set datapin HIGH
			setDataHigh();
		}
		else
		{  // set datapin LOW
			setDataLow();
		}
		// set clockpin HIGH
		setClockHigh();

		// set clockpin LOW
		setClockLow();
	}
}


// end software SPI communication
void CDotstar::spiEnd(void)
{
	// TODO: UNIMPLEMENTED
	// set data and clock pins as inputs
}


//*************************************************
// Desc: change data pin output to HIGH
// Pre-Cond: Datapin must be set as OUTPUT
//*************************************************
void CDotstar::setDataHigh()
{
	// TODO: NOT IMPLEMENTED
#ifdef DEBUG
	std::cout << "DATA_HIGH" << std::endl;
#endif
}


//*************************************************
// Desc: change data pin output to LOW
// Pre-Cond: Datapin must be set as OUTPUT
//*************************************************
void CDotstar::setDataLow()
{
	// TODO: NOT IMPLEMENTED
#ifdef DEBUG
	std::cout << "DATA_LOW" << std::endl;
#endif
}


//*************************************************
// Desc: change clock pin output to HIGH
// Pre-Cond: Clockpin must be set as OUTPUT
//*************************************************
void CDotstar::setClockHigh()
{
	// TODO: NOT IMPLEMENTED
#ifdef DEBUG
	std::cout << "CLOCK_HIGH" << std::endl;
#endif
}


//*************************************************
// Desc: change clock pin output to LOW
// Pre-Cond: Clockpin must be set as OUTPUT
//*************************************************
void CDotstar::setClockLow()
{
	// TODO: NOT IMPLEMENTED
#ifdef DEBUG
	std::cout << "CLOCK_LOW" << std::endl;
#endif
}


