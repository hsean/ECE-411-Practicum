/*******************************************************************************
* Author: Sean Hendrickson
* Date: 12/03/2015
* File: Dotstar.h
* Description:  This file models a Dotstar LED Strip from adafruit.com
*               for our ECE 411 self leveling table.  Our implementation
*               uses software SPI to control LEDs.  All code is based on
*               the Adafruit Dotstar Library for Arduino.  
*******************************************************************************/
#pragma once
#include <cstdint>
#include <iostream>

#define NUM_PIXELS 15
#define NUM_SUBPIXELS 3
#define BRIGHTNESS 255
#define RED_SUBPIXEL 0
#define GREEN_SUBPIXEL 0
#define BLUE_SUBPIXEL 0

#define START_FRAME 0x0
#define PIXEL_START 0xFF
#define END_FRAME 0xFF

class CDotstar
{
public:

	//*************************************************
	// Desc: constructor for software SPI
	// Param: numPixels - number of pixels on strip
	//        data - data pin #
	//        clock - clock pin #
	//*************************************************
	CDotstar(uint16_t numPixels, uint8_t data, uint8_t clock);
	
	~CDotstar(void);

	//*************************************************
	// Desc: initialize pins and SPI for output
	//*************************************************
	void begin(void);

	//*************************************************
	// Desc: sets all pixel data to zero
	//*************************************************
	void clear(void);

	//*************************************************
	// Desc: set global brightness setting
	// Param: brightness - 8-bit value
	//*************************************************
	void setBrightness(uint8_t brightness);

	//*************************************************
	// Desc: set color of single pixel
	// Param: LEDPos - pos of pixel on strip
	//        color - 32-bit packed form of color, e.g(0xFFFFFF)
	//*************************************************
	void setPixelColor(uint16_t pixelPos, uint32_t color);

	//*************************************************
	// Desc: set color of single pixel 
	// Param: LEDPos - pos of pixel on strip
	//        red   - 8-bit sub-pixel color
	//        blue  - 8-bit sub-pixel color
	//        green - 8-bit sub-pixel color
	//*************************************************
	void setPixelColor(uint16_t pixelPos, uint8_t red, 
		               uint8_t green, uint8_t blue);

	//*************************************************
	// Desc: send color data to strip
	//*************************************************
	void show(void);

	//*************************************************
	// Desc: convert individual subpixel to packed form
	// Param: red   - 8-bit sub-pixel color
	//        blue  - 8-bit sub-pixel color
	//        green - 8-bit sub-pixel color
	//*************************************************
	uint32_t packColor(uint8_t red, uint8_t green, uint8_t blue);

	//*************************************************
	// Desc: get color of pixel in packed form
	// Param: pixelPos - pixel to get color of
	// Ret: packed form of RGB pixels
	//*************************************************
	uint32_t getPixelColor(uint16_t pixelPos) const;

	//*************************************************
	// Desc: get number of pixels on strip
	// Ret: number of pixels on strip
	//*************************************************
	uint16_t getNumPixels(void);

	//*************************************************
	// Desc: convert individual subpixel to packed form
	// Ret: number of pixels on strip
	//*************************************************
	uint8_t getBrightness(void) const;

	//*************************************************
	// Desc: get color of specific subpixel
	// Param: pixels - pixel
	//        subpixel - position of pixel in array
	// Ret: color of subpixel as 8-bit value
	//*************************************************
	uint8_t getPixels(uint8_t pixel, uint8_t subpixel);

private:
	uint16_t mNumPixels;                    // Number of pixels
	uint8_t mDataPin;                       // data pin #
	uint8_t mClockPin;                      // clock pin #
	uint8_t mBrightness;                    // global brightness setting
	uint8_t mPixels[NUM_PIXELS][NUM_SUBPIXELS]; // array of pixels [pixel][subpixel]

	void spiInit(void);            // start software SPI communication
	void spiOut(uint8_t pixelPos); // send software SPI data
	void spiEnd(void);             // end software SPI communication
	void setDataHigh();
	void setDataLow();
	void setClockHigh();
	void setClockLow();
	
};

