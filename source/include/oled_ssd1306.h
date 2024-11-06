/*
 * OLED
 * 安装以下库：
 * lib_deps =
    adafruit/Adafruit SSD1306@^2.5.11
    adafruit/Adafruit GFX Library@^1.11.10
    adafruit/Adafruit BusIO@^1.16.1
    添加以下库：
    lib_deps =
    Wire
    SPI
 */
#pragma once

#ifndef __OLED_SSD1306
#define __OLED_SSD1306

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#ifdef __cplusplus
extern "C"
{
#endif



#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
  

#define NUMFLAKES 10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT 16
#define LOGO_WIDTH 16

    static const unsigned char PROGMEM logo_bmp[] =
        {0b00000000, 0b11000000,
         0b00000001, 0b11000000,
         0b00000001, 0b11000000,
         0b00000011, 0b11100000,
         0b11110011, 0b11100000,
         0b11111110, 0b11111000,
         0b01111110, 0b11111111,
         0b00110011, 0b10011111,
         0b00011111, 0b11111100,
         0b00001101, 0b01110000,
         0b00011011, 0b10100000,
         0b00111111, 0b11100000,
         0b00111111, 0b11110000,
         0b01111100, 0b11110000,
         0b01110000, 0b01110000,
         0b00000000, 0b00110000};

    void init_my_oled(void); // 初始化OLED
    void testdrawline();     // Draw many lines

    void testdrawrect();      // Draw rectangles (outlines)
    void testfillrect();      // Draw rectangles (filled)
    void testdrawcircle();    // Draw circles (outlines)
    void testfillcircle();    // Draw circles (filled)
    void testdrawroundrect(); // Draw rounded rectangles (outlines)
    void testfillroundrect(); // Draw rounded rectangles (filled)
    void testdrawtriangle();  // Draw triangles (outlines)
    void testfilltriangle();  // Draw triangles (filled)
    void testdrawchar();      // Draw characters of the default font
    void testdrawstyles();    // Draw 'stylized' characters
    void testscrolltext();    // Draw scrolling text
    void testdrawbitmap();    // Draw a small bitmap image
    void testanimate(const uint8_t *bitmap, uint8_t w, uint8_t h);
#ifdef __cplusplus
}
#endif
#endif /*__RY_BLE*/
