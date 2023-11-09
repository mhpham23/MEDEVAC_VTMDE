#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

#define TFT_DC 10
#define TFT_CS 9
#define TFT_RST -1

void setup_LCD(Adafruit_ILI9341 tft);
void LCD_printStabOn(Adafruit_ILI9341 tft);
void LCD_printStabOff(Adafruit_ILI9341 tft);
void LCD_printSensorMode(Adafruit_ILI9341 tft);
void LCD_printBLEstatus(Adafruit_ILI9341 tft);
void LCD_printData(Adafruit_ILI9341 tft, float angleX, float angleXvel);
