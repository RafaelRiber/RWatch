/**
  The MIT License (MIT)
  Copyright (c) 2016 Rafael Riber
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
**/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>
#include <RTCZero.h>
#include <Chrono.h>
#include <LowPower.h>
#include <Button.h>
#include <SparkFunBQ27441.h>
#include <StopWatch.h>
#include <Adafruit_DRV2605.h>
#include <BLEPeripheral.h>
#include <BLEUtil.h>

#define OLED_MOSI   10 //FINAL: ??, PROTO: 10
#define OLED_CLK   11  //FINAL: ??, PROTO: 11
#define OLED_DC    12  //FINAL: ??, PROTO: 12
#define OLED_CS    A0  //FINAL: ??, PROTO: A0
#define OLED_RESET A1  //FINAL: ??, PROTO: A1
#define B1Pin       5  //FINAL: ??, PROTO: 5
#define B2Pin       6  //FINAL: ??, PROTO: 6
#define B3Pin       9  //FINAL: ??, PROTO: 9
#define BLE_REQ   A3   //FINAL: ??, PROTO: A3
#define BLE_RDY   A4   //FINAL: ??, PROTO: A4
#define BLE_RST   A5   //FINAL: ??, PROTO: A5

Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
Adafruit_DRV2605 drv;
Adafruit_BNO055 bno = Adafruit_BNO055();
Adafruit_BMP280 bmp;
StopWatch stopwatch;
Chrono myChrono(Chrono::SECONDS);
Button Button1(B1Pin, false, true, 20);  //Select
Button Button2(B2Pin, false, true, 20); //Right
Button Button3(B3Pin, false, true, 20); //Left
BLEPeripheral blePeripheral = BLEPeripheral(BLE_REQ, BLE_RDY, BLE_RST);
BLEBondStore bleBondStore;
// remote services
BLERemoteService ancsService = BLERemoteService("7905f431b5ce4e99a40f4b1e122d00d0");
// remote characteristics
BLERemoteCharacteristic ancsNotificationSourceCharacteristic = BLERemoteCharacteristic("9fbf120d630142d98c5825e699a21dbd", BLENotify);
RTCZero rtc;

const unsigned int BATTERY_CAPACITY = 500;
int timer;
float QFF = 1020.00;
int chronoDeci;
int chronoHour;
int chronoSeconds;
int chronoMinutes;
int chronoCounting = 0;
bool shouldBeSleeping = false;
int inMenu = 0;
int modeInMenu;
int mode;
//int timeout = 10000;
int timeout = 10000000000;
int first_millis;
bool bleConnected = false;

char* daysOfTheWeek[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};

const unsigned char RWatch_logo[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x01, 0xe0, 0x00,
  0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x01, 0xe0, 0x00,
  0x3f, 0xff, 0xf0, 0xf3, 0xc0, 0x1e, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x01, 0xe0, 0x00,
  0x3f, 0xff, 0xf8, 0xf3, 0xc0, 0x1e, 0x00, 0xf0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x01, 0xe0, 0x00,
  0x3f, 0xff, 0xf8, 0xf1, 0xe0, 0x3f, 0x01, 0xe0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x01, 0xe0, 0x00,
  0x38, 0x00, 0x38, 0xf1, 0xe0, 0x3f, 0x01, 0xe0, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x01, 0xe0, 0x00,
  0x38, 0x00, 0x38, 0xf1, 0xe0, 0x7f, 0x01, 0xe7, 0xff, 0xf0, 0xff, 0xc3, 0xff, 0xf9, 0xff, 0xfc,
  0x38, 0x00, 0x38, 0xf0, 0xf0, 0x7f, 0x83, 0xc7, 0xff, 0xf8, 0xff, 0xc7, 0xff, 0xf9, 0xff, 0xfe,
  0x38, 0x00, 0x38, 0xf0, 0xf0, 0x7f, 0x83, 0xc7, 0xff, 0xfc, 0xff, 0xc7, 0xff, 0xf9, 0xff, 0xfe,
  0x38, 0x00, 0x38, 0xf0, 0x70, 0xf3, 0xc3, 0x80, 0x00, 0x3c, 0xf0, 0x07, 0x00, 0x01, 0xe0, 0x0e,
  0x38, 0x00, 0x38, 0xf0, 0x78, 0xf3, 0xc7, 0x80, 0x00, 0x3c, 0xf0, 0x07, 0x00, 0x01, 0xe0, 0x0e,
  0x3f, 0xff, 0xf8, 0xf0, 0x78, 0xf3, 0xc7, 0x80, 0x00, 0x3c, 0xf0, 0x07, 0x00, 0x01, 0xe0, 0x0e,
  0x3f, 0xff, 0xf8, 0xf0, 0x3d, 0xe1, 0xef, 0x07, 0xff, 0xfc, 0xf0, 0x07, 0x00, 0x01, 0xe0, 0x0e,
  0x3f, 0xff, 0xf8, 0xf0, 0x3d, 0xe1, 0xef, 0x07, 0xff, 0xfc, 0xf0, 0x07, 0x00, 0x01, 0xe0, 0x0e,
  0x3f, 0xff, 0xe0, 0xf0, 0x3d, 0xc0, 0xef, 0x07, 0xff, 0xfc, 0xf0, 0x07, 0x00, 0x01, 0xe0, 0x0e,
  0x38, 0x0f, 0x80, 0xf0, 0x1f, 0xc0, 0xfe, 0x07, 0x80, 0x3c, 0xf0, 0x07, 0x00, 0x01, 0xe0, 0x0e,
  0x38, 0x07, 0xc0, 0xf0, 0x1f, 0xc0, 0xfe, 0x07, 0x80, 0x3c, 0xf0, 0x07, 0x00, 0x01, 0xe0, 0x0e,
  0x38, 0x03, 0xe0, 0xf0, 0x1f, 0x80, 0x7e, 0x07, 0x80, 0x3c, 0xf0, 0x07, 0x00, 0x01, 0xe0, 0x0e,
  0x38, 0x01, 0xf0, 0xf0, 0x0f, 0x80, 0x7c, 0x07, 0xff, 0xfc, 0x7f, 0xc7, 0xff, 0xf9, 0xe0, 0x0e,
  0x38, 0x00, 0xf0, 0xf0, 0x0f, 0x80, 0x7c, 0x03, 0xff, 0xfc, 0x7f, 0xc7, 0xff, 0xf9, 0xe0, 0x0e,
  0x38, 0x00, 0xf8, 0xf0, 0x07, 0x00, 0x38, 0x03, 0xff, 0xfc, 0x3f, 0xc3, 0xff, 0xf9, 0xe0, 0x0e,
  0x38, 0x00, 0x78, 0xf0, 0x07, 0x00, 0x38, 0x00, 0xff, 0xfc, 0x1f, 0xc1, 0xff, 0xf9, 0xe0, 0x0e,
  0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char batt_icon_empty16x8[] PROGMEM = {
  0xff, 0xfe, 0x80, 0x02, 0xa0, 0x03, 0xa0, 0x03, 0xa0, 0x03, 0xa0, 0x03, 0x80, 0x02, 0xff, 0xfe
};

const unsigned char batt_icon_low16x8[] PROGMEM = {
  0xff, 0xfe, 0x80, 0x02, 0xbc, 0x03, 0xbc, 0x03, 0xbc, 0x03, 0xbc, 0x03, 0x80, 0x02, 0xff, 0xfe
};

const unsigned char batt_icon_high16x8[] PROGMEM = {
  0xff, 0xfe, 0x80, 0x02, 0xbf, 0xc3, 0xbf, 0xc3, 0xbf, 0xc3, 0xbf, 0xc3, 0x80, 0x02, 0xff, 0xfe
};

const unsigned char batt_icon_full16x8[] PROGMEM = {
  0xff, 0xfe, 0x80, 0x02, 0xbf, 0xfb, 0xbf, 0xfb, 0xbf, 0xfb, 0xbf, 0xfb, 0x80, 0x02, 0xff, 0xfe
};

const unsigned char chg_icon8x8[] PROGMEM = {
  0x00, 0x08, 0x10, 0x30, 0x7e, 0x0c, 0x08, 0x10
};

const unsigned char BLE8x8[] PROGMEM = {
  0x10, 0x18, 0x54, 0x38, 0x54, 0x18, 0x10, 0x00,
};

void interr1()
{}

void interr2()
{}

void menu()
{
  first_millis = millis();
  //Button1 = select
  //Button2 = right
  //Button3 = left

  switch (modeInMenu)
  {
    case 0://main screen
      display.clearDisplay();
      display.drawRect(0, 0, 128, 64, 1);
      display.setTextSize(1);
      display.setTextColor(BLACK, WHITE);
      display.setCursor(2, 2);
      display.println("  Main screen");
      display.setTextColor(WHITE);
      display.println(" Stopwatch");
      display.println(" Altimeter");
      display.println(" Compass");
      display.println(" Accelerometer");
      display.println(" Power");
      display.display();
      if (Button1.wasPressed())
      {
        button_buzz();
        mode = 0;
      }
      break;
    case 1://Stopwatch
      display.clearDisplay();
      display.drawRect(0, 0, 128, 64, 1);
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(2, 2);
      display.println(" Main screen");
      display.setTextColor(BLACK, WHITE);
      display.println("  Stopwatch");
      display.setTextColor(WHITE);
      display.println(" Altimeter");
      display.println(" Compass");
      display.println(" Accelerometer");
      display.println(" Power");
      display.display();
      if (Button1.wasPressed())
      {
        button_buzz();
        mode = 1;
      }
      break;
    case 2://Alti
      display.clearDisplay();
      display.drawRect(0, 0, 128, 64, 1);
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(2, 2);
      display.println(" Main screen");
      display.println(" Stopwatch");
      display.setTextColor(BLACK, WHITE);
      display.println("  Altimeter");
      display.setTextColor(WHITE);
      display.println(" Compass");
      display.println(" Accelerometer");
      display.println(" Power");
      display.display();
      if (Button1.wasPressed())
      {
        button_buzz();
        mode = 2;
      }
      break;
    case 3: //Compass
      display.clearDisplay();
      display.drawRect(0, 0, 128, 64, 1);
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(2, 2);
      display.println(" Main screen");
      display.println(" Stopwatch");
      display.println(" Altimeter");
      display.setTextColor(BLACK, WHITE);
      display.println("  Compass");
      display.setTextColor(WHITE);
      display.println(" Accelerometer");
      display.println(" Power");
      display.display();
      if (Button1.wasPressed())
      {
        button_buzz();
        mode = 3;
      }
      break;
    case 4: //Accelprint
      display.clearDisplay();
      display.drawRect(0, 0, 128, 64, 1);
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(2, 2);
      display.println(" Main screen");
      display.println(" Stopwatch");
      display.println(" Altimeter");
      display.println(" Compass");
      display.setTextColor(BLACK, WHITE);
      display.println("  Accelerometer");
      display.setTextColor(WHITE);
      display.println(" Power");
      display.display();
      if (Button1.wasPressed())
      {
        button_buzz();
        mode = 4;
      }
      break;
    case 5: //Power
      display.clearDisplay();
      display.drawRect(0, 0, 128, 64, 1);
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(2, 2);
      display.println(" Main screen");
      display.println(" Stopwatch");
      display.println(" Altimeter");
      display.println(" Compass");
      display.println(" Accelerometer");
      display.setTextColor(BLACK, WHITE);
      display.println("  Power");
      display.display();
      if (Button1.wasPressed())
      {
        mode = 5;
      }
      break;
  }

  if (Button2.wasPressed())
  {
    button_buzz();
    modeInMenu++;
  }

  if (Button3.wasPressed())
  {
    button_buzz();
    modeInMenu = modeInMenu - 1;
  }

  if (modeInMenu > 5)
  {
    modeInMenu = 5;
  }

  if (modeInMenu < 0)
  {
    modeInMenu = 0;
  }
}

void show_mode()
{
  switch (mode)
  {
    case 0:
      main_screen();
      break;
    case 1:
      chrono();
      break;
    case 2:
      altimeter();
      break;
    case 3:
      compass();
      break;
    case 4:
      accel();
      break;
    case 5:
      powerStats();
      break;
  }
}

void main_screen()
{
  display.clearDisplay();

  display.drawRect(0, 0, 128, 64, 1);

  display.drawFastHLine(0, 14, 128, 1);
  display.drawFastHLine(0, 48, 128, 1);

  //BATT STAT
  if (lipo.soc() == 100)
  {
    display.drawBitmap(5, 52, batt_icon_full16x8, 16, 8, 1);
  }
  if (lipo.soc() < 100 && lipo.soc() > 50)
  {
    display.drawBitmap(5, 52, batt_icon_high16x8, 16, 8, 1);
  }
  if (lipo.soc() < 50 && lipo.soc() > 30)
  {
    display.drawBitmap(5, 52, batt_icon_low16x8, 16, 8, 1);
  }
  if (lipo.soc() < 30)
  {
    display.drawBitmap(5, 52, batt_icon_empty16x8, 16, 8, 1);
  }

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(25, 52);
  display.print(lipo.soc());
  display.print("%");

  int pw = lipo.power();

  if (pw > 0)
  {
    display.drawBitmap(50, 52, chg_icon8x8, 8, 8, 1);
  }

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(24, 4);

  int wd ; // WeekDay ( Sunday - 0,Monday- 1,.....)

  int d = rtc.getDay();

  int m = rtc.getMonth();

  int y = 2000 + rtc.getYear();

  wd = ((d += m < 3 ? y-- : y - 2, 23 * m / 9 + d + 4 + y / 4 - y / 100 + y / 400) % 7);


  //Serial.print(wd);
  display.print(daysOfTheWeek[wd]);
  display.print(" ");

  if (rtc.getDay() < 10)
  {
    display.print("0");
    display.print(rtc.getDay());
  }
  if (rtc.getDay() >= 10)
  {
    display.print(rtc.getDay());
  }
  display.print('/');
  if (rtc.getMonth() < 10)
  {
    display.print("0");
    display.print(rtc.getMonth());
  }
  if (rtc.getMonth() >= 10)
  {
    display.print(rtc.getMonth());
  }
  display.print('/');
  display.print(rtc.getYear());

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(15, 25);

  if (rtc.getHours() < 10)
  {
    display.print('0');
    display.print(rtc.getHours());
  }
  else
  {
    display.print(rtc.getHours());
  }
  display.print(':');
  if (rtc.getMinutes() < 10)
  {
    display.print('0');
    display.print(rtc.getMinutes());
  }
  else
  {
    display.print(rtc.getMinutes());
  }
  display.print(':');
  if (rtc.getSeconds() < 10)
  {
    display.print('0');
    display.print(rtc.getSeconds());
  }
  else
  {
    display.print(rtc.getSeconds());
  }

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(100, 52);
  int temp = bmp.readTemperature();
  display.print(temp);
  display.print((char)247);
  display.print("C");

  if (bleConnected == true)
  {
    display.drawBitmap(90, 52, BLE8x8, 8, 8, 1);
  }

  display.display();
  timer = millis();
}

void chrono()
{
  first_millis = millis();
  display.clearDisplay();
  display.drawRect(0, 0, 128, 64, 1);
  //Button1 = select
  //Button2 = right
  //Button3 = left

  switch (chronoCounting)
  {
    case 0:
      if (stopwatch.isRunning() == true)
      {
        stopwatch.stop();
      }
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(15, 25);
      //display.print(stopwatch.elapsed());
      if (stopwatch.elapsed() / 1000 < 60)
      {
        if ((stopwatch.elapsed() / 100000) < 10)
        {
          display.print("0");
          display.print(stopwatch.elapsed() / 100000);
        }
        else
        {
          display.print(stopwatch.elapsed() / 100000);
        }
        display.print(":");
        if ((stopwatch.elapsed() / 1000) < 10)
        {
          display.print("0");
          display.print(stopwatch.elapsed() / 1000);
        }
        else
        {
          display.print(stopwatch.elapsed() / 1000);
        }
        display.print(".");
        display.print(stopwatch.elapsed() / 100 % 10);
        display.print(stopwatch.elapsed() / 10 % 10);
      }
      if (stopwatch.elapsed() / 1000 > 60)
      {
        chronoMinutes = (stopwatch.elapsed() / 1000) / 60;
        chronoSeconds = (stopwatch.elapsed() / 1000) % 60;
        if (chronoMinutes < 10)
        {
          display.print("0");
          display.print(chronoMinutes);
        }
        else
        {
          display.print(chronoMinutes);
        }
        display.print(":");
        if (chronoSeconds < 10)
        {
          display.print("0");
          display.print(chronoSeconds);
        }
        else
        {
          display.print(chronoSeconds);
        }
        display.print(".");
        display.print(stopwatch.elapsed() / 100 % 10);
        display.print(stopwatch.elapsed() / 10 % 10);
      }
      break;
    case 1:
      if (stopwatch.isRunning() == false)
      {
        stopwatch.start();
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(15, 25);
        //display.print(stopwatch.elapsed());
        if (stopwatch.elapsed() / 1000 < 60)
        {
          if ((stopwatch.elapsed() / 100000) < 10)
          {
            display.print("0");
            display.print(stopwatch.elapsed() / 100000);
          }
          else
          {
            display.print(stopwatch.elapsed() / 100000);
          }
          display.print(":");
          if ((stopwatch.elapsed() / 1000) < 10)
          {
            display.print("0");
            display.print(stopwatch.elapsed() / 1000);
          }
          else
          {
            display.print(stopwatch.elapsed() / 1000);
          }
          display.print(".");
          display.print(stopwatch.elapsed() / 100 % 10);
          display.print(stopwatch.elapsed() / 10 % 10);
        }
        if (stopwatch.elapsed() / 1000 > 60)
        {
          chronoMinutes = (stopwatch.elapsed() / 1000) / 60;
          chronoSeconds = (stopwatch.elapsed() / 1000) % 60;
          if (chronoMinutes < 10)
          {
            display.print("0");
            display.print(chronoMinutes);
          }
          else
          {
            display.print(chronoMinutes);
          }
          display.print(":");
          if (chronoSeconds < 10)
          {
            display.print("0");
            display.print(chronoSeconds);
          }
          else
          {
            display.print(chronoSeconds);
          }
          display.print(".");
          display.print(stopwatch.elapsed() / 100 % 10);
          display.print(stopwatch.elapsed() / 10 % 10);
        }
      }
      else
      {
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(15, 25);
        //display.print(stopwatch.elapsed());
        if (stopwatch.elapsed() / 1000 < 60)
        {
          if ((stopwatch.elapsed() / 100000) < 10)
          {
            display.print("0");
            display.print(stopwatch.elapsed() / 100000);
          }
          else
          {
            display.print(stopwatch.elapsed() / 100000);
          }
          display.print(":");
          if ((stopwatch.elapsed() / 1000) < 10)
          {
            display.print("0");
            display.print(stopwatch.elapsed() / 1000);
          }
          else
          {
            display.print(stopwatch.elapsed() / 1000);
          }
          display.print(".");
          display.print(stopwatch.elapsed() / 100 % 10);
          display.print(stopwatch.elapsed() / 10 % 10);
        }
        if (stopwatch.elapsed() / 1000 > 60)
        {
          chronoMinutes = (stopwatch.elapsed() / 1000) / 60;
          chronoSeconds = (stopwatch.elapsed() / 1000) % 60;
          if (chronoMinutes < 10)
          {
            display.print("0");
            display.print(chronoMinutes);
          }
          else
          {
            display.print(chronoMinutes);
          }
          display.print(":");
          if (chronoSeconds < 10)
          {
            display.print("0");
            display.print(chronoSeconds);
          }
          else
          {
            display.print(chronoSeconds);
          }
          display.print(".");
          display.print(stopwatch.elapsed() / 100 % 10);
          display.print(stopwatch.elapsed() / 10 % 10);
        }
      }
      break;
  }
  if (Button2.wasPressed())
  {
    button_buzz();
    if (chronoCounting <= 1)
    {
      chronoCounting++;
    }
    if (chronoCounting > 1)
    {
      chronoCounting = 0;
    }
  }


  if (Button3.wasPressed())
  {
    button_buzz();
    stopwatch.reset();
    chronoCounting = 0;
  }

  display.display();
}

void altimeter()
{

  first_millis = millis();
  display.clearDisplay();
  display.drawRect(0, 0, 128, 64, 1);
  display.drawFastHLine(0, 14, 128, 1);
  display.drawFastHLine(0, 48, 128, 1);
  //Button1 = select
  //Button2 = right
  //Button3 = left

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(25, 4);
  display.print("QFF : ");
  display.print(QFF);

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(40, 25);
  int alt = bmp.readAltitude(QFF);
  display.print(alt);
  display.print("m");

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(34, 52);
  display.print("Temp: ");
  int temp = bmp.readTemperature();
  display.print(temp);
  display.print((char)247);
  display.print("C");

  display.display();

  if (Button2.wasPressed())
  {
    button_buzz();
    QFF = QFF + 0.20;
  }

  if (Button3.wasPressed())
  {
    button_buzz();
    QFF = QFF - 0.20;
  }
}

void compass()
{
  first_millis = millis();
  display.clearDisplay();
  display.drawRect(0, 0, 128, 64, 1);
  //Button1 = select
  //Button2 = right
  //Button3 = left

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(45, 4);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  display.print("Cal.: ");
  display.print(mag);

  imu::Vector<3> compass = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  int heading = atan2(compass.y(), compass.x()) * (180 / PI); // angle in degrees

  if (heading < 0)
  {
    heading = 360 + heading;
  }


  /*
    CALIB ?
    //  adafruit_bno055_offsets_t calibData;
    //
    //  //Calibration Results:
    //  //Accelerometer: 65515 65520 13
    //  //Gyro: 0 0 0
    //  //Mag: 710 118 65414
    //  //Accel Radius: 1000
    //  //Mag Radius: 636
    //
    //  calibData.accel_offset_x = 65515;
    //  calibData.accel_offset_y = 65520;
    //  calibData.accel_offset_z = 13;
    //
    //  calibData.gyro_offset_x = 0;
    //  calibData.gyro_offset_y = 0;
    //  calibData.gyro_offset_z = 0;
    //
    //  calibData.mag_offset_x = 710;
    //  calibData.mag_offset_y = 118;
    //  calibData.mag_offset_z = 65414;
    //
    //  calibData.accel_radius = 1000;
    //
    //  calibData.mag_radius = 636;
    //
    //  if (!bno.isFullyCalibrated())
    //  {
    //    bno.setSensorOffsets(calibData);
    //  }
  */
  if (heading < 0)
  {
    heading = 360 + heading;
  }

  if (heading > 337  or heading <= 22)
  {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(58, 15);
    display.print("N");
  }
  if (heading > 22  and heading <= 67)
  {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(52, 15);
    display.print("NE");
  }
  if (heading > 67  and heading <= 112)
  {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(58, 15);
    display.print("E");
  }
  if (heading > 112  and heading <= 157)
  {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(52, 15);
    display.print("SE");
  }
  if (heading > 157  and heading <= 202)
  {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(58, 15);
    display.print("S");
  }
  if (heading > 202  and heading <= 247)
  {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(52, 15);
    display.print("SO");
  }
  if (heading > 247  and heading <= 292)
  {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(58, 15);
    display.print("O");
  }
  if (heading > 292  and heading <= 337)
  {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(52, 15);
    display.print("NO");
  }

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(42, 35);

  display.print(heading);
  display.print((char)247);

  display.display();
  delay(100);
}

void accel()
{
  first_millis = millis();
  display.clearDisplay();
  //display.drawRect(0, 0, 128, 64, 1);
  //Button1 = select
  //Button2 = right
  //Button3 = left

  imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  float ax = linaccel.x();
  float ay = linaccel.y();
  float az = linaccel.z();

  float gx = gravity.x();
  float gy = gravity.y();
  float gz = gravity.z();
  display.println("Linear acceleration:");
  display.print("X: ");
  display.print(ax);
  display.println(" m/s^2");
  display.print("Y: ");
  display.print(ay);
  display.println(" m/s^2");
  display.print("Z: ");
  display.print(az);
  display.println(" m/s^2");
  display.println("Gravity:");
  display.print("X: ");
  display.print(gx);
  display.println(" m/s^2");
  display.print("Y: ");
  display.print(gy);
  display.println(" m/s^2");
  display.print("Z: ");
  display.print(gz);
  display.println(" m/s^2");
  display.display();
  delay(75);
}

void button_buzz()
{
  drv.setWaveform(0, 1);  // strong click 100%, see datasheet part 11.2
  drv.setWaveform(1, 0);  // end of waveforms
  drv.go();
}

void powerStats()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  // Now print out those values:
  display.print("SOC: ");
  display.print(lipo.soc());
  display.println("%");

  display.print("Voltage: ");
  display.print(lipo.voltage());
  display.println(" mV");

  display.print("Current: ");
  display.print(lipo.current(AVG));
  display.println(" mA");

  display.print("Capacity: ");
  display.print(lipo.capacity(REMAIN));
  display.print((char)47);
  display.print(lipo.capacity(FULL));
  display.println(" mAh");

  display.print("Power: ");
  display.print(lipo.power());
  display.println(" mW");

  display.print("Health: ");
  display.print(lipo.soh());
  display.print("%");
  display.display();
}

void blePeripheralConnectHandler(BLECentral& central) {
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  bleConnected = false;
}

void blePeripheralBondedHandler(BLECentral& central) {
  // central bonded event handler
  bleConnected = true;

  if (ancsNotificationSourceCharacteristic.canSubscribe()) {
    ancsNotificationSourceCharacteristic.subscribe();
  }
}

void blePeripheralRemoteServicesDiscoveredHandler(BLECentral& central) {
  // central remote services discovered event handler

  if (ancsNotificationSourceCharacteristic.canSubscribe()) {
    ancsNotificationSourceCharacteristic.subscribe();
  }
}

enum AncsNotificationEventId {
  AncsNotificationEventIdAdded    = 0,
  AncsNotificationEventIdModified = 1,
  AncsNotificationEventIdRemoved  = 2
};

enum AncsNotificationEventFlags {
  AncsNotificationEventFlagsSilent         = 1,
  AncsNotificationEventFlagsImportant      = 2,
  AncsNotificationEventFlagsPositiveAction = 4,
  AncsNotificationEventFlagsNegativeAction = 8
};

enum AncsNotificationCategoryId {
  AncsNotificationCategoryIdOther              = 0,
  AncsNotificationCategoryIdIncomingCall       = 1,
  AncsNotificationCategoryIdMissedCall         = 2,
  AncsNotificationCategoryIdVoicemail          = 3,
  AncsNotificationCategoryIdSocial             = 4,
  AncsNotificationCategoryIdSchedule           = 5,
  AncsNotificationCategoryIdEmail              = 6,
  AncsNotificationCategoryIdNews               = 7,
  AncsNotificationCategoryIdHealthAndFitness   = 8,
  AncsNotificationCategoryIdBusinessAndFinance = 9,
  AncsNotificationCategoryIdLocation           = 10,
  AncsNotificationCategoryIdEntertainment      = 11
};

struct AncsNotification {
  unsigned char eventId;
  unsigned char eventFlags;
  unsigned char catergoryId;
  unsigned char catergoryCount;
  unsigned long notificationUid;
};

void ancsNotificationSourceCharacteristicValueUpdated(BLECentral& central, BLERemoteCharacteristic& characteristic) {
  Serial.println(F("ANCS Notification Source Value Updated:"));
  struct AncsNotification notification;

  memcpy(&notification, characteristic.value(), sizeof(notification));

  Serial.print("\tEvent ID: ");
  Serial.println(notification.eventId);
  Serial.print("\tEvent Flags: 0x");
  Serial.println(notification.eventFlags, HEX);
  Serial.print("\tCategory ID: ");
  Serial.println(notification.catergoryId);
  Serial.print("\tCategory Count: ");
  Serial.println(notification.catergoryCount);
  Serial.print("\tNotification UID: ");
  Serial.println(notification.notificationUid);
}

void setup()
{
  char s_month[5];
  int tmonth, tday, tyear, thour, tminute, tsecond;
  static const char month_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";

  Serial.begin(9600); //DEBUG
  Wire.begin();
  pinMode(B1Pin, INPUT_PULLUP);
  pinMode(B2Pin, INPUT_PULLUP);
  pinMode(B3Pin, INPUT_PULLUP);

  timer = millis();

  rtc.begin();

  // __DATE__ is a C++ preprocessor string with the current date in it.
  // It will look something like 'Mar  13  2016'.
  // So we need to pull those values out and convert the month string to a number.
  sscanf(__DATE__, "%s %d %d", s_month, &tday, &tyear);

  // Similarly, __TIME__ will look something like '09:34:17' so get those numbers.
  sscanf(__TIME__, "%d:%d:%d", &thour, &tminute, &tsecond);

  // Find the position of this month's string inside month_names, do a little
  // pointer subtraction arithmetic to get the offset, and divide the
  // result by 3 since the month names are 3 chars long.
  tmonth = (strstr(month_names, s_month) - month_names) / 3;

  byte months = tmonth + 1;  // The RTC library expects months to be 1 - 12.
  byte days = tday;
  byte years = tyear - 2000; // The RTC library expects years to be from 2000.
  byte hours = thour;
  byte minutes = tminute;
  byte seconds = tsecond + 6; //+6 to compensate for prog. time.

  rtc.setTime(hours, minutes, seconds);
  rtc.setDate(days, months, years);

  rtc.setTime(hours, minutes, seconds);
  rtc.setDate(days, months, years);

  bmp.begin();

  bno.begin();
  bno.setExtCrystalUse(true);

  //BATT SETUP
  lipo.begin();
  lipo.setCapacity(BATTERY_CAPACITY);

  drv.begin();
  drv.setMode(DRV2605_MODE_INTTRIG);
  drv.selectLibrary(1);

  bleBondStore.clearData();

  blePeripheral.setBondStore(bleBondStore);

  blePeripheral.setServiceSolicitationUuid(ancsService.uuid());
  blePeripheral.setLocalName("ANCS");

  // set device name and appearance
  blePeripheral.setDeviceName("RWatch");
  blePeripheral.setAppearance(0x0080);

  blePeripheral.addRemoteAttribute(ancsService);
  blePeripheral.addRemoteAttribute(ancsNotificationSourceCharacteristic);

  // assign event handlers for connected, disconnected to peripheral
  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  blePeripheral.setEventHandler(BLEBonded, blePeripheralBondedHandler);
  blePeripheral.setEventHandler(BLERemoteServicesDiscovered, blePeripheralRemoteServicesDiscoveredHandler);

  // assign event handlers for characteristic
  ancsNotificationSourceCharacteristic.setEventHandler(BLEValueUpdated, ancsNotificationSourceCharacteristicValueUpdated);

  // begin initialization
  blePeripheral.begin();

  display.begin(SSD1306_SWITCHCAPVCC);
  display.clearDisplay();
  display.drawBitmap(0, 0, RWatch_logo, 128, 64, 1);
  display.display();
  delay(1000);
  display.clearDisplay();
}

void loop()
{
  if (rtc.getSeconds() % 2 == 0)
  {
    blePeripheral.poll();
  }
  shouldBeSleeping = false;
  Button1.read();
  Button2.read();
  Button3.read();

  if (Button1.pressedFor(1500) or timer - first_millis >= timeout)
  {
    shouldBeSleeping = true;
  }

  switch (shouldBeSleeping)
  {
    case 0:
      break;
    case 1:
      display.clearDisplay();
      display.display();
      //attachInterrupt(A2, interr1, HIGH);
      attachInterrupt(B1Pin, interr2, LOW);
      LowPower.standby();
      display.drawBitmap(0, 0, RWatch_logo, 128, 64, 1);
      display.display();
      detachInterrupt(B1Pin);
      //detachInterrupt(A2);
      first_millis = millis();
      inMenu = 0;
      mode = 0;
      modeInMenu = 0;
      shouldBeSleeping = false;
  }

  switch (inMenu)
  {
    case 0:
      show_mode();
      break;
    case 1:
      menu();
      break;
  }

  if (Button1.wasPressed())
  {
    inMenu++;
    button_buzz();
    if (inMenu > 1)
    {
      inMenu = 0;
    }
  }
}
