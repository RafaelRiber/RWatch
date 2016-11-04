#include <Wire.h>
#include <LSM303.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RTClib.h>
#include <Chrono.h>
#include <LowPower.h>
#include <Button.h>
#include <StopWatch.h>

#define OLED_MOSI   10 //FINAL: ??, PROTO: 10
#define OLED_CLK   11  //FINAL: ??, PROTO: 11
#define OLED_DC    12  //FINAL: ??, PROTO: 12
#define OLED_CS    A0  //FINAL: ??, PROTO: A0
#define OLED_RESET A1  //FINAL: ??, PROTO: A1

#define B1Pin       5  //FINAL: D12, PROTO: 9
#define B2Pin       6 //FINAL: D10, PROTO: 10
#define B3Pin       9 //FINAL: D9, PROTO: 11

#define ClickPin    7 //FINAL: D7 //INT2 Double click

LSM303 lsm;

StopWatch stopwatch;


Chrono myChrono(Chrono::SECONDS);

Button Button1(B1Pin, false, true, 20);  //Select
Button Button2(B2Pin, false, true, 20); //Right
Button Button3(B3Pin, false, true, 20); //Left
Button Click(ClickPin, false, true, 0); //Double Click

Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

RTC_DS3231 rtc;

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

int set_mode = 0;
int minhour = 0;
int time_set_hour;
int time_set_min;

int timeout = 10000;
int first_millis;

//int accelSens = 1; //1, 2, 4 or 12
int accelSens = 2; //+-4G

//FR:
//char daysOfTheWeek[7][12] = {"DIM", "LUN", "MAR", "MER", "JEU", "VEN", "SAM"};
//ENG:
char daysOfTheWeek[7][12] = {"SUN", "MON", "TUE", "WEN", "THU", "FRI", "SAT"};


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
      display.println(" Main screen");
      display.setTextColor(WHITE);
      display.println(" Stopwatch");
      display.println(" Compass");
      display.println(" Accelerometer");
      display.display();
      if (Button1.wasPressed())
      {
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
      display.println(" Stopwatch");
      display.setTextColor(WHITE);
      display.println(" Compass");
      display.println(" Accelerometer");
      display.display();
      if (Button1.wasPressed())
      {
        mode = 1;
      }
      break;
    case 2: //Compass
      display.clearDisplay();
      display.drawRect(0, 0, 128, 64, 1);
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(2, 2);
      display.println(" Main screen");
      display.println(" Stopwatch");
      display.setTextColor(BLACK, WHITE);
      display.println(" Compass");
      display.setTextColor(WHITE);
      display.println(" Accelerometer");
      display.display();
      if (Button1.wasPressed())
      {
        mode = 2;
      }
      break;
    case 3: //Accelprint
      display.clearDisplay();
      display.drawRect(0, 0, 128, 64, 1);
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(2, 2);
      display.println(" Main screen");
      display.println(" Stopwatch");
      display.println(" Compass");
      display.setTextColor(BLACK, WHITE);
      display.println(" Accelerometer");
      display.setTextColor(WHITE);
      display.display();
      if (Button1.wasPressed())
      {
        mode = 3;
      }
      break;
  }

  if (Button2.wasPressed())
  {
    modeInMenu++;
  }

  if (Button3.wasPressed())
  {
    modeInMenu = modeInMenu - 1;
  }

  if (modeInMenu > 3)
  {
    modeInMenu = 3;
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
      compass();
      break;
    case 3:
      accel();
      break;
  }
}

void main_screen()
{
  DateTime now = rtc.now();

  display.clearDisplay();

  display.drawRect(0, 0, 128, 64, 1);

  display.drawFastHLine(0, 14, 128, 1);
  //display.drawFastHLine(0, 48, 128, 1);

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(24, 4);
  display.print(daysOfTheWeek[now.dayOfTheWeek()]);
  display.print(" ");

  if (now.day() < 10)
  {
    display.print("0");
    display.print(now.day());
  }
  if (now.day() >= 10)
  {
    display.print(now.day());
  }
  display.print('/');
  if (now.month() < 10)
  {
    display.print("0");
    display.print(now.month());
  }
  if (now.month() >= 10)
  {
    display.print(now.month());
  }
  display.print('/');
  display.print(now.year());

  display.setTextSize(2);
  display.setTextColor(WHITE);

  //WITH LINE:
  //display.setCursor(15, 25);

  //WITHOUT LINE:
  display.setCursor(15, 30);

  if (now.hour() < 10)
  {
    display.print('0');
    display.print(now.hour());
  }
  else
  {
    display.print(now.hour());
  }
  display.print(':');
  if (now.minute() < 10)
  {
    display.print('0');
    display.print(now.minute());
  }
  else
  {
    display.print(now.minute());
  }
  display.print(':');
  if (now.second() < 10)
  {
    display.print('0');
    display.print(now.second());
  }
  else
  {
    display.print(now.second());
  }

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(100, 52);

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
    stopwatch.reset();
    chronoCounting = 0;
  }

  display.display();
}


void compass()
{
  first_millis = millis();
  display.clearDisplay();
  display.drawRect(0, 0, 128, 64, 1);
  //Button1 = select
  //Button2 = right
  //Button3 = left

  lsm.read();

  int heading = lsm.heading((LSM303::vector<int>) {
    1, 0, 0
  });



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
  delay(150);
}

void accel()
{
  first_millis = millis();
  display.clearDisplay();
  //display.drawRect(0, 0, 128, 64, 1);
  //Button1 = select
  //Button2 = right
  //Button3 = left

  lsm.readAcc();
  float ax = ((lsm.a.x / 16) * accelSens * 0.01 * -1);
  float ay = ((lsm.a.y / 16) * accelSens * 0.01 * -1);
  float az = ((lsm.a.z / 16) * accelSens * 0.01 * -1);

  float gx = ((lsm.a.x / 16) * accelSens * 0.01 * -1 / 10);
  float gy = ((lsm.a.y / 16) * accelSens * 0.01 * -1 / 10);
  float gz = ((lsm.a.z / 16) * accelSens * 0.01 * -1 / 10);

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  display.print("X: ");
  display.print(ax);
  display.println(" m/s^2");
  display.print("Y: ");
  display.print(ay);
  display.println(" m/s^2");
  display.print("Z: ");
  display.print(az);
  display.println(" m/s^2");
  display.println("");
  display.println("");
  display.print("X: ");
  display.print(gx);
  display.println(" G");
  display.print("Y: ");
  display.print(gy);
  display.println(" G");
  display.print("Z: ");
  display.print(gz);
  display.println(" G");
  display.display();
  delay(75);
}

void setup()
{
  Serial.begin(9600); //DEBUG
  Wire.begin();
  pinMode(B1Pin, INPUT_PULLUP);
  pinMode(B2Pin, INPUT_PULLUP);
  pinMode(B3Pin, INPUT_PULLUP);
  pinMode(ClickPin, INPUT);

  timer = millis();

  rtc.begin();


  if (rtc.lostPower())
  {
    // The following line sets the RTC to the date & time this sketch was compiled
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time:
    //example: rtc.adjust(DateTime(YYYY, M/MM, D/DD, H/HH, M/MM, S/SS));
    //rtc.adjust(DateTime(2016, 10, 17, 16, 15, 0));
  }

  lsm.init();
  lsm.enableDefault();
  lsm.writeAccReg(LSM303::CTRL_REG4_A, 24); //+- 4G
  //INT1 Wake Interrupt
  lsm.writeAccReg(LSM303::CTRL_REG2_A, 0x01); //High-pass filter enabled for AOI function on Interrupt 1
  lsm.writeAccReg(LSM303::CTRL_REG3_A, 0x40); //Enable Interrupt
  lsm.writeAccReg(LSM303::CTRL_REG6_A, 0x02); //Interrupt active low and Interrupt 1 on PAD2.
  lsm.writeAccReg(LSM303::INT1_CFG_A, 0xA1); //AND combination of interrupt events.
  lsm.writeAccReg(LSM303::INT1_THS_A, 0x0F); //Interrupt 1 threshold.
  lsm.writeAccReg(LSM303::INT1_DURATION_A, 0x01); //Duration value.

  //INT1 + Click on INT2
  //  lsm.writeAccReg(LSM303::CTRL_REG2_A, 0x05); //High-pass filter enabled for AOI function on Interrupt 1 and Click
  //  lsm.writeAccReg(LSM303::CTRL_REG3_A, 0xC0); //Enable Interrupt1 and Click interrupt
  //  lsm.writeAccReg(LSM303::CTRL_REG6_A, 0x82); //Interrupt active low and click on PAD (PIN) 2
  //  //INT1 CFG
  //  lsm.writeAccReg(LSM303::INT1_CFG_A, 0xD4); //AND/OR combination of interrupt events. 6-direction detection function enabled.
  //  lsm.writeAccReg(LSM303::INT1_THS_A, 0x0F); //Interrupt 1 threshold.
  //  lsm.writeAccReg(LSM303::INT1_DURATION_A, 0x01); //Duration value.
  //  //CLICK CFG
  //  lsm.writeAccReg(LSM303::CLICK_CFG_A, 0x30); //Double CLICK interrupt on Z axis
  //  lsm.writeAccReg(LSM303::CLICK_THS_A, 0x0F); // Threshold
  //  lsm.writeAccReg(LSM303::TIME_LIMIT_A, 0x46); //Time limit setting
  //  lsm.writeAccReg(LSM303::TIME_LATENCY_A, 0x19); //Latency Setting
  //  lsm.writeAccReg(LSM303::TIME_WINDOW_A, 0x1E); //Window

  //PROTO: min: {  -621,    -148,   -206}    max: {  +283,   +729,   +892}

  //ASSEMBLY REV.A : min: {  -505,   -631,   -308}    max: {  +677,   +568,   +902}

  lsm.m_min = (LSM303::vector<int16_t>) {
    -505, -631, -308
  };
  lsm.m_max = (LSM303::vector<int16_t>) {
    +677, +568, +994
  };

  display.begin(SSD1306_SWITCHCAPVCC);
  display.clearDisplay();
  display.drawBitmap(0, 0, RWatch_logo, 128, 64, 1);
  display.display();
  delay(1000);
  display.clearDisplay();
}

void loop()
{
  shouldBeSleeping = false;
  Button1.read();
  Button2.read();
  Button3.read();
  //Click.read();

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
      delay(250);
      //attachInterrupt(6, interr1, LOW);
      attachInterrupt(B1Pin, interr2, LOW);
      LowPower.standby();
      display.drawBitmap(0, 0, RWatch_logo, 128, 64, 1);
      display.display();
      detachInterrupt(B1Pin);
      //detachInterrupt(6);
      inMenu = 0;
      modeInMenu = 0;
      mode = 0;
      first_millis = millis();
      delay(500);
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
    if (inMenu > 1)
    {
      inMenu = 0;
    }
  }
}
