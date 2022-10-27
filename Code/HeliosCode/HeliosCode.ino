#include "RTClib.h"
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>

#define BATT_PIN A7
#define SERVO_PIN 10
#define LOGGER_PIN 4

#define PRE_VENT_ALT 15500
#define FLOAT_ALT 25000
#define PRE_VENT_RATIO .6
#define VENT_TIMER 900
#define SEA_LEVEL_PRESSURE 996.00

#define VENT_OPEN 0
#define VENT_CLOSED 165
#define CLOSED 0
#define OPEN 1
#define PRE_VENT_NOT_DONE 0
#define PRE_VENT_DONE 1
#define FLOAT_VENT_NOT_DONE 0
#define FLOAT_VENT_DONE 1
#define NORMAL_ASCENT 0
#define PRE_VENTING 1
#define PRE_VENTED 2
#define FLOAT_VENTING 3
#define FLOATING 4

Adafruit_BMP280 bmp;
RTC_DS1307 rtc;

//storage
float prev_alt = 0;
uint32_t prev_time = 1;
float prev_ascent_rate[5] = {0, 0, 0, 0, 0};
float rate_at_open = 999;
uint32_t vent_open_time = 0;

//flags
int pre_vent_status = 0; //0 = not done, 1 = done
int float_vent_status = 0; //0 = not done, 1 = done
int vent_status = 0; //0 = closed, 1 = open
int float_status = 0; //0 = normal ascent, 1 = pre-venting, 2 = float-venting, 3 = floating

Servo ventValve;
File logFile;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  ventValve.attach(SERVO_PIN);
  delay(50);
  ventValve.write(VENT_CLOSED);

  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  if(!bmp.begin())
    Serial.println("Error BMP not found!");
  
  if(!SD.begin(LOGGER_PIN))
    Serial.println("Error: SD Card Logger Not Initialized");
}

void loop() {  
  //get time
  DateTime now = rtc.now();
  uint32_t now_seconds = now.secondstime();
  
  //get pressure, temp, alt
  float temp = bmp.readTemperature();
  float pressure = bmp.readPressure();
  float alt = bmp.readAltitude(SEA_LEVEL_PRESSURE);

  //measure battery voltage
  float vbatt = analogRead(BATT_PIN);
  vbatt *= 2;    // we divided by 2, so multiply back
  vbatt *= 3.3;  // Multiply by 3.3V, our reference voltage
  vbatt /= 1024; // convert to voltage
  
  //calc ascent rate
  float curr_ascent_rate = (alt - prev_alt)/(now_seconds - prev_time);
  float ascent_rate = (prev_ascent_rate[0] + prev_ascent_rate[1] + prev_ascent_rate[2] + prev_ascent_rate[3] + prev_ascent_rate[4] + curr_ascent_rate)/6;

  //if vent is closed, see if we should open it
  if(vent_status == CLOSED)
  {
    if(pre_vent_status == PRE_VENT_NOT_DONE && alt > PRE_VENT_ALT && alt < FLOAT_ALT)
    {
      ventValve.write(VENT_OPEN);
      vent_status = OPEN;
      float_status = PRE_VENTING;
      rate_at_open = ascent_rate;
      vent_open_time = now_seconds;
    }
    else if(float_vent_status = FLOAT_VENT_NOT_DONE && alt > FLOAT_ALT)
    {
      ventValve.write(VENT_OPEN);
      vent_status = OPEN;
      float_status = FLOAT_VENTING;
      rate_at_open = ascent_rate;
    }
  }

  if(vent_status == OPEN)
  {
    if(float_status == PRE_VENTING)
    {
      if(ascent_rate <= PRE_VENT_RATIO*rate_at_open)
      {
        ventValve.write(VENT_CLOSED);
        vent_status = CLOSED;
        float_status = PRE_VENTED;
        pre_vent_status = PRE_VENT_DONE;
      }
    }
    else if(float_status == FLOAT_VENTING)
    {
      if(ascent_rate < 1)
       {
        ventValve.write(VENT_CLOSED);
        vent_status = CLOSED;
        float_status = FLOATING;
        float_vent_status = FLOAT_VENT_DONE;
       }
    }
    else
    {
      ventValve.write(VENT_CLOSED);
      vent_status = CLOSED;
    }
  }

  if(vent_open_time != 0)
  {
    if(now_seconds - vent_open_time > VENT_TIMER)
    {
      ventValve.write(VENT_CLOSED);
      vent_status = CLOSED;
      if(float_status == PRE_VENTING && pre_vent_status == PRE_VENT_NOT_DONE)
      {
        pre_vent_status = PRE_VENT_DONE;
        float_status = PRE_VENTED;
      }
      else if(float_status = FLOAT_VENTING && float_vent_status == FLOAT_VENT_NOT_DONE)
      {
        float_vent_status = FLOAT_VENT_DONE;
        float_status = FLOATING;
      }
    }
  }
  
  //write to the data file
  logFile = SD.open("datalog.txt", FILE_WRITE);
  if(now.hour() < 10)
    logFile.print("0");
  logFile.print(now.hour());
  logFile.print(":");
  if(now.minute() < 10)
    logFile.print("0");
  logFile.print(now.minute());
  logFile.print(":");
  if(now.second() < 10)
    logFile.print("0");
  logFile.print(now.second());
  logFile.print(", ");
  logFile.print(now_seconds);
  logFile.print(", ");
  logFile.print(temp);
  logFile.print(", ");
  logFile.print(pressure);
  logFile.print(", ");
  logFile.print(alt);
  logFile.print(", ");
  logFile.print(curr_ascent_rate);
  logFile.print(", ");
  logFile.print(ascent_rate);
  logFile.print(", ");
  logFile.print(vbatt);
  logFile.print(", ");
  logFile.print(vent_status);
  logFile.print(", ");
  logFile.print(float_status);
  logFile.print(", ");
  logFile.print(pre_vent_status);
  logFile.print(", ");
  logFile.println(float_vent_status);
  logFile.close();

  //clean up
  for(int i = 0; i < 4; i++)
  {
    prev_ascent_rate[i] = prev_ascent_rate[i+1];
  }
  prev_alt = alt;
  prev_time = now_seconds;
  prev_ascent_rate[4] = ascent_rate;

  delay(1000);
}
