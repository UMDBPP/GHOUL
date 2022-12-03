//GHOUL flight software written by:
//Michael Kalin with love and support from Kruti Geeta-Rajnikant and Jeremy Joseph (JJ) Kuznetsov

#include <RTClib.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <TimeLib.h>

//sensor declarations
#define SERVO_PIN 30
#define FEEDBACK_PIN A13
#define CUTDOWN_PIN 6
#define LOGGER_PIN BUILTIN_SDCARD

//float parameters
#define PRE_VENT_ALT 24000
#define FLOAT_ALT 40000
#define PRE_VENT_RATIO .5
#define VENT_TIMER 900
#define SEA_LEVEL_PRESSURE 1024.00

//cutdown parameters
#define CUT_INTERVAL 60
#define TOTAL_CUTS 4
#define CUTDOWN_ALTITUDE 27000
#define CUTDOWN_TIMER_TRIGGER_ALT 1000
#define CUTDOWN_TIMER_DURATION 4900
#define ARATE_TRIGGER_ALT 2000
#define ASCENT_RATE_TRIGGER 1

//flags
#define VENT_OPEN_POS 0
#define VENT_CLOSED_POS 180
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
#define NOT_CUT 0
#define CUT 1
#define TIMER_NOT_STARTED 0
#define TIMER_STARTED 1
#define ARATE_TRIGGER_NOT_STARTED 0
#define ARATE_TRIGGER_STARTED 1

//sensors
Adafruit_BMP280 bmp;
Servo ventValve;
File logFile;

//storage
float prev_alt = 0;
uint32_t prev_time = 1;
float prev_ascent_rate[5] = {0, 0, 0, 0, 0};
float rate_at_open = 999;
uint32_t vent_open_time = 0;
int num_cuts = 0;
uint32_t next_cut_time = UINT_MAX;
uint32_t cutdown_time = UINT_MAX;

//flags
int pre_vent_status = PRE_VENT_NOT_DONE; //0 = not done, 1 = done
int float_vent_status = FLOAT_VENT_NOT_DONE; //0 = not done, 1 = done
int vent_status = CLOSED; //0 = closed, 1 = open
int float_status = NORMAL_ASCENT; //0 = normal ascent, 1 = pre-venting, 2 = float-venting, 3 = floating
int cut_status = NOT_CUT; //0 = not cut, 1 = cut
int timer_status = TIMER_NOT_STARTED; //0 = timer not started, 1 = timer started
int arate_trigger_status = ARATE_TRIGGER_NOT_STARTED; //0 = arate trigger not started, 1 = arate trigger started

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  ventValve.attach(SERVO_PIN);
  delay(50);
  ventValve.write(VENT_CLOSED_POS);

  pinMode(CUTDOWN_PIN, OUTPUT);
  digitalWrite(CUTDOWN_PIN, LOW);

  pinMode(FEEDBACK_PIN, INPUT);
  
  setSyncProvider(getTeensy3Time);
  if(timeStatus() != timeSet)
  {
    Serial.println("Unable to sync with the RTC");
  }
  else
  {
    Serial.println("RTC has set the system time");
  }
    
  if(!bmp.begin())
    Serial.println("Error BMP not found!");
  else
    Serial.println("BMP found!");
  
  if(!SD.begin(LOGGER_PIN))
    Serial.println("Error: SD Card Logger Not Initialized");
  else
    Serial.println("SD card initialized");
}

void loop() {  
  //get time
  DateTime curr_time = DateTime(now());
  uint32_t now_seconds = now();
  
  //get pressure, temp, alt
  float temp = bmp.readTemperature();
  float pressure = bmp.readPressure();
  float alt = bmp.readAltitude(SEA_LEVEL_PRESSURE);
  
  //calc ascent rate
  float curr_ascent_rate = (alt - prev_alt)/(now_seconds - prev_time);
  float ascent_rate = (prev_ascent_rate[0] + prev_ascent_rate[1] + prev_ascent_rate[2] + prev_ascent_rate[3] + prev_ascent_rate[4] + curr_ascent_rate)/6;

  float raw_servo_pos = analogRead(FEEDBACK_PIN);
  float servo_pos = (raw_servo_pos/1024)*180;

  //if vent is closed, see if we should open it
  if(vent_status == CLOSED)
  {
    if(pre_vent_status == PRE_VENT_NOT_DONE && alt > PRE_VENT_ALT && alt < FLOAT_ALT)
    {
      ventValve.write(VENT_OPEN_POS);
      vent_status = OPEN;
      float_status = PRE_VENTING;
      rate_at_open = ascent_rate;
      vent_open_time = now_seconds;
    }
    else if(float_vent_status == FLOAT_VENT_NOT_DONE && alt > FLOAT_ALT)
    {
      ventValve.write(VENT_OPEN_POS);
      vent_status = OPEN;
      float_status = FLOAT_VENTING;
      rate_at_open = ascent_rate;
    }
  }

  //if vent is open, check whether we are pre-venting or float venting, then close vent if target ascent rate is reached
  if(vent_status == OPEN)
  {
    if(float_status == PRE_VENTING)
    {
      if(ascent_rate <= PRE_VENT_RATIO*rate_at_open)
      {
        ventValve.write(VENT_CLOSED_POS);
        vent_status = CLOSED;
        float_status = PRE_VENTED;
        pre_vent_status = PRE_VENT_DONE;
      }
    }
    else if(float_status == FLOAT_VENTING)
    {
      if(ascent_rate < 1)
       {
        ventValve.write(VENT_CLOSED_POS);
        vent_status = CLOSED;
        float_status = FLOATING;
        float_vent_status = FLOAT_VENT_DONE;
       }
    }
    else
    {
      ventValve.write(VENT_CLOSED_POS);
      vent_status = CLOSED;
    }
  }

  if(vent_open_time != 0)
  {
    if(now_seconds - vent_open_time > VENT_TIMER)
    {
      ventValve.write(VENT_CLOSED_POS);
      vent_status = CLOSED;
      if(float_status == PRE_VENTING && pre_vent_status == PRE_VENT_NOT_DONE)
      {
        pre_vent_status = PRE_VENT_DONE;
        float_status = PRE_VENTED;
      }
      else if(float_status == FLOAT_VENTING && float_vent_status == FLOAT_VENT_NOT_DONE)
      {
        float_vent_status = FLOAT_VENT_DONE;
        float_status = FLOATING;
      }
    }
  }

  //altitude trigger
  if(alt >= CUTDOWN_ALTITUDE && cut_status == NOT_CUT)
  {
    cutdown();
    cut_status = CUT;
    num_cuts++;
    next_cut_time = now_seconds + CUT_INTERVAL;
  }

  //ascent rate trigger
  if(arate_trigger_status == ARATE_TRIGGER_NOT_STARTED && alt >= ARATE_TRIGGER_ALT)
  {
    arate_trigger_status = ARATE_TRIGGER_STARTED;
  }
  if(arate_trigger_status == ARATE_TRIGGER_STARTED && ascent_rate < ASCENT_RATE_TRIGGER && cut_status == NOT_CUT)
  {
    cutdown();
    cut_status = CUT;
    num_cuts++;
    next_cut_time = now_seconds + CUT_INTERVAL;
  }
    
  //timer trigger
  if(timer_status == TIMER_NOT_STARTED && alt >= CUTDOWN_TIMER_TRIGGER_ALT)
  {
    cutdown_time = now_seconds + CUTDOWN_TIMER_DURATION;
    timer_status = TIMER_STARTED;
  }
  if(now_seconds >= cutdown_time && cut_status == NOT_CUT)
  {
    cutdown();
    cut_status = CUT;
    num_cuts++;
    next_cut_time = now_seconds + CUT_INTERVAL;
  }

  //subsequent cuts
  if(cut_status == CUT && num_cuts < TOTAL_CUTS)
  {
    if(now_seconds >= next_cut_time)
    {
      cutdown();
      num_cuts++;
      next_cut_time = now_seconds + CUT_INTERVAL;
    }
  }
  
  //write to the data file
  logFile = SD.open("datalog.txt", FILE_WRITE);
  if(curr_time.hour() < 10)
    logFile.print("0");
  logFile.print(curr_time.hour());
  logFile.print(":");
  if(curr_time.minute() < 10)
    logFile.print("0");
  logFile.print(curr_time.minute());
  logFile.print(":");
  if(curr_time.second() < 10)
    logFile.print("0");
  logFile.print(curr_time.second());
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
  logFile.print(raw_servo_pos);
  logFile.print(", ");
  logFile.print(servo_pos);
  logFile.print(", ");
  logFile.print(num_cuts);
  logFile.print(", ");
  logFile.print(next_cut_time);
  logFile.print(", ");
  logFile.print(cutdown_time);
  logFile.print(", ");
  logFile.print(vent_status);
  logFile.print(", ");
  logFile.print(float_status);
  logFile.print(", ");
  logFile.print(pre_vent_status);
  logFile.print(", ");
  logFile.print(float_vent_status);
  logFile.print(", ");
  logFile.print(cut_status);
  logFile.print(", ");
  logFile.print(timer_status);
  logFile.print(", ");
  logFile.print(arate_trigger_status);
  logFile.println();
  logFile.close();

  //write to serial
  if(curr_time.hour() < 10)
    Serial.print("0");
  Serial.print(curr_time.hour());
  Serial.print(":");
  if(curr_time.minute() < 10)
    Serial.print("0");
  Serial.print(curr_time.minute());
  Serial.print(":");
  if(curr_time.second() < 10)
    Serial.print("0");
  Serial.print(curr_time.second());
  Serial.print(", ");
  Serial.print(now_seconds);
  Serial.print(", ");
  Serial.print(temp);
  Serial.print(", ");
  Serial.print(pressure);
  Serial.print(", ");
  Serial.print(alt);
  Serial.print(", ");
  Serial.print(curr_ascent_rate);
  Serial.print(", ");
  Serial.print(ascent_rate);
  Serial.print(", ");
  Serial.print(raw_servo_pos);
  Serial.print(", ");
  Serial.print(servo_pos);
  Serial.print(", ");
  Serial.print(num_cuts);
  Serial.print(", ");
  Serial.print(next_cut_time);
  Serial.print(", ");
  Serial.print(cutdown_time);
  Serial.print(", ");
  Serial.print(vent_status);
  Serial.print(", ");
  Serial.print(float_status);
  Serial.print(", ");
  Serial.print(pre_vent_status);
  Serial.print(", ");
  Serial.print(float_vent_status);
  Serial.print(", ");
  Serial.print(cut_status);
  Serial.print(", ");
  Serial.print(timer_status);
  Serial.print(", ");
  Serial.print(arate_trigger_status);
  Serial.println();

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

void cutdown()
{
  digitalWrite(CUTDOWN_PIN, HIGH);
  delay(8000);
  digitalWrite(CUTDOWN_PIN, LOW);
}

void yolo_cutdown()
{
  delay(120000); 
  digitalWrite(CUTDOWN_PIN, HIGH);
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}
