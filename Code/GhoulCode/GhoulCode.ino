//GHOUL flight software written by:
//Michael Kalin and Jeremy Joseph (JJ) Kuznetsov with love and support from Kruti Geeta-Rajnikant and Daniel Grammar and Akemi Takeuchi

#include <RTClib.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <TimeLib.h>
#include <Adafruit_GPS.h>
#include <TinyGPS++.h>

//pin definitions
#define SERVO_PIN 2
#define FEEDBACK_PIN A9
#define CUTDOWN_PIN_1 35
#define CUTDOWN_PIN_2 36
#define LOGGER_PIN BUILTIN_SDCARD
#define GPSSerial Serial2
#define XbeeSerial Serial1

//float parameters
#define PRE_VENT_ALT 24000
#define FLOAT_ALT 40000
#define PRE_VENT_RATIO .36
#define VENT_TIMER 600
#define SEA_LEVEL_PRESSURE 1020.65

//cut-down parameters
#define CUT_INTERVAL 60
#define TOTAL_CUTS 1
#define CUTDOWN_ALTITUDE 27000
#define CUTDOWN_TIMER_TRIGGER_ALT 1000
#define CUTDOWN_TIMER_DURATION 6000
#define ARATE_TRIGGER_ALT 40000
#define ASCENT_RATE_TRIGGER 1
#define LONG_EAST_BOUND -76.937243 
#define LONG_WEST_BOUND -76.940906
#define LAT_NORTH_BOUND 38.994138
#define LAT_SOUTH_BOUND 38.991425

//flags
#define VENT_OPEN_POS 117
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
#define BAD_FIX 2
#define TIMER_NOT_STARTED           0
#define TIMER_STARTED               1
#define ARATE_TRIGGER_NOT_STARTED   0
#define ARATE_TRIGGER_STARTED       1
#define CUT_REASON_TIMER            1
#define CUT_REASON_ALTITUDE         2
#define CUT_REASON_ASCENT_RATE      3
#define CUT_REASON_GEOFENCE         4

//timer
IntervalTimer gpsTimer;

//sensors
Adafruit_BMP280 bmp;
Servo ventValve;
File logFile;
Adafruit_GPS GPS(&GPSSerial);
//TinyGPSPlus gps;

//storage
float prev_alt = 0;
uint32_t prev_time = 1;
float prev_ascent_rate[5] = {0, 0, 0, 0, 0};
float rate_at_open = 999;
uint32_t vent_open_time = 0;
int num_cuts = 0;
uint32_t next_cut_time = UINT_MAX;
uint32_t cutdown_time = UINT_MAX;
float gps_lat;
float gps_long;
float gps_alt;
int gps_sats;
int antenna_status;
int gps_fixqual;
int gps_fault_counter;

//flags
int pre_vent_status = PRE_VENT_NOT_DONE;              //0 = not done, 1 = done
int float_vent_status = FLOAT_VENT_NOT_DONE;          //0 = not done, 1 = done
int vent_status = CLOSED;                             //0 = closed, 1 = open
int float_status = NORMAL_ASCENT;                     //0 = normal ascent, 1 = pre-venting, 2 = float-venting, 3 = floating
int cut_status = NOT_CUT;                             //0 = not cut, 1 = cut
int timer_status = TIMER_NOT_STARTED;                 //0 = timer not started, 1 = timer started
int arate_trigger_status = ARATE_TRIGGER_NOT_STARTED; //0 = arate trigger not started, 1 = arate trigger started
int cut_reason = NOT_CUT;                             //0 = not cut, 1 = timer, 2 = altitude, 3 = ascent rate, 4 = geofence

void setup() {
  Serial.begin(9600);

  // Initiate/close servo
  ventValve.attach(SERVO_PIN);
  delay(50);
  ventValve.write(VENT_CLOSED_POS);

  // Servo Analog Feedback Pin
  pinMode(FEEDBACK_PIN, INPUT);

  // Initiate cut-down pins
  pinMode(CUTDOWN_PIN_1, OUTPUT);
  digitalWrite(CUTDOWN_PIN_1, LOW);

  // Initiate RTC
  setSyncProvider(getTeensy3Time);
  if(timeStatus() != timeSet)
  {
    Serial.println("Unable to sync with the RTC");
  }
  else
  {
    Serial.println("RTC has set the system time");
  }

  // Initiate BMP280
  if(!bmp.begin())
    Serial.println("Error BMP not found!");
  else
    Serial.println("BMP found!");

  // Initiate GPS
  GPSSerial.begin(9600);

  // Check/Initiate SD Logger
  if(!SD.begin(LOGGER_PIN))
    Serial.println("Error: SD Card Logger Not Initialized");
  else
    Serial.println("SD card initialized");

  // Interrupt Timer for GPS
  gpsTimer.begin(readGPS, 1000);
}

void loop() {  


   /*  ============================================================================================
   *   
   *                              Data Collection and Storage
   *   
       ============================================================================================ */

       
       
  //get time ------------------------------------------------------------------------------------ time
  DateTime curr_time = DateTime(now());
  uint32_t now_seconds = now();
  
  //get pressure, temp, alt --------------------------------------------------------------------- pressure, temp, alt
  float temp = bmp.readTemperature();
  float pressure = bmp.readPressure();
  float alt = bmp.readAltitude(SEA_LEVEL_PRESSURE);

  //read gps data -------------------------------------------------------------------------------- gps (long, lat, alt, fix, sats #)
  noInterrupts();
  if(GPS.newNMEAreceived())
  {
    GPS.parse(GPS.lastNMEA());
    gps_fixqual = GPS.fix;
    if(gps_fixqual == 1)
    {
      gps_lat = GPS.latitudeDegrees;
      gps_long = GPS.longitudeDegrees;
      gps_alt = GPS.altitude;
      gps_sats = GPS.satellites;
      Serial.println(GPS.latitudeDegrees, 6);
      Serial.println(GPS.longitudeDegrees, 6);
      Serial.println(GPS.altitude);
      Serial.println(GPS.satellites);
    }   
  }
  else
    Serial.println("Nada :(");
  interrupts();
  

  /*  ============================================================================================
   *   
   *                              Calculations and Decisions
   *   
       ============================================================================================ */

       
  
  //calc ascent rate ------------------------------------------------------------------------------ Ascent Rate Calculation
  float curr_ascent_rate = (alt - prev_alt)/(now_seconds - prev_time);
  float ascent_rate = (prev_ascent_rate[0] + prev_ascent_rate[1] + prev_ascent_rate[2] + prev_ascent_rate[3] + prev_ascent_rate[4] + curr_ascent_rate)/6;

  float raw_servo_pos = analogRead(FEEDBACK_PIN);
  float servo_pos = (raw_servo_pos/1024)*180;

  //if vent is closed, see if we should open it --------------------------------------------------- Should we open vent?
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

  //if vent is open, check whether we are pre-venting or float venting, 
  // then close vent if target ascent rate is reached -------------------------------------------- Should we close vent?
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
  //  =============================================================================================
  //  
  //                                          CUT DOWN TRIGGERS
  //
  //  =============================================================================================


  
  //altitude trigger ------------------------------------------------------------------------------ Altitude Trigger
  if(alt >= CUTDOWN_ALTITUDE && cut_status == NOT_CUT)
  {
    cutdown();
    cut_status = CUT;
    num_cuts++;
    next_cut_time = now_seconds + CUT_INTERVAL;
    cut_reason = CUT_REASON_ALTITUDE;
  }

  //ascent rate trigger --------------------------------------------------------------------------- Ascent Rate Trigger
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
    cut_reason = CUT_REASON_ASCENT_RATE;
  }
    
  //timer trigger --------------------------------------------------------------------------------- Timer Trigger
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
    cut_reason = CUT_REASON_TIMER;
  }

  //GPS Trigger ---------------------------------------------------------------------------------- GPS Geofence Trigger
  if(cut_status == NOT_CUT && geofence_check(gps_long, gps_lat, gps_fixqual) == CUT)
  {
    cutdown();
    cut_status = CUT;
    num_cuts++;
    next_cut_time = now_seconds + CUT_INTERVAL;
    cut_reason = CUT_REASON_GEOFENCE;
  }

  //Additional Cuts After Trigger Activation ----------------------------------------------------- Additional Cuts (for redundancy)
  if(cut_status == CUT && num_cuts < TOTAL_CUTS)
  {
    if(now_seconds >= next_cut_time)
    {
      cutdown();
      num_cuts++;
      next_cut_time = now_seconds + CUT_INTERVAL;
    }
  }
  /*  ============================================================================================
   *   
   *                              Writing to SD Card
   *   
      ============================================================================================ */
  
  //---------------------------------------------------------------------------------------------- Time
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
  //---------------------------------------------------------------------------------------------- GPS
  logFile.print(gps_lat);
  logFile.print(", ");
  logFile.print(gps_long);
  logFile.print(", ");
  logFile.print(gps_alt);
  logFile.print(", ");
  logFile.print(gps_sats);
  logFile.print(", ");
  logFile.print(antenna_status);
  logFile.print(", ");
  logFile.print(gps_fixqual);
  logFile.print(", ");
  logFile.print(gps_fault_counter);
  logFile.print(", ");
  //---------------------------------------------------------------------------------------------- Temp, Pressure, Alt, Ascent Rate, Servo Position
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
  //---------------------------------------------------------------------------------------------- Cut-Down & Flight Details
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

   /*  ============================================================================================
   *   
   *                              Writing to Serial Monitor
   *   
       ============================================================================================ */
  
  //---------------------------------------------------------------------------------------------- Time
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
  //---------------------------------------------------------------------------------------------- GPS
  Serial.print(gps_lat);
  Serial.print(", ");
  Serial.print(gps_long);
  Serial.print(", ");
  Serial.print(gps_alt);
  Serial.print(", ");
  Serial.print(gps_sats);
  Serial.print(", ");
  Serial.print(antenna_status);
  Serial.print(", ");
  Serial.print(gps_fixqual);
  Serial.print(", ");
  Serial.print(gps_fault_counter);
  Serial.print(", ");
  //---------------------------------------------------------------------------------------------- Temp, Pressure, Alt, Ascent Rate, Servo Position
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
  //---------------------------------------------------------------------------------------------- Cut-Down & Flight Details
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

  Serial.println("Cut reason: ");
  Serial.println(cut_reason);

  

  //Cleaning up ascent-rate data
  for(int i = 0; i < 4; i++)
  {
    prev_ascent_rate[i] = prev_ascent_rate[i+1];
  }
  prev_alt = alt;
  prev_time = now_seconds;
  prev_ascent_rate[4] = ascent_rate;

  delay(1000);
} // End of Loop

/*  ============================================================================================
   *   
   *                                 Additional Methods, etc.
   *   
      ============================================================================================ */

void cutdown() // Standard Cut-down
{
  digitalWrite(CUTDOWN_PIN_1, HIGH);
  delay(8000);
  digitalWrite(CUTDOWN_PIN_1, LOW);
}

void yolo_cutdown() // Last-Attempt Cut-Down
{
  delay(120000); 
  digitalWrite(CUTDOWN_PIN_1, HIGH);
}

time_t getTeensy3Time() // Getting Time from RTC
{
  return Teensy3Clock.get();
}


int geofence_check(float long_coord, float lat_coord, int fix_qual) // Checks Geofence Compliance (0 = do not cut down, 1 = cut down, 2 = bad fix)
{
  // Checks fix quality first
  if(fix_qual == 0)
  {
    return BAD_FIX;
  }
  // Checks Geofence compliance and adds to fault counter
  if(long_coord > LONG_EAST_BOUND || long_coord < LONG_WEST_BOUND)
  {
    // Noncompliant
    gps_fault_counter++;
  }
  else if(lat_coord < LAT_SOUTH_BOUND || lat_coord > LAT_NORTH_BOUND)
  {
    //Noncompliant
    gps_fault_counter++;
  }
  else
  {
    //Compliant, resets margin counter
    gps_fault_counter = 0;
  }
  // Returns cut-down command
  if(gps_fault_counter > 10)
  {
    return CUT;
  }
  else
  {
    return NOT_CUT;
  }

}

void readGPS() // Reads GPS, it seems
{
  GPS.read();
}
