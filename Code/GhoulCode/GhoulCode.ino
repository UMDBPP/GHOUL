//GHOUL flight software written by:
//Michael Kalin and Jeremy Joseph (JJ) Kuznetsov with love and support from Kruti Geeta-Rajnikant, Daniel Grammar and Akemi Takeuchi

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <TimeLib.h>
#include <RTClib.h>
#include <Adafruit_GPS.h>
#include <XBee.h>
#include <Adafruit_MMA8451.h>

//pin definitions
#define SCL_PIN 7
#define SDA_PIN 8
#define SERVO_PIN 2
#define FEEDBACK_PIN A9
#define CUTDOWN_PIN_1 35
#define CUTDOWN_PIN_2 36
#define LOGGER_PIN BUILTIN_SDCARD
#define GPSSerial Serial2
#define XBeeSerial Serial1

//servo characteristics
#define VENT_OPEN_POS 83
#define VENT_CLOSED_POS 18

//float parameters
#define PRE_VENT_ALT 23500
#define FLOAT_ALT 40000
#define PRE_VENT_RATIO .2
#define VENT_TIMER 1800 //seconds
#define SEA_LEVEL_PRESSURE 982.45 //mbar

//cut-down parameters
#define CUT_INTERVAL 60 //seconds
#define TOTAL_CUTS 3
#define CUTDOWN_ALTITUDE 32000 //meters
#define CUTDOWN_TIMER_TRIGGER_ALT 1000 //meters
#define CUTDOWN_TIMER_DURATION 9000 //seconds
#define ARATE_TRIGGER_ALT 40000 //meters
#define ASCENT_RATE_TRIGGER 1 //meters per second
#define LONG_EAST_BOUND -70 
#define LONG_WEST_BOUND -85
#define LAT_NORTH_BOUND 45
#define LAT_SOUTH_BOUND 32

//flags
#define CLOSED 0
#define OPEN 1
#define XBEE_CLOSED 2
#define XBEE_OPENED 3
#define PRE_VENT_NOT_DONE 0
#define PRE_VENT_DONE 1
#define FLOAT_VENT_NOT_DONE 0
#define FLOAT_VENT_DONE 1
#define NORMAL_ASCENT 0
#define PRE_VENTING 1
#define PRE_VENTED 2
#define FLOAT_VENTING 3
#define FLOATING 4
#define XBEE_FLOATING 5
#define NOT_CUT 0
#define CUT 1
#define BAD_FIX 2
#define TIMER_NOT_STARTED 0
#define TIMER_STARTED 1
#define ARATE_TRIGGER_NOT_STARTED 0
#define ARATE_TRIGGER_STARTED 1
#define CUT_REASON_TIMER 1
#define CUT_REASON_ALTITUDE 2
#define CUT_REASON_ASCENT_RATE 3
#define CUT_REASON_GEOFENCE 4
#define CUT_REASON_XBEE 5
#define XBEE_DO_NOTHING 0
#define XBEE_BITS_TEST 1
#define XBEE_GROUND_TEST 2
#define XBEE_OPEN 3
#define XBEE_CLOSE 4
#define XBEE_CUTDOWN 5

//timer interrupts
IntervalTimer gpsTimer;

//timers
elapsedMillis burn_timer;

//sensors
Adafruit_BMP280 bmp;
Adafruit_MMA8451 mma = Adafruit_MMA8451();
Servo ventValve;
File logFile;
Adafruit_GPS GPS(&GPSSerial);
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();

//xbee stuff
const uint32_t UniSH = 0x0013A200;    //Common across any and all XBees
const uint32_t BitsSL = 0x417B4A3B;   //BITS   (white)Specific to the XBee on Bits (the Serial Low address value)
const uint32_t GroundSL = 0x417B4A36; //GndStn (u.fl)
ZBTxStatusResponse txStatus = ZBTxStatusResponse(); //What lets the library check if things went through
ZBRxResponse rx = ZBRxResponse();                   //Similar to above
ModemStatusResponse msr = ModemStatusResponse();    //And more
const int xbeeRecBufSize = 50; //Rec must be ~15bytes larger than send because
const int xbeeSendBufSize = 35;//there is overhead in the transmission that gets parsed out
uint8_t xbeeRecBuf[xbeeRecBufSize];
uint8_t xbeeSendBuf[xbeeSendBufSize];

//storage
float prev_gps_alt = 0;
float prev_pressure_alt = 0;
uint32_t prev_time = 1;
float prev_pressure_ascent_rate[5] = {0, 0, 0, 0, 0};
float prev_gps_ascent_rate[5] = {0, 0, 0, 0, 0};
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
float raw_servo_pos;
float servo_pos;

//fault counters
int alt_fault_counter = 0;
int ar_fault_counter = 0;
int gps_fault_counter = 0;

//flags
int pre_vent_status = PRE_VENT_NOT_DONE;              //0 = not done, 1 = done
int float_vent_status = FLOAT_VENT_NOT_DONE;          //0 = not done, 1 = done
int vent_status = CLOSED;                             //0 = closed, 1 = open
int float_status = NORMAL_ASCENT;                     //0 = normal ascent, 1 = pre-venting, 2 = float-venting, 3 = floating
int cut_status = NOT_CUT;                             //0 = not cut, 1 = cut
int timer_status = TIMER_NOT_STARTED;                 //0 = timer not started, 1 = timer started
int arate_trigger_status = ARATE_TRIGGER_NOT_STARTED; //0 = arate trigger not started, 1 = arate trigger started
int geofence_status = NOT_CUT;                        //0 = geofence not started, 1 = geofence trigger started, 2 = bad fix
int cut_reason = NOT_CUT;                             //0 = not cut, 1 = timer, 2 = altitude, 3 = ascent rate, 4 = geofence
int xbee_status = XBEE_DO_NOTHING;                    //0 = do nothing, 1 = bits test (print to serial/file), 2 = ground test, 3 = open, 4 = close, 5 = cutdown
int cutdown_flag = 1;

void setup() {
  Serial.begin(9600);

  Serial.println("Powered on!");
  logFile = SD.open("datalog.txt", FILE_WRITE);
  logFile.println("Powered on!");
  logFile.close();

  // Reassign default pins for I2C bus
  Wire.setSCL(SCL_PIN);
  Wire.setSDA(SDA_PIN);

  // Set up XBee Serial
  XBeeSerial.begin(9600);
  XBeeSerial.setRX(27);
  XBeeSerial.setTX(26);
  
  // XBee Set up
  xbee.setSerial(XBeeSerial);
  String("GHOULxbee_ON").getBytes(xbeeSendBuf, xbeeSendBufSize);
  xbeeSend(GroundSL, xbeeSendBuf);

  // Initiate/close servo
  ventValve.attach(SERVO_PIN);
  delay(50);
  ventValve.write(VENT_CLOSED_POS);

  // Servo Analog Feedback Pin
  pinMode(FEEDBACK_PIN, INPUT);
  raw_servo_pos = analogRead(FEEDBACK_PIN);
  servo_pos = (raw_servo_pos/1024)*180;

  // Initiate cut-down pins
  pinMode(CUTDOWN_PIN_1, OUTPUT);
  digitalWrite(CUTDOWN_PIN_1, LOW);
  pinMode(CUTDOWN_PIN_2, OUTPUT);
  digitalWrite(CUTDOWN_PIN_2, LOW);

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

  // Initiate Accelerometer
  if(!mma.begin())
    Serial.println("Error accelerometer not found!");
  else
    Serial.println("Acceleromter found!");

  mma.setRange(MMA8451_RANGE_8_G);

  // Initiate GPS
  GPSSerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGAONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  // Check/Initiate SD Logger
  if(!SD.begin(LOGGER_PIN))
    Serial.println("Error: SD Card Logger Not Initialized");
  else
    Serial.println("SD card initialized");

  // Interrupt Timer for GPS
  gpsTimer.begin(readGPS, 1000); //try changing to 10000

  Serial.println("Setup done!");
  logFile = SD.open("datalog.txt", FILE_WRITE);
  logFile.println("Setup done!");
  logFile.close();
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
  float pressure_alt = bmp.readAltitude(SEA_LEVEL_PRESSURE);

  //read accelerometer
  sensors_event_t event; 
  mma.getEvent(&event);
  float x = event.acceleration.x;
  float y = event.acceleration.y;
  float z = event.acceleration.z;

   //read gps data -------------------------------------------------------------------------------- gps (long, lat, alt, fix, sats #)
  noInterrupts();
  if(GPS.newNMEAreceived())
  {
    Serial.println("new nmea recieved!");
    Serial.println(GPS.lastNMEA());
    logFile = SD.open("datalog.txt", FILE_WRITE);
    logFile.print(GPS.lastNMEA());
    logFile.close();
    if(!GPS.parse(GPS.lastNMEA()))
    {
      Serial.println("GPS Parse failed!");
    }
    gps_fixqual = GPS.fix;
    if(gps_fixqual == 1)
    {
      gps_lat = GPS.latitudeDegrees;
      gps_long = GPS.longitudeDegrees;
      gps_alt = GPS.altitude;
      gps_sats = GPS.satellites; 
    }
  }
  interrupts();

  //check xbee
  delay(10);
  xbee_status = xbeeRead();
  delay(10);
  //Serial.print("XBee Status: ");
  //Serial.println(xbee_status);

  /*  ============================================================================================
   *   
   *                              Calculations and Decisions
   *   
       ============================================================================================ */
       
  
  //calc pressure_ascent rate ------------------------------------------------------------------------------ Pressure Ascent Rate Calculation
  float curr_pressure_ascent_rate = (pressure_alt - prev_pressure_alt)/(now_seconds - prev_time);
  float pressure_ascent_rate = (prev_pressure_ascent_rate[0] + prev_pressure_ascent_rate[1] + prev_pressure_ascent_rate[2] + prev_pressure_ascent_rate[3] + prev_pressure_ascent_rate[4] + curr_pressure_ascent_rate)/6;

  //calc gps ascent rate ----------------------------------------------------------------------------------- GPS Ascent Rate Calculation
  float curr_gps_ascent_rate = (gps_alt - prev_gps_alt)/(now_seconds - prev_time);
  float gps_ascent_rate = (prev_gps_ascent_rate[0] + prev_gps_ascent_rate[1] + prev_gps_ascent_rate[2] + prev_gps_ascent_rate[3] + prev_gps_ascent_rate[4] + curr_gps_ascent_rate)/6;

  //choose whether to use pressure or gps ascent rate
  float alt;
  if (gps_fixqual == 1) //we should check more than this
    alt = gps_alt;
  else
    alt = pressure_alt;

  float ascent_rate;
  if (gps_fixqual == 1) // we should check more than this
    ascent_rate = gps_ascent_rate;
  else
    ascent_rate = pressure_ascent_rate;

  //read servo position
  raw_servo_pos = analogRead(FEEDBACK_PIN);
  servo_pos = (raw_servo_pos/1024)*180;

  //if vent is closed, see if we should open it --------------------------------------------------- Should we open vent?
  if(xbee_status == XBEE_OPEN)// && vent_status != XBEE_OPENED)
  {
    ventValve.write(VENT_OPEN_POS);
    vent_status = XBEE_OPENED;
    rate_at_open = ascent_rate;
    vent_open_time = now_seconds;
  }
  else if(xbee_status == XBEE_CLOSE && vent_status != XBEE_CLOSED)
  {
    ventValve.write(VENT_CLOSED_POS);
    vent_status = XBEE_CLOSED;
  }
  
  if(vent_status == XBEE_OPENED)
  {
    ventValve.write(VENT_OPEN_POS);
    if(ascent_rate < 1 && alt > CUTDOWN_TIMER_TRIGGER_ALT && gps_fixqual == 1)
    {
      ventValve.write(VENT_CLOSED_POS);
      vent_status = XBEE_CLOSED;
      float_status = FLOATING;
      float_vent_status = FLOAT_VENT_DONE;
    }
  }
  else if(vent_status == XBEE_CLOSED)
  {
    //do nothing except keep closed
    ventValve.write(VENT_CLOSED_POS);
  }
  else
  {
    if(vent_status == CLOSED)
    {
      ventValve.write(VENT_CLOSED_POS);
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
      ventValve.write(VENT_OPEN_POS);
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
  }
  //  =============================================================================================
  //  
  //                                          CUT DOWN TRIGGERS
  //
  //  =============================================================================================


  //xbee trigger ---------------------------------------------------------------------------------- XBee Trigger
  if (cut_status == NOT_CUT && xbee_status == XBEE_CUTDOWN)
  {
    cutdown();
    cut_status = CUT;
    num_cuts++;
    next_cut_time = now_seconds + CUT_INTERVAL;
    cut_reason = CUT_REASON_XBEE;
  }
  
  //altitude trigger ------------------------------------------------------------------------------ Altitude Trigger
  if(cut_status == NOT_CUT && alt_check(alt) == CUT)
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
  
  if(arate_trigger_status == ARATE_TRIGGER_STARTED && cut_status == NOT_CUT && ar_check(ascent_rate) == CUT)
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
  geofence_status = geofence_check(gps_long, gps_lat, gps_fixqual);
  if(cut_status == NOT_CUT && geofence_status == CUT)
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
  logFile.print(gps_lat, 5);
  logFile.print(", ");
  logFile.print(gps_long, 5);
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
  logFile.print(pressure_alt);
  logFile.print(", ");
  logFile.print(alt);
  logFile.print(", ");
  logFile.print(curr_pressure_ascent_rate);
  logFile.print(", ");
  logFile.print(pressure_ascent_rate);
  logFile.print(", ");
  logFile.print(curr_gps_ascent_rate);
  logFile.print(", ");
  logFile.print(gps_ascent_rate);
  logFile.print(", ");
  logFile.print(ascent_rate);
  logFile.print(", ");
  logFile.print(x);
  logFile.print(", ");
  logFile.print(y);
  logFile.print(", ");
  logFile.print(z);
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
  logFile.print(cut_reason);
  logFile.print(", ");
  logFile.print(timer_status);
  logFile.print(", ");
  logFile.print(arate_trigger_status);
  logFile.print(", ");
  logFile.print(geofence_status);
  logFile.print(", ");
  logFile.print(xbee_status);
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
  Serial.print(gps_lat, 5);
  Serial.print(", ");
  Serial.print(gps_long, 5);
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
  Serial.print(curr_pressure_ascent_rate);
  Serial.print(", ");
  Serial.print(pressure_ascent_rate);
  Serial.print(", ");
  Serial.print(curr_gps_ascent_rate);
  Serial.print(", ");
  Serial.print(gps_ascent_rate);
  Serial.print(", ");
  Serial.print(ascent_rate);
  Serial.print(", ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(z);
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
  Serial.print(cut_reason);
  Serial.print(", ");
  Serial.print(timer_status);
  Serial.print(", ");
  Serial.print(arate_trigger_status);
  Serial.print(", ");
  Serial.print(geofence_status);
  Serial.print(", ");
  Serial.print(xbee_status);
  Serial.println();
  
  //Cleaning up ascent-rate data
  for(int i = 0; i < 4; i++)
  {
    prev_pressure_ascent_rate[i] = prev_pressure_ascent_rate[i+1];
    prev_gps_ascent_rate[i] = prev_gps_ascent_rate[i+1];
  }
  prev_gps_alt = gps_alt;
  prev_pressure_alt = pressure_alt;
  prev_time = now_seconds;
  prev_pressure_ascent_rate[4] = pressure_ascent_rate;
  prev_gps_ascent_rate[4] = gps_ascent_rate;

  //Reset XBee status
  xbee_status = 0;

  delay(1000);
}

/*  ============================================================================================
   *   
   *                                 Additional Methods, etc.
   *   
      ============================================================================================ */

void cutdown() // Standard Cut-down
{
  if(cutdown_flag == 1)
  {
    digitalWrite(CUTDOWN_PIN_1, HIGH);
    delay(8000);
    digitalWrite(CUTDOWN_PIN_1, LOW);
    cutdown_flag = 2;
  }
  else
  {
    digitalWrite(CUTDOWN_PIN_2, HIGH);
    delay(8000);
    digitalWrite(CUTDOWN_PIN_2, LOW);
    cutdown_flag = 1;
  }

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

//---------------------------------------------------------------------------------------------- Geofence Cutdown Check
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
  // Returns proper value for cut-down command
  if(gps_fault_counter > 10)
  {
    return CUT;
  }
  else
  {
    return NOT_CUT;
  }

}
//---------------------------------------------------------------------------------------------- Altitude Cutdown Check
int alt_check(int alt_val)
{
  if(alt_val >= CUTDOWN_ALTITUDE)
  {
    alt_fault_counter++;
  }
  else
    alt_fault_counter = 0;

  // Returns proper value for cut-down command
  if(alt_fault_counter > 10)
  {
    return CUT;
  }
  else
    return NOT_CUT;
}

//---------------------------------------------------------------------------------------------- Ascent-Rate Cutdown Check
int ar_check(int arate)
{
  if(arate < ASCENT_RATE_TRIGGER)
  {
    ar_fault_counter++;
  }
  else
    ar_fault_counter = 0;
  
  if(ar_fault_counter > 10)
  {
    return CUT;
  }
  else
    return NOT_CUT;
}

void readGPS() // Reads GPS, it seems
{
  //Serial.println("Reading gps");
  GPS.read();
}

//xbee methods
bool xbeeSend(uint32_t TargetSL,uint8_t* payload){
  XBeeAddress64 TargetAddress = XBeeAddress64(UniSH,TargetSL);      //The full address, probably could be done more efficiently, oh well
  ZBTxRequest zbTx = ZBTxRequest(TargetAddress, payload, xbeeSendBufSize); //Assembles Packet
  xbee.send(zbTx);                                                  //Sends packet
  memset(xbeeSendBuf, 0, xbeeSendBufSize);                          //Nukes buffer
  if (xbee.readPacket(500)) {                                       //Checks Reception
    if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {   //If rec
      xbee.getResponse().getZBTxStatusResponse(txStatus);
      if (txStatus.getDeliveryStatus() == SUCCESS) {                //If positive transmit response
        Serial.println("SuccessfulTransmit");
        return true;
      } else {
        Serial.println("TxFail");
        return false;
      } 
    }
  } else if (xbee.getResponse().isError()) { //Stil have yet to see this trigger, might be broken...
    Serial.print("Error reading packet.  Error code: ");
    Serial.println(xbee.getResponse().getErrorCode());
  } else {
    Serial.println("Send Failure, check that remote XBee is powered on");  
  }
  return false;
}

int xbeeRead(){
  xbee.readPacket(); //read serial buffer
    if (xbee.getResponse().isAvailable()) { //got something
      if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) { //got a TxRequestPacket
        xbee.getResponse().getZBRxResponse(rx);
        
        uint32_t incominglsb = rx.getRemoteAddress64().getLsb(); //The SL of the sender
        Serial.print("Incoming Packet From: ");
        Serial.println(incominglsb,HEX);
        if(rx.getPacketLength()>=xbeeRecBufSize){                //Probably means something is done broke
          Serial.print("Oversized Message: ");
          Serial.println(rx.getPacketLength());
        }
        memset(xbeeRecBuf, 0, xbeeRecBufSize); // Nukes old buffer
        memcpy(xbeeRecBuf,rx.getData(),rx.getPacketLength());
        if(incominglsb == BitsSL){ //Seperate methods to handle messages from different senders
          return processBitsMessage();    //prevents one payload from having the chance to be mistaken as another
        }
        if(incominglsb == GroundSL){ //Ground Station
          return processGroundMessage();
        }    
      }
    }
  return XBEE_DO_NOTHING;
}

int processBitsMessage(){ //Just print things to the monitor
  Serial.println("RecFromBits");
  Serial.write(xbeeRecBuf,xbeeRecBufSize);

  if(strstr((char*)xbeeRecBuf,"test")){ //Checks if "test" is within buffer
      Serial.println();
      Serial.println("BitsTest");
      String test_response = "TestAck " + String(servo_pos);
      test_response.getBytes(xbeeSendBuf,xbeeSendBufSize);
      xbeeSend(BitsSL,xbeeSendBuf);
      return XBEE_BITS_TEST;
  }
  if(strstr((char*)xbeeRecBuf,"open")){ //Checks if "test" is within buffer
      Serial.println();
      Serial.println("OpenTest");
      String("OpenAck").getBytes(xbeeSendBuf,xbeeSendBufSize);
      xbeeSend(BitsSL,xbeeSendBuf);
      return XBEE_OPEN;
  }
  if(strstr((char*)xbeeRecBuf,"close")){ 
      Serial.println();
      Serial.println("CloseTest");
      String("CloseAck").getBytes(xbeeSendBuf,xbeeSendBufSize);
      xbeeSend(BitsSL,xbeeSendBuf);
      return XBEE_CLOSE;
  }
  if(strstr((char*)xbeeRecBuf,"terminate")){ 
      Serial.println("");
      Serial.println("Terminate");
      String("TermAck").getBytes(xbeeSendBuf,xbeeSendBufSize);
      xbeeSend(BitsSL,xbeeSendBuf);
      return XBEE_CUTDOWN;
  }
  return XBEE_DO_NOTHING;
}

int processGroundMessage(){
  Serial.print("RecFromGND: ");
  Serial.write(xbeeRecBuf,xbeeRecBufSize);

  if(strstr((char*)xbeeRecBuf,"test")){
      Serial.println("");
      Serial.println("ackTest");
      String test_response = "TestAck " + String(servo_pos);
      test_response.getBytes(xbeeSendBuf,xbeeSendBufSize);
      xbeeSend(GroundSL,xbeeSendBuf);
      return XBEE_GROUND_TEST;
  }
  if(strstr((char*)xbeeRecBuf,"open")){
      Serial.println("");
      Serial.println("OpenAck");
      String("OpenAck").getBytes(xbeeSendBuf,xbeeSendBufSize);
      xbeeSend(GroundSL,xbeeSendBuf);
      return XBEE_OPEN;
  }
  if(strstr((char*)xbeeRecBuf,"close")){ 
      Serial.println("");
      Serial.println("CloseAck");
      String("CloseAck").getBytes(xbeeSendBuf,xbeeSendBufSize);
      xbeeSend(GroundSL,xbeeSendBuf);
      return XBEE_CLOSE;
  }
  if(strstr((char*)xbeeRecBuf,"terminate")){ 
      Serial.println("");
      Serial.println("TermAck");
      String("TermAck").getBytes(xbeeSendBuf,xbeeSendBufSize);
      xbeeSend(GroundSL,xbeeSendBuf);
      return XBEE_CUTDOWN;
  }
  return XBEE_DO_NOTHING;
}
