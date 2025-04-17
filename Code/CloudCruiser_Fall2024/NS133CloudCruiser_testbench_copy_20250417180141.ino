/*
This is the code for Cloud Cruiser. Like all of it. For NS-131. For country and for kingdom
Andy Ngo - Please keep me in your prayers
Elizabeth Moonjelly - Me 2
Arnav mention!
*/

//Include them libraries
#include <Stepper.h>
#include <Wire.h> //Needed for I2C
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //GPS library
#include <MS5607.h> //PTEMP
#include <STM32SD.h> //SD
#include <Arduino.h>
#include "PID_RT.h" //PID
#define MINUTE 60000
#define FEET_TO_MM 304.8
#define CUTDOWN_PIN 6   // Pin to activate cut-down mechanism
#define MAX_FLIGHT_DURATION 120 * MINUTE
#define FLOAT_ALTITUDE 75000 * FEET_TO_MM  // Target altitude
#define FLOAT_DURATION 10 * MINUTE  // Float time before cut-down
#define LOG_THRESHOLD 10000  // Only log if altitude deviation is significant
#define TEST_THRESHOLD 100
#define CUTDOWN_ALTITUDE 95000 * FEET_TO_MM  // Altitude threshold for cut-down
#define CUTDOWN_FAULT_LIMIT 10  // Number of consecutive altitude checks before triggering cut-down
#define CUT 1
#define NOT_CUT 0
//SD card
#ifndef SD_DETECT_PIN
#define SD_DETECT_PIN SD_DETECT_NONE
#endif
SFE_UBLOX_GNSS myGNSS;
MS5607 P_Sens;
File myFile;
String logBuffer = "";
float P_val,T_val,H_val, motorcheck, PIDsetPoint, error;
int test, gpscal, hour, minute, second;
int vent_counter = 0;
int ventprint,initial_altitude;
byte SIV;
long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
long diff_time = 0;
long latitude, longitude, altitude;
String timestamp;
PID_RT PID;  // 
int op = 0; //PID output
float input = 0; //PID input?
const int IN1 = 10;  // Motor A - Forward
const int IN2 = 11;  // Motor A - Reverse
const int IN3 = 12; // Motor B - Forward
const int IN4 = 13; // Motor B - Reverse
const int BUFFER_LIMIT = 256;
unsigned long lastFlushTime = 0;
const unsigned long FLUSH_INTERVAL = 10000;
int alt_fault_counter = 0;  // Tracks consecutive altitude violations
bool floating = false;
unsigned long float_start_time = 0;
int cutdownflag = 0;
int raw_output;

void triggerCutdown();
void adjustValve();
void logtoSD();
int alt_check();
void moveForward();
void moveBackward();
void stopMotor();
void setup() {
  Serial.begin(115200);
  // Set all motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
    // Ensure the motor starts off
  stopMotor();

  while (!Serial) {
    ;  // wait for serial port to connect. Needed for Leonardo only
  }
  Wire.begin(); //Initialize I2C


    //Check SD card type beat
  if (!SD.begin()) {
      Serial.println("SD initialization failed!");
      test = 1;
        return;
  }

  Serial.println("SD initialized.");
  //Create data file
  Serial.println("Creating NS133.txt...");
  Serial.flush();
  myFile = SD.open("NS133.txt", FILE_WRITE);
  myFile.println("Time,Altitude,Pressure,Temp,Vent_Count,Error,PID out,Cut_Status");
  myFile.close();


  Serial.println("Test1");
    //Check if GPS is init
  if (myGNSS.begin() == false){ //Connect to the u-blox module using Wire port  
  Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
  while (1)
    ;
}
myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

Serial.println("Test2");

//Check to see if PTEMP is init
  if(!P_Sens.begin()){
  Serial.println("Error in Communicating with sensor, check your connections!");
}else{
  Serial.println("MS5607 initialization successful!");
}
Serial.println("Test3");
PIDsetPoint = FLOAT_ALTITUDE;
PID.setPoint(PIDsetPoint*FEET_TO_MM); //Desired altitude in mm
PID.setOutputRange(0, 255);  // PWM range
PID.setInterval(500);
PID.setK(0.3, 0.005, 0.0001);    // Smaller gains to prevent overreaction
PID.start();
 

//Keep looking for SIV 
SIV = myGNSS.getSIV(); //get initial SIV
while (SIV < 1){ //Assumes first grab will be 0, keep looking 
Serial.println(SIV);
SIV = myGNSS.getSIV();
}
Serial.println(SIV);
Serial.println("SIV initialized!");
//set initial altitude
initial_altitude = myGNSS.getAltitude();
}

void loop() {
  Serial.println("IN LOOP IN LOOP");
    //  Get altitude
    input = myGNSS.getAltitude();
    error = PIDsetPoint - input;
        // Collect sensor data
    P_val = P_Sens.getPressure();
    T_val = P_Sens.getTemperature();
        //  Cut-down logic
    if (alt_check(input) == CUT) {
    triggerCutdown();
    }
    // Run PID control for helium venting
    if (PID.compute(input)) {
      raw_output = PID.getOutput();
        op = 255-raw_output;
        adjustValve(op);
        delay(100);
    }
    // Check if we should log (every significant altitude change)
    if (abs(input - initial_altitude) > LOG_THRESHOLD) {
    Serial.println("Logging to buffer");
    logToSD();
  }

  // Flush if buffer full or time passed
  if (logBuffer.length() >= BUFFER_LIMIT || millis() - lastFlushTime >= FLUSH_INTERVAL) {
    flushLog();
  }
  //hard cap time limit
    if (millis() > 120*MINUTE){
      flushLog();
      Serial.println("Closed that jawn!");
    }

}
void triggerCutdown() {
    Serial.println(" CUTDOWN ACTIVATED! ");
    digitalWrite(CUTDOWN_PIN, HIGH);
    delay(5000);  // Allow time for cut-down activation
    digitalWrite(CUTDOWN_PIN, LOW);
    cutdownflag = 1;
}
// need to fix valve control
void adjustValve(int op) {
      if (op < 50) {
          stopMotor();  // No venting
      } else if (op < 200) {
          moveBackward();  // Partial venting
          delay(5000);  // Vent for a short time
          moveForward();
          delay(5000);
          stopMotor();
          vent_counter = vent_counter+1;
      } else {
          moveBackward();  // Full venting
          delay(10000);  // Vent for a longer time
          moveForward();
          delay(10000);
          stopMotor();
          vent_counter = vent_counter+2;
      }
    
}

void logToSD() {
    ventprint = vent_counter;
    logBuffer += String(millis()) + "," +
                 String(input) + "," +
                 String(P_val) + "," +
                 String(T_val) + "," +
                 String(ventprint) + "," +
                 String(error) + "," +
                 String(op) + "," +
                 String(cutdownflag) + "\n";
}

void flushLog() {
  if (logBuffer.length() == 0) return;
  
  myFile = SD.open("NS133.txt", FILE_WRITE); //local file object
  if (myFile) {
    myFile.seek(myFile.size());
    Serial.println("Buffer contents:\n" + logBuffer);
    Serial.print("Writing at position: ");
    Serial.println(myFile.position()); // should be at the end of the file
    myFile.write((const uint8_t*)logBuffer.c_str(), logBuffer.length()); //wRITE that jawn to the SD card
    myFile.close();
    Serial.println("Flushed that shii");
  } else {
    Serial.println("I did not open it");
  }
  logBuffer = ""; // Clear the buffer!
  lastFlushTime = millis();
}

int alt_check(float alt_val) {
    if (alt_val >= CUTDOWN_ALTITUDE) {
        alt_fault_counter++;  // Increment fault counter if altitude exceeds threshold
    } else {
        alt_fault_counter = 0;  // Reset counter if altitude is below threshold
    }
    // Trigger cut-down if altitude has been exceeded for 10 consecutive checks
    if (alt_fault_counter > CUTDOWN_FAULT_LIMIT) {
        return CUT;
    } else {
        return NOT_CUT;
    }
}
void moveForward() { //motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() { //motor backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotor() { //stop motor
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
