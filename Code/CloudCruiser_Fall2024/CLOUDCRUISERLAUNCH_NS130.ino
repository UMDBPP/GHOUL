/*
This is the code for Cloud Cruiser. Like all of it. For NS-130. For country and for kingdom
Andy Ngo - Please keep me in your prayers
Elizabeth Moonjelly - Me 2
*/
//Include them libraries
#include <Stepper.h>
#include <Wire.h> //Needed for I2C
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //GPS library
#include <MS5607.h> //PTEMP
#include <STM32SD.h> //SD
#include <Arduino.h>

// change this to the number of steps on your motor
#define STEPS 145
//SD card
#ifndef SD_DETECT_PIN
#define SD_DETECT_PIN SD_DETECT_NONE
#endif
//Instance class stepper w/ 4 GPIO pins. 
Stepper stepper(STEPS, 10, 11, 12, 13);
SFE_UBLOX_GNSS myGNSS;
MS5607 P_Sens;
File myFile;

float P_val,T_val,H_val, motorcheck;
int test, gpscal, hour, minute, second;
int vent_counter = 0;
int ventprint;
byte SIV;
long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
long diff_time = 0;
long latitude, longitude, altitude, initial_altitude;
String timestamp;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
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
  Serial.println("Creating NS130.txt...");
  Serial.flush();
  myFile = SD.open("NS130.txt", FILE_WRITE);
  myFile.println("Time,Lat,Long,Altitude,Pressure,Temp,Vent_Count");
  myFile.flush();
  myFile.close();

 

      //Check if GPS is init
    if (myGNSS.begin() == false){ //Connect to the u-blox module using Wire port  
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)



  //Check to see if PTEMP is init
    if(!P_Sens.begin()){
    Serial.println("Error in Communicating with sensor, check your connections!");
  }else{
    Serial.println("MS5607 initialization successful!");
  }

    // set the speed of the motor to ???
  stepper.setSpeed(30);

    //set inital altitude
        initial_altitude = myGNSS.getAltitude();

}

void loop() {
  // put your main code here, to run repeatedly:
  
  while (millis() < 6000000){
  Serial.println(diff_time);
  Serial.print("and");
  Serial.print(SIV);
  myFile = SD.open("NS130.txt", FILE_WRITE);
  SIV = myGNSS.getSIV();
  //Condition for writing to SD card, if satellites are in view, we are getting accurate data
    while (SIV > 0 && millis() < 6000000){
    Serial.println("Writing to SD card!");
    Serial.println("millis is");
    Serial.print(millis());
    //latlonif
    if (millis() - lastTime > 1000){
    lastTime = millis(); //Update the timer
    //Routine to collect GPS data
     latitude = myGNSS.getLatitude();


    longitude = myGNSS.getLongitude();


    altitude = myGNSS.getAltitude();

    hour = myGNSS.getHour();
    minute = myGNSS.getMinute();
    second = myGNSS.getSecond();
    timestamp = String(hour) + String(minute) + String(second);
  }
      //Read from PT sensor if ready
    if(P_Sens.readDigitalValue()){
    T_val = P_Sens.getTemperature();
    P_val = P_Sens.getPressure();
    H_val = P_Sens.getAltitude();
  }else{
    Serial.println("Error in reading digital value in sensor!");
  }
  motorcheck = millis() % 300000;
  Serial.println("motorcheck is ");
  Serial.print(motorcheck);
    //Want to activate motor every X Min, check condition
    if (0 < motorcheck && motorcheck < 2000){// && altitude-intial_altitude > 10000 )
        //Vent for 5 seconds. THIS WILL CHANGE THE MOST IN THE NEXT ITER OF CLOUD CRUISER
        Serial.println("Motorvation");
        vent_counter = vent_counter+1;
        ventprint = vent_counter;
        Serial.println("Vent counter is");
        Serial.print(vent_counter);
        stepper.step(-STEPS);
        delay(1000);
        stepper.step(STEPS);
        
    }
    //Write to SD
    myFile.println();
    myFile.print(millis());
    myFile.print(",");
    myFile.print(latitude);
    myFile.print(",");
    myFile.print(longitude);
    myFile.print(",");
    myFile.print(altitude);
    myFile.print(",");
    myFile.print(P_val);
    myFile.print(",");
    myFile.print(T_val);
    myFile.print(",");
    myFile.print(ventprint);
    myFile.flush();
    diff_time = millis();
    
    
    }
  myFile.close();
  Serial.println("File closed!");
}
  Serial.println("End of loop, pick a god and pray!");
 
  delay(1000);
}
