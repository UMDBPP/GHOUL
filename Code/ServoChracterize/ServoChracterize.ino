#include <Servo.h>

#define SERVO_PIN 2
#define FEEDBACK_PIN A9

#define VENT_CLOSED_POSITION 0
#define VENT_OPEN_POSITION 180

Servo ventValve;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  ventValve.attach(SERVO_PIN);
  delay(50);
  pinMode(FEEDBACK_PIN, INPUT);
  float raw_servo_pos = analogRead(FEEDBACK_PIN);
  float servo_pos = (raw_servo_pos/1024)*180;
  Serial.println(raw_servo_pos);
  Serial.println(servo_pos);
}

void loop() {
  // put your main code here, to run repeatedly
  if(Serial.available() > 0)
  {
    int servoPos = Serial.parseInt();
    Serial.println(servoPos);
    delay(500);
    ventValve.write(servoPos);
    delay(2000);
    float raw_servo_pos = analogRead(FEEDBACK_PIN);
    float servo_pos = (raw_servo_pos/1024)*180;
    Serial.println(raw_servo_pos);
    Serial.println(servo_pos);
  }
}
