#include <TimeLib.h>

#define MOSFET_PIN 6
#define LED_PIN 13

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(MOSFET_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(5000);
  digitalWrite(MOSFET_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  delay(8000);
  digitalWrite(MOSFET_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
}
