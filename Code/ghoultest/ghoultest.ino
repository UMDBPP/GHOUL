#include <TimeLib.h>

#define MOSFET_PIN_1 35
#define MOSFET_PIN_2 36
#define LED_PIN 13

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(MOSFET_PIN_1, OUTPUT);
  pinMode(MOSFET_PIN_2, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN_1, LOW);
  digitalWrite(MOSFET_PIN_2, LOW);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(5000);
  digitalWrite(MOSFET_PIN_1, HIGH);
  digitalWrite(LED_PIN, HIGH);
  delay(4000);
  digitalWrite(MOSFET_PIN_1, LOW);
  digitalWrite(LED_PIN, LOW);
  delay(10000);
  digitalWrite(MOSFET_PIN_2, HIGH);
  digitalWrite(LED_PIN, HIGH);
  delay(4000);
  digitalWrite(MOSFET_PIN_2, LOW);
  digitalWrite(LED_PIN, LOW);
  delay(30000);
}
