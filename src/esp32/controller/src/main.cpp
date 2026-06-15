#include <Arduino.h>
#include <ESP32Servo.h>

Servo myServo;

const int shoulderServo = 23;

void setup() {
  Serial.begin(115200);
  myServo.setPeriodHertz(50);
  myServo.attach(shoulderServo, 500, 2500);
}

void loop() {
  Serial.println("Enter servo angle (0-180):");

  while (!Serial.available()) {}

  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input.length() == 0) {
    return;
  }

  int angle = input.toInt();
  angle = constrain(angle, 0, 180);

  myServo.write(angle);
  Serial.print("Servo angle set to: ");
  Serial.println(angle); 
}