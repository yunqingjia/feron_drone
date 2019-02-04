// Simple program to test the Teensy's
// ability to control ESCs/Brushless Motors

#include "Servo.h" 

#define ESC_PIN         (23)  // PWM pin for signaling ESC
#define DRIVE_PIN       (2)   // Drive pin for power MOSFET

Servo ESC;  
int speed = 0;

void arm(){
  digitalWrite(DRIVE_PIN, HIGH);
  setSpeed(0); //Sets speed variable
  delay(1000);
}

/* Callibrate ESC's PWM range for first use */
void callibrate() {
  digitalWrite(DRIVE_PIN, LOW);   // Disconnect ESC from power
  delay(500);                     // Wait 500ms
  setSpeed(1000);            // Request full speed
  digitalWrite(DRIVE_PIN, HIGH);  // Reconnect ESC to power
  delay(5000);                     // Wait 5s
  setSpeed(0);               // Request 0 speed
}

void setSpeed(int input_speed){
  int us = map(input_speed, 0, 1000, 1000, 2000); //Sets servo positions to different speed
  ESC.writeMicroseconds(us);
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Configure MOSFET drive pin
  pinMode(DRIVE_PIN, OUTPUT);
  digitalWrite(DRIVE_PIN, LOW);
  
  ESC.attach(ESC_PIN); // Adds ESC to certain pin.
}

void loop() {
  if (Serial.available()) {
    if (Serial.peek() == ' ') {
      Serial.println("Callibrating ESC");
      callibrate();
      Serial.read();
    } else {
      speed = Serial.parseInt();
      Serial.println(speed);
      setSpeed(speed);
    }
  }
}
