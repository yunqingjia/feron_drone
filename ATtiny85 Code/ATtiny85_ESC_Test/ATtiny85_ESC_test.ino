// Simple program to test the ATtiny85's
// ability to control ESCs/Brushless Motors

#include "SoftwareSerial.h"
#include "Servo8Bit.h" 

#define ESC_PIN         (0)   // PWM pin for signaling ESC
#define RX_PIN          (3)   // RX pin for SoftwareSerial
#define TX_PIN          (4)   // TX pin for SoftwareSerial
#define DRIVE_PIN       (1)   // Drive pin for power MOSFET

// Initialize SoftwareSerial pins
SoftwareSerial  mySerial(RX_PIN, TX_PIN);

// Create an Servo8Bit object
Servo8Bit ESC;  
int speed = 0;

void arm(){
  digitalWrite(DRIVE_PIN, HIGH);
  setSpeed(0); //Sets speed variable
  delay(1000);
}

/* Callibrate ESC's PWM range for first use */
void callibrate(Servo8Bit *ESC) {
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
  mySerial.begin(9600);

  // Configure MOSFET drive pin
  pinMode(DRIVE_PIN, OUTPUT);
  digitalWrite(DRIVE_PIN, LOW);
  
  ESC.attach(0); //Adds ESC to certain pin.
  arm();
}

void loop() {
  if (mySerial.available()) {
    if (mySerial.peek() == ' ') {
      mySerial.println("Callibrating ESC");
      callibrate(&ESC);
      mySerial.read();
    } else {
      speed = mySerial.parseInt();
      mySerial.println(speed);
      setSpeed(speed);
    }
  }
}
