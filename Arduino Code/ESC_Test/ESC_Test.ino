// Simple program to test the Arduino's
// ability to control ESCs/Brushless Motors

#include "Servo.h" 

#define ESC_PIN         (6)   // PWM pin for signaling ESC
#define DRIVE_PIN       (10)  // Drive pin for power MOSFET

Servo ESC;  
int speed = 0;

void arm(){
  Serial.print("Arming ESC... ");
  digitalWrite(DRIVE_PIN, LOW);   // Disconnect ESC from power
  delay(500);                     // Wait 500ms for ESC to power down
  setSpeed(0);                    // Set speed to 0
  digitalWrite(DRIVE_PIN, HIGH);  // Reconnect ESC to power
  delay(2500);                    // 2.5 second delay for ESC to respond
  Serial.println("Arming complete");
}

/* Callibrate ESC's PWM range for first use */
void callibrate() {
  Serial.print("Calibrating ESC... ");
  digitalWrite(DRIVE_PIN, LOW);   // Disconnect ESC from power
  delay(500);                     // Wait 500ms
  setSpeed(1000);                 // Request full speed
  digitalWrite(DRIVE_PIN, HIGH);  // Reconnect ESC to power
  delay(5000);                    // Wait 5 seconds
  setSpeed(0);                    // Request 0 speed
  delay(8000);                    // Wait 8 seconds
  Serial.println("Calibration complete");
}

void setSpeed(int input_speed){
  //Sets servo positions to different speed
  int us = map(input_speed, 0, 1000, 1000, 2000);
  ESC.writeMicroseconds(us);
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Configure MOSFET drive pin
  pinMode(DRIVE_PIN, OUTPUT);
  digitalWrite(DRIVE_PIN, LOW);

  // Attach ESC to designated pin.
  ESC.attach(ESC_PIN);

  // Arm ESC
  arm();
}

void loop() {
  if (Serial.available()) {
    // Check for calibration request
    if (Serial.peek() == 'c') {
      Serial.println("Callibrating ESC");
      callibrate();
      Serial.read();
    }
    // Otherwise, interpret as new throttle value
    else {
      speed = Serial.parseInt();
      Serial.println(speed);
      setSpeed(speed);
    }
  }
}
