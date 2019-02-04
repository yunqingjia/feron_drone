// Program to manually control the ESC/Motor
// while measuring the arm's angle

#include "Servo.h"
#include "MsTimer2.h"

#define ESC_PIN       (6)   // PWM pin for signaling ESC
#define DRIVE_PIN     (10)  // Drive pin for power MOSFET
#define SENSOR_PIN    (A0)  // Pin for measuring arm angle
#define BUFFER_SIZE   (2)   // Used for filtering measurements

int sensorVal;

volatile float filterBuffer[BUFFER_SIZE] = {0};
volatile float filteredVal = 0.0;
volatile int index = 0;

/*
 * Lowpass moving average filter to
 * smooth analog sensor readings
 */
float filter(float value) {
  // Remove oldest value from moving average
  filteredVal -= filterBuffer[index] / BUFFER_SIZE;

  // Add new value to buffer and incrememnt index
  filterBuffer[index++] = value;

  // Add new value to moving average
  filteredVal += value / BUFFER_SIZE;

  // Prevent index out of bounds errors
  index %= BUFFER_SIZE;

  return filteredVal;
}
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

void sensorPrint() {
  sensorVal = analogRead(SENSOR_PIN);
  Serial.print("Sensor Reading: ");
  Serial.print(sensorVal);
  Serial.print("\tFiltered Angle Reading: ");
  Serial.println(filter(-0.3656*sensorVal+185.64), 2); // Insert your angle equation here if it is different than the default
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

  // Attach ISR for timer interrupt
  MsTimer2::set(10, sensorPrint); // Print data once per 10ms
  MsTimer2::start();
}

void loop() {
  // Check for user input
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
      setSpeed(speed);
    }
  }
}
