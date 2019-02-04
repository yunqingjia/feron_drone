// Simple ESC tester program to verify operation
// and operating range of new ESC/Motor pairs

#include <Servo.h> 

Servo ESC;
int speed = 0;

void arm(){
  setSpeed(0); //Sets speed variable
  delay(1000);
}

void setSpeed(int input_speed){
  int us = map(input_speed, 0, 1000, 1000, 2000); //Sets servo positions to different speed
  ESC.writeMicroseconds(us);
}

void setup() {
  Serial.begin(9600);
  ESC.attach(3); //Adds ESC to certain pin.
  arm();
}

void loop() {
  if (Serial.available()) {
    speed = Serial.parseInt();
    Serial.println(speed);
    setSpeed(speed);
  }
}
