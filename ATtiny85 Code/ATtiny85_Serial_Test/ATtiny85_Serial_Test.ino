#include "SoftwareSerial.h"

#define RX_PIN          (3)   // RX pin for SoftwareSerial
#define TX_PIN          (4)   // TX pin for SoftwareSerial

// Initialize SoftwareSerial pins
SoftwareSerial  mySerial(RX_PIN, TX_PIN);

int incomingByte = 0; // For incoming serial data
int led = 0;

void setup() {
  // Initialize serial communication
  mySerial.begin(9600);
  pinMode(led, OUTPUT);

  mySerial.println("Hello from Serial!  Type a character!");
}

void loop() {
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);               // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);               // wait for a second
  
  if(mySerial.available()) {
    incomingByte = mySerial.read();
    mySerial.print("You typed '");
    mySerial.print(char(incomingByte));
    mySerial.println("'!");
  }
}
