// Simple program to test SerialPlot's ability
// to plot serial data from the Arduino

#include "MsTimer2.h"

volatile unsigned long count = 0;
volatile bool updated = false;

// Update count and set updated flag
void update() {
  count++;
  updated = true;
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Attatch ISR for timer interrupt
  MsTimer2::set(1, update);
  MsTimer2::start();
}

void loop() {
  // Pause and resume for any user input
  if (Serial.available()) {
    Serial.read();              // Clear entered data
    MsTimer2::stop();           // Pause interrupts
    while(!Serial.available()); // Wait for user input to resume
    Serial.read();              // Clear entered data
    MsTimer2::start();          // Resume interrupts
  }
  
  
  // Print data over serial for plotting
  if (updated) {
    Serial.print(100*sin(0.005*count));
    Serial.print("\t");
    Serial.print(-100*sin(0.005*count));
    Serial.print("\t");
    Serial.print(100*cos(0.005*count));
    Serial.print("\t");
    Serial.println(-100*cos(0.005*count));
    updated = false;
  }
}
