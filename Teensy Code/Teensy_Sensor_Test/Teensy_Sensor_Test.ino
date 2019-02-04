// Simple program to test the Teensy's
// analogRead of the rotary position sensor 

int sensorVal;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  analogReadResolution(13);
}

void loop() {
  sensorVal = analogRead(A0);
  Serial.printf("Sensor Reading: %d\t", sensorVal);
  Serial.printf("Angle Reading: %f\n", (0.0457*sensorVal)-185.64); 
}
