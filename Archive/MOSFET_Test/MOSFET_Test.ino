// Test sketch to test Arduino control of power MOSFET

void setup() {
  Serial.begin(9600);
  pinMode(4, OUTPUT);
}

void loop() {
  // wait for ready
  Serial.println("Send any character to power on... ");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
  digitalWrite(4, HIGH);
  Serial.println("Send any character to power off... ");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
  digitalWrite(4, LOW);
}
