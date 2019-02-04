// Simple program to test the Arduino's
// analogRead of the rotary position sensor 

#define SENSOR_PIN    (A0)
#define BUFFER_SIZE   (2)

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

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
}

void loop() {
  sensorVal = analogRead(SENSOR_PIN);
  Serial.print("Sensor Reading: ");
  Serial.print(sensorVal);
  Serial.print("\tFiltered Angle Reading: ");
  Serial.println(filter(-0.3656*sensorVal+185.64), 2);  // In Emily's code, the filter variable was made negative, i.e.
  // FROM EMILY'S CODE (Just add a negative) Serial.println(-filter(-0.3656*sensorVal+185.64), 2);
}
