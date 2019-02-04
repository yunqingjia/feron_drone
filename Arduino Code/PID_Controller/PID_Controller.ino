#include "Servo.h"
#include "MsTimer2.h"

/* Default Tuning Variables */
#define P_GAIN          (3)     // Proportional gain    // 2.3
#define I_GAIN          (1.5)   // Integral gain        // 4.8
#define D_GAIN          (0.4)   // Derivative gain      // 0.4
#define MIN_I_TERM      (-250)  // Minimum Contribution of iTerm in PI controller
#define MAX_I_TERM      (250)   // Maximum Contribution of iTerm in PI controller
#define COMMAND         (0)     // Commanded/Requested pitch (in degrees from horizontal)
#define FREQUENCY       (100)   // Refresh rate of controlller (in Hz)

/* Hardware Restrictions */
#define MIN_ANGLE       (-65)
#define MAX_ANGLE       (30)
#define MAX_FREQ        (1000) // Maximum refresh rate (in Hz)

/* Pin Numbers */
#define ESC_PIN         (6)   // PWM pin for signaling ESC
#define DRIVE_PIN       (10)  // Drive pin for power MOSFET
#define SENSOR_PIN      (A0)  // Pin for reading sensor values

/* Filter buffer size */
#define BUFFER_SIZE     (2)

/* Controller struct */
typedef struct PID_t {
  float   input;      // Input to controller (requested value)
  float   Ki;         // Integral gain
  float   Kp;         // Proportional gain
  float   Kd;         // Derivative gain
  float   dt;         // Period between updates (in seconds)
  float   old_error;  // Last error value
  float   iState;     // Integrator state (accumulated error)
} PID;

volatile PID controller;  // PID controller for the system
volatile float pitch;     // Measured pitch
Servo ESC;                // ESC to drive motor
volatile int drive;       // Drive signal value fed to ESC

volatile bool updatedPID; // Flag to indicate whenever controller is updated

float filterBuffer[BUFFER_SIZE];  // Array for moving average filter
float filteredVal;                // Current filtered valued
int index;                        // Current index of filterBuffer

// ================================================================
// ===                    HELPER FUNCTIONS                      ===
// ================================================================

/*
 * Lowpass moving average filter to smooth analog sensor readings
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

/* 
 * Reset the state of controller, filter, and pitch reading
 * Should be invoked whenever the system is stopped or paused
 */
void resetSystem() {
  // Reset drive signal
  drive = 0;

  // Reset updatedPID flag
  updatedPID = false;
  
  // Reset and refill filter buffer to avoid throttle spikes
  for (int i = 0; i < BUFFER_SIZE; i++)
    // FROM EMILY'S CODE (it was also commented out: "//-filter(-0.3656*sensorVal+185.64)"
    pitch = filter((-0.3656 * analogRead(SENSOR_PIN)) + 185.64);

  // Reset the integrator state
  controller.iState = 0;

  // Re-initialize the derivative state to prevent derivative spikes
  controller.old_error = controller.input - pitch;
}

/*
 * Update PID output signal using current system state
 */
void updatePID() {
  // P, I, & D terms
  float pTerm, iTerm, dTerm;

  // Measure and filter rotary sensor value
  pitch = filter((-0.3656 * analogRead(SENSOR_PIN)) + 185.64);  // In EMILY'S CODE THIS WAS NEGATIVE (before filter, just add a negative)
  
  // Controller error is difference between input and current state
  float error = controller.input - pitch;

  // Calculate the proportional term
  pTerm = controller.Kp * error;

  // Calculate the integral state with appropriate min/max constraints
  controller.iState += error * controller.dt;
  controller.iState = constrain(controller.iState, MIN_I_TERM/controller.Ki, MAX_I_TERM/controller.Ki);

  // Calculate the integral term
  iTerm  = controller.Ki * controller.iState;

  // Calculate the derivative term
  dTerm  = controller.Kd * ((error - controller.old_error)/controller.dt);

  // Update the dState of the controller
  controller.old_error = error;

  // Add PID terms to get new drive signal (0-1000 scale)
  drive = pTerm + iTerm + dTerm;

  // Send new drive signal to ESC
  setSpeed(&ESC, drive);

  // Set updatedPID flag
  updatedPID = true;
}

/*
 * Arm ESC for first use upon startup
 */
void arm(Servo *ESC) {
  Serial.print("Arming ESC... ");
  digitalWrite(DRIVE_PIN, LOW);   // Disconnect ESC from power
  delay(500);                     // Wait 500ms for ESC to power down
  setSpeed(ESC, 0);               // Set speed to 0
  digitalWrite(DRIVE_PIN, HIGH);  // Reconnect ESC to power
  delay(2500);                    // 2.5 second delay for ESC to respond
  Serial.println("Arming complete");
}

/*
 * Calibrate ESC's PWM range for first use
 */
void calibrate(Servo *ESC) {
  Serial.print("Calibrating ESC... ");
  digitalWrite(DRIVE_PIN, LOW);   // Disconnect ESC from power
  delay(500);                     // Wait 500ms
  setSpeed(ESC, 1000);            // Request full speed
  digitalWrite(DRIVE_PIN, HIGH);  // Reconnect ESC to power
  delay(5000);                    // Wait 5 seconds
  setSpeed(ESC, 0);               // Request 0 speed
  delay(8000);                    // Wait 8 seconds
  Serial.println("Calibration complete");
}

/*
 * Drive ESC with 0-1000 drive signal
 */
void setSpeed(Servo *ESC, int drive) {
  // Scale drive signal to ESC's range of accepted values
  int us = map(drive, 0, 1000, 1000, 2000); //Scale drive signal to ESC's accepted range
  ESC->writeMicroseconds(us);
}

/*
 * Change PID tuning parameters via Serial interface
 */
void tuneController(volatile PID *pid) {
  Serial.print("Set Proportional Gain (Current Value: ");
  Serial.print(pid->Kp);
  Serial.println(")"); 
  while (Serial.available() && Serial.read());  // empty buffer
  while (!Serial.available());                  // wait for data
  pid->Kp = Serial.parseFloat();                // Set new proportial gain

  Serial.print("Set Integrator Gain (Current Value: ");
  Serial.print(pid->Ki);
  Serial.println(")");
  while (Serial.available() && Serial.read());  // empty buffer
  while (!Serial.available());                  // wait for data
  pid->Ki = Serial.parseFloat();                // Set new integral gain

  Serial.print("Set Derivative Gain (Current Value: ");
  Serial.print(pid->Kd);
  Serial.println(")");
  while (Serial.available() && Serial.read());  // empty buffer
  while (!Serial.available());                  // wait for data
  pid->Kd = Serial.parseFloat();                // Set new derivative gain

  // Wait for ready
  Serial.println("New values set. Send any character to resume...");
  while (Serial.available() && Serial.read());  // empty buffer
  while (!Serial.available());                  // wait for data
  while (Serial.available() && Serial.read());  // empty buffer again
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Attach ESC
  ESC.attach(ESC_PIN);

  // Configure I/O pins
  pinMode(DRIVE_PIN, OUTPUT);
  digitalWrite(DRIVE_PIN, LOW);

  // Setup initial PID controller values
  controller.input      = COMMAND;
  controller.Kp         = P_GAIN;
  controller.Ki         = I_GAIN;
  controller.Kd         = D_GAIN;
  controller.dt         = 1.0 / FREQUENCY; // period = 1/frequency
    
  Serial.println("Send any character to arm ESC or 'c' to calibrate...");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  
  if (Serial.read() == 'c') {
    // Only calibrate ESC if user opted for it
    calibrate(&ESC);
  } else {
    // Otherwise, only arm ESC
    arm(&ESC);
  }

  // Wait for ready
  Serial.println("\nSend any character to begin...");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // Reset system parameters before starting to avoid unpredictable behavior 
  resetSystem();

  // Attach ISR for timer interrupt
  MsTimer2::set(1000/FREQUENCY, updatePID); // period (ms) = 1,000/frequency
  MsTimer2::start();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // Check for user input
  if (Serial.available()) {
    // 'p' for pause
    if (Serial.peek() == 'p') {
      MsTimer2::stop();   // Disable interrupts
      Serial.read();      // Flush buffer
      setSpeed(&ESC, 0);  // Kill power to the motor(s)
      
      // wait for ready
      Serial.println("Send any character to resume...");
      while (Serial.available() && Serial.read()); // empty buffer
      while (!Serial.available());                 // wait for data
      while (Serial.available() && Serial.read()); // empty buffer again

      // Reset system parameters before resuming to avoid unpredictable behavior 
      resetSystem();

      // Re-enable interrupts and continue
      MsTimer2::start();
    }
    // 't' for tune
    else if (Serial.peek() == 't') {
      MsTimer2::stop();   // Disable interrupts
      Serial.read();      // Flush buffer
      setSpeed(&ESC, 0);  // Kill power to the motor(s)

      // Call tuning subroutine
      tuneController(&controller);

      // Reset system parameters before resuming to avoid unpredictable behavior 
      resetSystem();

      // Re-enable interrupts and continue
      MsTimer2::start();
    }
    // 'c' for calibrate
    else if (Serial.peek() == 'c') {
      MsTimer2::stop();   // Disable interrupts
      Serial.read();      // Flush buffer
      setSpeed(&ESC, 0);  // Kill power to the motor(s)

      // Call calibration subroutine
      calibrate(&ESC);

      // wait for ready
      Serial.println("Send any character to resume...");
      while (Serial.available() && Serial.read()); // empty buffer
      while (!Serial.available());                 // wait for data
      while (Serial.available() && Serial.read()); // empty buffer again

      // Reset system parameters before resuming to avoid unpredictable behavior 
      resetSystem();

      // Re-enable interrupts and continue
      MsTimer2::start();
    }
    // 'f' for frequency
    else if (Serial.peek() == 'f') {
      MsTimer2::stop();   // Disable interrupts
      Serial.read();      // Flush buffer
      setSpeed(&ESC, 0);  // Kill power to the motor(s)

      // Ask user to set new refresh rate
      Serial.print("Set new refresh rate in Hz (1-");
      Serial.print(MAX_FREQ);
      Serial.println(")");
      while (Serial.available() && Serial.read());  // empty buffer
      while (!Serial.available());                  // wait for data
      int newFreq = Serial.parseInt();              // Read user input
      
      // Check for valid new frequency
      if (newFreq > 0 && newFreq <= MAX_FREQ) {
        controller.dt = 1.0 / newFreq;            // Set new controller dt
        MsTimer2::set(1000 / newFreq, updatePID); // Set new interrupt period (in milliseconds)
      }
      
      // wait for ready
      Serial.println("Send any character to resume...");
      while (Serial.available() && Serial.read()); // empty buffer
      while (!Serial.available());                 // wait for data
      while (Serial.available() && Serial.read()); // empty buffer again

      // Reset system parameters before resuming to avoid unpredictable behavior 
      resetSystem();

      // Re-enable interrupts and continue
      MsTimer2::start();
    }
    // Otherwise, treat input as new pitch request
    else {
      // See if user sent new pitch request
      float newAngle = Serial.parseFloat();

      // If new angle is within acceptable range, update input angle
      if (newAngle >= MIN_ANGLE && newAngle <= MAX_ANGLE) {
        MsTimer2::stop();   // Disable interrupts
        controller.old_error = newAngle - pitch;
        controller.input = newAngle;
        MsTimer2::start();  // Re-enable interrupts and continue
      }
    }
  }

  // Print pitch and drive info to serial after PID updates
  if (updatedPID) {
    Serial.print(pitch, 2);
    Serial.print("\t");
    Serial.print(drive);
    Serial.print("\t");
    Serial.println(controller.input);
    updatedPID = false;
  }
}
