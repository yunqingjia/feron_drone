#include "SoftwareSerial.h"
#include "Servo8Bit.h" 

/* Default Tuning Variables */
#define P_GAIN          (0.1)  // Proportional gain
#define I_GAIN          (0.004) // Integral gain
#define D_GAIN          (0.3) // Derivative gain
#define MIN_I_TERM      (-40)   // Minimum Contribution of iTerm in PI controller (percentage of full throttle)
#define MAX_I_TERM      (40)    // Maximum Contribution of iTerm in PI controller (percentage of full throttle)
#define COMMAND         (0)     // Commanded/Requested pitch (in degrees from horizontal)

/* Input Angle Range */
#define MIN_ANGLE       (-40)
#define MAX_ANGLE       (35)

/* Time Scalar for allowing time/frequency independent operation */
#define TIME_SCALAR     (0.0001)

#define ESC_PIN         (0)   // PWM pin for signaling ESC
#define RX_PIN          (3)   // RX pin for SoftwareSerial
#define TX_PIN          (4)   // TX pin for SoftwareSerial
#define DRIVE_PIN       (1)   // Drive pin for power MOSFET
#define SENSOR_PIN      (3)   // Sensor pin for pitch readings

/* See this link for more info: 
 * https://www.embedded.com/design/prototyping-and-development/4211211/PID-without-a-PhD
 */
typedef struct PID_t {
  double        input;    // Input to controller (requested value)
  unsigned long tState;   // Timestamp of last call to updatePID()
  double        dState;   // Last position input
  double        iState;   // Integrator state
  double        iMax;     // Maximum allowable integrator state 
  double        iMin;     // Minimum allowable integrator state
  double        iGain;    // Integral gain
  double        pGain;    // Proportional gain
  double        dGain;    // Derivative gain
} PID;

// Initialize SoftwareSerial pins
SoftwareSerial  mySerial(RX_PIN, TX_PIN);

PID         controller;        // PID controller for the system
double      pitch;             // Measured pitch
Servo8Bit   ESC;               // ESC to drive motor
int         drive;             // Drive signal value fed to ESC
bool        updateTime = true; // Flag to indicate whether tState should be updated

// ================================================================
// ===                    HELPER FUNCTIONS                      ===
// ================================================================

/* Update PID output signal using current system state */
double updatePID(PID *pid, double currentState) {
  // P, I, & D terms and time factor
  double pTerm, iTerm, dTerm, tFactor;

  // Controller error is difference between input and current state
  double error = pid->input - currentState;

  // Calculate time factor and update tState 
  unsigned long timeStamp = micros();
  tFactor = (timeStamp - pid->tState) * TIME_SCALAR;
  pid->tState = timeStamp;

  // Calculate the proportional term
  pTerm = pid->pGain * error;   
 
  // Calculate the integral state with appropriate min/max constraints
  pid->iState += error;
  pid->iState = constrain(pid->iState, pid->iMin, pid->iMax);

  // Calculate the integral term
  iTerm  = pid->iGain * pid->iState;
  iTerm *= tFactor;
  
  // Calculate the derivative term
  dTerm  = pid->dGain * (currentState - pid->dState);
  dTerm /= tFactor;

  // Update the dState of the controller
  pid->dState = currentState;

  // Return new throttle request value (0-100 scale)
  return pTerm + iTerm - dTerm;
}

/* Arm ESC for first use upon startup */
void arm(Servo8Bit *ESC) {
  setSpeed(ESC, 0); // Set speed variable
  delay(500);       // 500ms delay for ESC to respond
}

/* Callibrate ESC's PWM range for first use */
void callibrate(Servo8Bit *ESC) {
  digitalWrite(DRIVE_PIN, LOW);   // Disconnect ESC from power
  delay(500);                     // Wait 500ms
  setSpeed(ESC, 1000);            // Request full speed
  digitalWrite(DRIVE_PIN, HIGH);  // Reconnect ESC to power
  delay(5000);                     // Wait 5s
  setSpeed(ESC, 0);               // Request 0 speed
}

/* Drive ESC with 0-1000 drive signal */
void setSpeed(Servo8Bit *ESC, int drive) {
  // Scale drive signal to ESC's range of accepted values
  int us = map(drive, 0, 1000, 1000, 2000); //Scale drive signal to ESC's accepted range
  ESC->writeMicroseconds(us);
}

/* Change PID tuning parameters via Serial interface */
/*void tuneController(PID *pid) {
  double old_iGain = pid->iGain;
  
  // Set up PID controller values
  pid->iState = 0;         // Reset the integrator state
  pid->dState = MIN_ANGLE; // Reset the derivative state

  mySerial.print("Select Proportional Gain (Current Value: ");
  mySerial.print(pid->pGain);
  mySerial.println(")");
  while (mySerial.available() && mySerial.read()); // empty buffer
  while (!mySerial.available());                 // wait for data
  pid->pGain = mySerial.parseFloat();            // Set new pGain

  mySerial.print("Select Integrator Gain (Current Value: ");
  mySerial.print(pid->iGain, 4);
  mySerial.println(")");
  while (mySerial.available() && mySerial.read()); // empty buffer
  while (!mySerial.available());                 // wait for data
  pid->iGain = mySerial.parseFloat();            // Set new iGain

  mySerial.print("Select Derivative Gain (Current Value: ");
  mySerial.print(pid->dGain, 4);
  mySerial.println(")");
  while (mySerial.available() && mySerial.read()); // empty buffer
  while (!mySerial.available());                 // wait for data
  pid->dGain = mySerial.parseFloat();            // Set new dGain

  mySerial.print("Select Minimum Integrator Term Size (Current Value: ");
  mySerial.print(pid->iMin * old_iGain);
  mySerial.println(")");
  while (mySerial.available() && mySerial.read());  // empty buffer
  while (!mySerial.available());                  // wait for data
  pid->iMin = mySerial.parseFloat() / pid->iGain; // Set new iMin

  mySerial.print("Select Maximum Integrator Term Size (Current Value: ");
  mySerial.print(pid->iMax * old_iGain);
  mySerial.println(")");
  while (mySerial.available() && mySerial.read());  // empty buffer
  while (!mySerial.available());                  // wait for data
  pid->iMax = mySerial.parseFloat() / pid->iGain; // Set new iMax

  mySerial.print("Select New Controller Input (Current Value: ");
  mySerial.print(pid->input);
  mySerial.println(")");
  while (mySerial.available() && mySerial.read()); // empty buffer
  while (!mySerial.available());                 // wait for data
  pid->input = mySerial.parseFloat();            // Set new command

  // wait for ready
  mySerial.println("New values set. Send any character to resume... ");
  while (mySerial.available() && mySerial.read()); // empty buffer
  while (!mySerial.available());                 // wait for data
  while (mySerial.available() && mySerial.read()); // empty buffer again

  // Update tState of controller to reflect time elapsed
  updateTime = true;
}*/

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // Initialize serial communication
  mySerial.begin(9600);
  while (!mySerial); // wait for Leonardo enumeration, others continue immediately

  // Configure I/O pins
  pinMode(DRIVE_PIN, OUTPUT);

  // Setup PI controller values
  controller.input = COMMAND;
  controller.tState = 0;
  controller.pGain  = P_GAIN;
  controller.iGain  = I_GAIN;
  controller.dGain  = D_GAIN;
  controller.iState = 0;
  controller.dState = MIN_ANGLE;
  controller.iMin   = MIN_I_TERM / I_GAIN;
  controller.iMax   = MAX_I_TERM / I_GAIN;

  // TODO: Remove this for final build
  mySerial.println(F("\nSend any character to callibrate ESC: "));
  while (mySerial.available() && mySerial.read()); // empty buffer
  while (!mySerial.available());                 // wait for data
  while (mySerial.available() && mySerial.read()); // empty buffer again

  // Attach, callibrate, and arm ESC
  mySerial.println(F("Initializing ESC..."));
  ESC.attach(ESC_PIN);
  callibrate(&ESC);   // TODO: Enable this once sufficient test has occurred
  //arm(&ESC);

  // TODO: Remove this for finalized controller
  mySerial.println(F("\nEntering Infinite Loop"));
  while(1);

  // wait for ready
  mySerial.println(F("\nSend any character to begin: "));
  while (mySerial.available() && mySerial.read()); // empty buffer
  while (!mySerial.available());                 // wait for data
  while (mySerial.available() && mySerial.read()); // empty buffer again
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // See if user wants to tune controller or change commanded angle
  if (mySerial.available()) {
    if (mySerial.peek() == ' ') {
      mySerial.read();                // Flush buffer
      setSpeed(&ESC, 0);            // Kill power to the motor(s)
      //tuneController(&controller);  // Call tuning subroutine
    } else {
      // See if user sent new input
      double newAngle = mySerial.parseFloat();
      
      // If new angle is within acceptable range, update input angle
      if (newAngle >= MIN_ANGLE && newAngle <= MAX_ANGLE) {
        controller.input = newAngle;
      }
    }
  }
   
  // TODO: Maybe make updateTime a member of PID struct?

  // Check if tState needs to be updated
  if (updateTime) {
    updateTime = false;
    controller.tState = micros();
  }
        
  // Read and display rotary sensor value
  analogRead(SENSOR_PIN);
  mySerial.print("Pitch:\t");
  mySerial.print(pitch);
  mySerial.print("\t");
  mySerial.print("Drive:\t");
  mySerial.println(drive);

  // Update PID controller with new pitch reading
  drive = 10 * updatePID(&controller, pitch);
  setSpeed(&ESC, drive);

}
