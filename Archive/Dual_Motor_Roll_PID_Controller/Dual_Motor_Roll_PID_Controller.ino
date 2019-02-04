#include "I2Cdev.h"
#include "Servo.h" 

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

/* Default Tuning Variables */
#define P_GAIN          (0.001)  // Proportional gain
#define I_GAIN          (0.0001) // Integral gain
#define D_GAIN          (0.00)   // Derivative gain
#define MIN_I_TERM      (-0.4)   // Minimum Contribution of iTerm in PI controller (percentage of full throttle)
#define MAX_I_TERM      (0.4)    // Maximum Contribution of iTerm in PI controller (percentage of full throttle)
#define COMMAND         (0)      // Commanded/Requested pitch (in degrees from horizontal)

/* Total throttle to be shared between two motors */
#define THROTTLE        (300)

/* Input Angle Range (Roll) */
#define MIN_ANGLE       (-90)
#define MAX_ANGLE       (90)

/* Time Scalar for allowing time/frequency independent operation */
#define TIME_SCALAR     (0.0001)

#define ESC1_PIN        (3)  // PWM pin for signaling ESC1
#define ESC2_PIN        (5)  // PWM pin for signaling ESC2
#define INTERRUPT_PIN   (2)  // Use pin 2 on Arduino Uno & most boards
#define LED_PIN         (13) // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU6050 Sensor
MPU6050 mpu;

// MPU control/status vars
bool     dmpReady = false; // set true if DMP init was successful
uint8_t  mpuIntStatus;     // holds actual interrupt status byte from MPU
uint8_t  devStatus;        // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;       // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;        // count of all bytes currently in FIFO
uint8_t  fifoBuffer[64];   // FIFO storage buffer

// orientation/motion vars
Quaternion  q;          // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float       euler[3];   // [psi, theta, phi]    Euler angle container
float       ypr[3];     // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

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

PID    controller;        // PID controller for the system
double roll;              // Measured roll
Servo  ESC1, ESC2;        // ESCs to drive motors
int    throttle;          // Total throttle to be split between two motors
double proportion = 0.5;        // Proportion of total throttle to drive ESC1 (ESC2 gets remainder)
int    counter = 0;       // Counter to ignore preliminary sensor readings during calibration
bool   updateTime = true; // Flag to indicate whether tState should be updated

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
 
  // Calculate the integral state with appropriate limiting
  pid->iState += error;
  if (pid->iState > pid->iMax)
    pid->iState = pid->iMax;
  else if (pid->iState < pid->iMin)
    pid->iState = pid->iMin;

  // Calculate the integral term
  iTerm  = pid->iGain * pid->iState;
  iTerm *= tFactor;
  
  // Calculate the derivative term
  dTerm  = pid->dGain * (currentState - pid->dState);
  dTerm /= tFactor;

  // Update the dState of the controller
  pid->dState = currentState;

  // Return new throttle request value (0-100 scale)
  return 0.5 + pTerm + iTerm - dTerm;
}

/* Arm ESC for first use upon startup */
void arm(Servo *ESC){
  setSpeed(ESC, 0); // Set speed variable
  delay(500);       // 500ms delay for ESC to respond
}

/* Drive ESC with 0-1000 drive signal */
void setSpeed(Servo *ESC, int drive){
  // Scale drive signal to ESC's range of accepted values
  int us = map(drive, 0, 1000, 1000, 2000); //Scale drive signal to ESC's accepted range
  ESC->writeMicroseconds(us);
}

/* Change PID tuning parameters via Serial interface */
void tuneController(PID *pid) {
  double old_iGain = pid->iGain;
  
  // Set up PID controller values
  pid->iState = 0;         // Reset the integrator state
  pid->dState = MIN_ANGLE; // Reset the derivative state

  Serial.print("Select Proportional Gain (Current Value: ");
  Serial.print(pid->pGain);
  Serial.println(")");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  pid->pGain = Serial.parseFloat();            // Set new pGain

  Serial.print("Select Integrator Gain (Current Value: ");
  Serial.print(pid->iGain, 4);
  Serial.println(")");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  pid->iGain = Serial.parseFloat();            // Set new iGain

  Serial.print("Select Derivative Gain (Current Value: ");
  Serial.print(pid->dGain, 4);
  Serial.println(")");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  pid->dGain = Serial.parseFloat();            // Set new dGain

  Serial.print("Select Minimum Integrator Term Size (Current Value: ");
  Serial.print(pid->iMin * old_iGain);
  Serial.println(")");
  while (Serial.available() && Serial.read());  // empty buffer
  while (!Serial.available());                  // wait for data
  pid->iMin = Serial.parseFloat() / pid->iGain; // Set new iMin

  Serial.print("Select Maximum Integrator Term Size (Current Value: ");
  Serial.print(pid->iMax * old_iGain);
  Serial.println(")");
  while (Serial.available() && Serial.read());  // empty buffer
  while (!Serial.available());                  // wait for data
  pid->iMax = Serial.parseFloat() / pid->iGain; // Set new iMax

  Serial.print("Select New Total Throttle (Current Value: ");
  Serial.print(throttle);
  Serial.println(")");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  throttle = Serial.parseInt();                // Set new throttle

  Serial.print("Select New Controller Input (Current Value: ");
  Serial.print(pid->input);
  Serial.println(")");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  pid->input = Serial.parseFloat();            // Set new command

  // wait for ready
  Serial.println("New values set. Send any character to resume... ");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // FIFO has probably overflowed by this point so we reset it
  mpu.resetFIFO();

  // Update tState of controller to reflect time elapsed
  updateTime = true;
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

// indicates whether MPU interrupt pin has gone high
volatile bool mpuInterrupt = false;

// MPU Interrupt Service Routine
void dmpDataReady() {
    mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // Set total throttle to be shared between two motors
    throttle = THROTTLE;

    // Setup PI controller values
    controller.input  = COMMAND;
    controller.tState = 0;
    controller.pGain  = P_GAIN;
    controller.iGain  = I_GAIN;
    controller.dGain  = D_GAIN;
    controller.iState = 0;
    controller.dState = MIN_ANGLE;
    controller.iMin   = MIN_I_TERM / I_GAIN;
    controller.iMax   = MAX_I_TERM / I_GAIN;

    // Attach and arm ESCs
    Serial.println(F("Initializing ESCs..."));
    ESC1.attach(ESC1_PIN);
    ESC2.attach(ESC2_PIN);
    arm(&ESC1);
    arm(&ESC2);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize);

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        
        // See if user wants to tune controller or change commanded angle
        if (Serial.available()) {
          if (Serial.peek() == ' ') {
            Serial.read();                // Flush buffer
            setSpeed(&ESC1, 0);           // Kill power to motor 1
            setSpeed(&ESC2, 0);           // Kill power to motor 2
            tuneController(&controller);  // Call tuning subroutine
          } else {
            // See if user sent new input
            double newAngle = Serial.parseFloat();
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
        
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[2] * 180/M_PI);
        Serial.print("\t");
        Serial.print(constrain(1 - proportion, 0, 1) * throttle);
        Serial.print("\t");
        Serial.println(constrain(proportion, 0, 1) * throttle);

        if (counter < 100) {
          counter++;
        } else {
          roll = (ypr[2] * 180/M_PI);
          proportion = updatePID(&controller, roll);
          setSpeed(&ESC1, constrain(1 - proportion, 0, 1) * throttle);
          setSpeed(&ESC2, constrain(proportion, 0, 1) * throttle);
        }

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
