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
#define P_GAIN          (0.45)  // Proportional gain
#define I_GAIN          (0.002) // Integral gain
#define MAX_I_TERM      (30)    // Maximum Contribution of iTerm in PI controller (percentage of full throttle)
#define COMMAND         (0)     // Commanded/Requested pitch (in degrees from horizontal)

#define ESC_PIN         (3)

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

/* See this link for more info: 
 * https://www.embedded.com/design/prototyping-and-development/4211211/PID-without-a-PhD
 */
typedef struct {
  //double dState;        // Last position input
  double iState;        // Integrator state
  double iMax, iMin;    // Maximum and minimum allowable integrator state
  double  iGain,        // integral gain
          pGain;        // proportional gain
          //dGain;        // derivative gain
} PI_struct;

PI_struct PI_controller; // PI controller for the system
double    command;       // Requested/Commanded pitch
double    pitch;         // Measured pitch
Servo     ESC;           // ESC to drive motors
int       drive;         // Drive signal value fed to ESC

int       counter = 0;   // Counter to toss out preliminary sensor readings during calibration

// Update PI drive signal using new error and position readings
double updatePI(PI_struct *pi, double error, double position) {
  // P and I terms
  double pTerm, iTerm;

  // calculate the proportional term
  pTerm = pi->pGain * error;   
 
  // calculate the integral state with appropriate limiting
  pi->iState += error;
  if (pi->iState > pi->iMax)
    pi->iState = pi->iMax;
  else if (pi->iState < pi->iMin)
    pi->iState = pi->iMin;

  // calculate the integral term
  iTerm = pi->iGain * pi->iState;
  
  // calculate the derivative term
  //dTerm = pid->dGain * (position - pid->dState);

  // update the dState of the controller
  //pid->dState = position;

  // Return new throttle request value (0-100 scale)
  return pTerm + iTerm;
}

// Arm ESC for first use upon startup
void arm(){
  setSpeed(0); // Sets speed variable
  delay(1000); // One second delay for ESC to respond
}

// Drive ESC with 0-1000 drive signal
void setSpeed(int drive){
  // Scale drive signal to ESC's range of accepted values
  int us = map(drive, 0, 1000, 1000, 2000); //Sets servo positions to different speed
  ESC.writeMicroseconds(us);
}

// Change tuning parameters of controller via Serial interface
void tuneController() {
  double old_iGain = PI_controller.iGain;
  
  // Setup PI controller values
  PI_controller.iState = 0; // Reset the integrator state

  Serial.println("\nTune Controller Values:\n");
  
  Serial.print("Select Proportional Gain (Current Value: ");
  Serial.print(PI_controller.pGain);
  Serial.println(")");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  PI_controller.pGain = Serial.parseFloat();  // Set new pGain

  Serial.print("Select Integrator Gain (Current Value: ");
  Serial.print(PI_controller.iGain, 4);
  Serial.println(")");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  PI_controller.iGain = Serial.parseFloat();  // Set new iGain

  Serial.print("Select Minimum Integrator Term Size (Current Value: ");
  Serial.print(PI_controller.iMin * old_iGain);
  Serial.println(")");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  PI_controller.iMin = Serial.parseFloat() / PI_controller.iGain;    // Set new iMin

  Serial.print("Select Maximum Integrator Term Size (Current Value: ");
  Serial.print(PI_controller.iMax * old_iGain);
  Serial.println(")");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  PI_controller.iMax = Serial.parseFloat() / PI_controller.iGain;    // Set new iMax

  Serial.print("Select New Requested Pitch Angle (Current Value: ");
  Serial.print(command);
  Serial.println(")");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  command = Serial.parseFloat();              // Set new command

  // wait for ready
  Serial.println("New values set. Send any character to resume... ");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // FIFO has probably overflowed by this point so we reset it
  mpu.resetFIFO();
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
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

    // Setup PI controller values
    PI_controller.pGain  = P_GAIN;
    PI_controller.iGain  = I_GAIN;
    PI_controller.iState = 0;
    PI_controller.iMin   = -INFINITY;
    PI_controller.iMax   = MAX_I_TERM / I_GAIN;

    // Set requested/commanded pitch
    command = COMMAND;

    // Attach and arm ESC
    Serial.println(F("Initializing ESC..."));
    ESC.attach(ESC_PIN);
    arm();

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
            Serial.read();    // Flush buffer
            setSpeed(0);      // First kill power to the motor(s)
            tuneController(); // Call tuning subroutine
          } else {
            // See if user sent new angle
            double newAngle = Serial.parseFloat();
            // If new angle is within acceptable range, update commanded angle
            if (newAngle > -42 && newAngle < 36) {
              command = newAngle;
            }
          }
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
        Serial.println(drive);

        counter++;

        if (counter > 100) {
          pitch = (ypr[1] * 180/M_PI);
          drive = 10 * updatePI(&PI_controller, command - pitch, pitch);
          setSpeed(drive);
        }

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
