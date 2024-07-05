
/*
  Included Libraries:

  PID_v1.h: Library for PID control.
  LMotorController.h: Custom library for motor control.
  I2Cdev.h: Library for I2C communication.
  MPU6050_6Axis_MotionApps20.h: Library for interfacing with the MPU6050 sensor.
  Conditional Compilation:

  Includes the Wire library if the I2C implementation is set to Arduino Wire.
  Defines:

  LOG_INPUT: A flag for logging input data (true/false).
  MIN_ABS_SPEED: Minimum speed for the motors.
*/
#include <SPI.h>
#include <MFRC522.h>
#include "PID_v1.h"
#include "LMotorController.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define LOG_INPUT true
#define MIN_ABS_SPEED 20



#define SS_PIN 10
#define RST_PIN 9
MFRC522 rfid(SS_PIN, RST_PIN);

double inputSav;

const double GradeAdjust = 4.15; // se va a destra bisogna diminuire il valore
const double motorSpeedFactorLeft = 0.65;
const double motorSpeedFactorRight = 0.75;

/*
   MPU6050 Related Variables:

  Instances and status variables for handling the MPU6050 sensor and its Digital Motion Processor (DMP).
  Orientation/Motion Variables:

  Quaternion, VectorFloat, and ypr for storing orientation data.
  PID Controller Variables:

  input, output, setpoint, Kp, Ki, Kd, and the PID instance for PID control.
  Motor Controller Variables:

  motorSpeedFactorLeft, motorSpeedFactorRight to balance motor speeds.
  ENA, IN1, IN2, IN3, IN4, ENB for motor controller pins.
  Instance of LMotorController.
  Timer Variables:

  Various timing variables for controlling the program flow.
  mpuInterrupt is a flag set by the interrupt service routine dmpDataReady().
*/
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];

// PID
double originalSetpoint = 0;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
int temp = 0;
double input, output;
int moveState = 0;
double Kp = 180;   //
double Kd = 13;
double Ki = 15;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);


// MOTOR CONTROLLER
int ENA = 6;
int IN1 = 8;
int IN2 = 7;
int IN3 = 3;
int IN4 = 4 ;
int ENB = 5;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

// timers
long time1Hz = 0;
long time5Hz = 0;
unsigned long timeStart = 0;
unsigned long timeChange = 0;
unsigned long timeChange2 = 0;
unsigned long timeyhange = 0;
unsigned long timeStep = 0;
unsigned long timeInterval = 0;
volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}



void setup()
{

  SPI.begin();
  rfid.PCD_Init();
  /*
    I2C Setup:

    Initializes the I2C bus depending on the implementation.
    Serial Communication:

    Sets up serial communication at 115200 baud rate.
    MPU6050 Initialization:

    Initializes the MPU6050 and configures the DMP.
    Sets gyro and accelerometer offsets.
    Waits for 8 seconds to allow stabilization.
    Checks DMP initialization status and sets up interrupts if successful.
    Configures the PID controller with automatic mode, sample time, and output limits.
    Pin Modes(LED SETUP):
    Sets up the pin modes for the Arduino pins A1, A2, and A3.
  */
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif



  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  //Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  //Serial.println(F("Testing device connections..."));
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  timeyhange = millis();

  // while (millis() - timeyhange < 8000) {}
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    //setup PID
    timeyhange = millis();
    while (millis() - timeyhange < 5000) {}
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
  }
  pinMode(A1, OUTPUT); // rosso
  pinMode(A2, OUTPUT); // blu
  pinMode(A0, OUTPUT); // verde
 }


void loop()
{

  /*
     Phase 1:
    Activates specific pins to initiate the first phase.
    Runs for 15 seconds, continuously computing PID values and controlling the motors based on sensor input.

    Phase 2:
    Switches pin states to indicate phase change.
    Runs for 30 seconds with similar PID control.
    Includes a routine to move the motors in steps every 9 seconds.

    Phase 3:
    Activates different pin states for the third phase.
    Enters an infinite loop performing continuous PID computation and motor control.
  */

  Serial.println("fase1");  // Print "fase1" to indicate the beginning of the first phase
  analogWrite(A1, 255);    // Set A1 to high (255) for the first phase
  analogWrite(A2, 0);      // Set A2 to low (0) for the first phase
  analogWrite(A0, 0);      // Set A3 to low (0) for the first phase
  timeStart = millis();  // Record the start time for phase 1
  timeChange = millis();
  bool flag = true;
  // Loop for 15 seconds
  while (flag) {
    if (!dmpReady) return; // If DMP is not ready, exit the loop

    // PID computation and motor control until an interrupt is detected
    while (!mpuInterrupt && fifoCount < packetSize) {
      pid.Compute();    // Compute the PID output
      motorController.move(output, MIN_ABS_SPEED); // Move the motors based on PID output
    }

    mpuInterrupt = false;   // Reset the interrupt flag
    mpuIntStatus = mpu.getIntStatus();  // Get the interrupt status
    fifoCount = mpu.getFIFOCount();     // Get the FIFO count

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) { // Check for FIFO overflow
      mpu.resetFIFO();     // Reset FIFO if overflow occurred
    } else if (mpuIntStatus & 0x02) {   // Check if data is available
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount(); // Wait until enough data
      mpu.getFIFOBytes(fifoBuffer, packetSize); // Get FIFO data
      fifoCount -= packetSize;    // Decrease FIFO count by packet size
      mpu.dmpGetQuaternion(&q, fifoBuffer);  // Get quaternion data
      mpu.dmpGetGravity(&gravity, &q);       // Get gravity vector
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // Get yaw, pitch, roll
      input = ypr[1] * 180 / M_PI - GradeAdjust;       // Convert pitch to degrees and adjust
    }

    //delay(2);  // Small delay to allow the system to update
    //Serial.println("prima");

    if (millis() - timeChange > 2000) {
      timeChange = millis();
      Serial.println("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
      if (rfid.PICC_IsNewCardPresent()) {
        Serial.println("dopo");
        flag = false;
      }
      mpuInterrupt = false;     // Reset interrupt flag
      mpu.resetFIFO();          // Reset FIFO
    }
  }

  Serial.println("fase2");  // Print "fase2" to indicate the beginning of the second phase
  analogWrite(A1, 0);    // Set A1 to low (0) for the second phase
  analogWrite(A2, 255);  // Set A2 to high (255) for the second phase
  analogWrite(A0, 0);    // Set A3 to low (0) for the second phase
  timeStart = millis();  // Record the start time for phase 2
  timeChange = millis(); // Record the time for phase change
  timeStep = 5000;
  timeInterval = 1000;
  mpu.resetFIFO();       // Reset FIFO

  // Loop for 30 seconds
  bool balanced = true;
  while (millis() - timeStart < 25000) {
    if (!dmpReady) return; // If DMP is not ready, exit the loop

    // PID computation and motor control until an interrupt is detected
    while (!mpuInterrupt && fifoCount < packetSize) {
      pid.Compute();    // Compute the PID output
      motorController.move(output, MIN_ABS_SPEED); // Move the motors based on PID output
      Serial.print("Qui = ");
      Serial.println(input);
    }

    mpuInterrupt = false;   // Reset the interrupt flag
    mpuIntStatus = mpu.getIntStatus();  // Get the interrupt status
    fifoCount = mpu.getFIFOCount();     // Get the FIFO count

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) { // Check for FIFO overflow
      mpu.resetFIFO();     // Reset FIFO if overflow occurred
    } else if (mpuIntStatus & 0x02) {   // Check if data is available
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount(); // Wait until enough data
      mpu.getFIFOBytes(fifoBuffer, packetSize); // Get FIFO data
      fifoCount -= packetSize;    // Decrease FIFO count by packet size
      mpu.dmpGetQuaternion(&q, fifoBuffer);  // Get quaternion data
      mpu.dmpGetGravity(&gravity, &q);       // Get gravity vector
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // Get yaw, pitch, roll
      input = ypr[1] * 180 / M_PI - GradeAdjust - 0.8;       // Convert pitch to degrees and adjust
      if (!balanced) {
        input = input + 0.5;
        Serial.print("nuovo = ");
        Serial.println(input);
      }
    }

    // Move motors in a specific pattern every 9 seconds
    if (millis() - timeChange < timeInterval) {
      balanced = false;
      Serial.println("VERDEEEEEEEEEEEEEEEEEEE");
      analogWrite(A1, 0);    // Set A1 to low (0) for the second phase
      analogWrite(A2, 0);  // Set A2 to high (255) for the second phase
      analogWrite(A0, 255);    // Set A3 to low (0) for the second phase
    }
    else {
      balanced = true;
      if (millis() - timeChange > timeStep) {
        Serial.println("Rossoooooooooooooooooo");
        timeChange = millis();
        analogWrite(A1, 255);    // Set A1 to low (0) for the second phase
        analogWrite(A2, 0);  // Set A2 to high (255) for the second phase
        analogWrite(A0, 0);    // Set A3 to low (0) for the second phase
      }
    }
    /*
      int j = 50;
      for (int i = 1; i <= 40; i++) {   // Move motors in sequence
      motorController.move(j, j);
      delay(20);
      motorController.move(j, j);
      delay(20);
      }

      //delay(1000);
    */

  }

  Serial.println("fase3");  // Print "fase3" to indicate the beginning of the third phase
  analogWrite(A1, 0);    // Set A1 to low (0) for the third phase
  analogWrite(A2, 0);    // Set A2 to low (0) for the third phase
  analogWrite(A0, 255);  // Set A3 to high (255) for the third phase
  mpu.resetFIFO();       // Reset FIFO
  timeStart = millis();  // Record the start time for phase 3

  // Enter an infinite loop for continuous operation
  while (true) {
    if (!dmpReady) return; // If DMP is not ready, exit the loop

    // PID computation and motor control until an interrupt is detected
    while (!mpuInterrupt && fifoCount < packetSize) {
      pid.Compute();    // Compute the PID output
      motorController.move(output, MIN_ABS_SPEED); // Move the motors based on PID output
    }

    mpuInterrupt = false;   // Reset the interrupt flag
    mpuIntStatus = mpu.getIntStatus();  // Get the interrupt status
    fifoCount = mpu.getFIFOCount();     // Get the FIFO count

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) { // Check for FIFO overflow
      mpu.resetFIFO();     // Reset FIFO if overflow occurred
    } else if (mpuIntStatus & 0x02) {   // Check if data is available
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount(); // Wait until enough data
      mpu.getFIFOBytes(fifoBuffer, packetSize); // Get FIFO data
      fifoCount -= packetSize;    // Decrease FIFO count by packet size
      mpu.dmpGetQuaternion(&q, fifoBuffer);  // Get quaternion data
      mpu.dmpGetGravity(&gravity, &q);       // Get gravity vector
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // Get yaw, pitch, roll
      input = ypr[1] * 180 / M_PI - GradeAdjust;       // Convert pitch to degrees and adjust
    }

    delay(2);  // Small delay to allow the system to update
  }
}
