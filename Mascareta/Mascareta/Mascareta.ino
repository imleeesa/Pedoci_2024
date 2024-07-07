#include <SoftwareSerial.h> // Include the SoftwareSerial library for serial communication
#include <Wire.h> // Include the Wire library for I2C communication
#include <VL53L0X.h> // Include the library for the VL53L0X sensor

VL53L0X sensor; // Create an instance of the VL53L0X sensor
VL53L0X sensorFront; // Create an instance of the VL53L0X sensor for the front
VL53L0X sensorBack; // Create an instance of the VL53L0X sensor for the back

float cmFront = 0.00; // Variable to store the front distance in cm
float cmFront2 = 0.00; // Variable to store the second front distance in cm
float cmBack = 0.00; // Variable to store the back distance in cm

unsigned long remataTime = 0; // Variable to store the time of the paddle stroke
unsigned long startTime = 0; // Variable to store the start time
bool frontSensorTriggered = false; // Flag to check if the front sensor was triggered

// Define the pins used
const int RXPin = 2; // Connect to TX of HC05
const int TXPin = 3; // Connect to RX of HC05
SoftwareSerial BTSerial(RXPin, TXPin); // Create a SoftwareSerial object for Bluetooth communication

// Define global variables
int distance2; // Distance calculated by the ultrasonic sensor
unsigned long tempo; // Time variable
long distance = 150; // Maximum reading distance of the ultrasonic sensor
const int ritardo = 200; // Time needed between one command and the next
String msgIn; // Message received from HC05
String msgOut; // Message to send to HC05
String msgOut2; // Response message
char a[100]; // Buffer for serial transmission

// Define the pins used for the motors
int ENA = 5;
int IN1 = 7;
int IN2 = 8;
int ENB = 6;
int IN3 = 10;
int IN4 = 9;

// Define the pins used for the Led
int redOut = A3;
int greenOut = A2;
int blueOut = A0;

// Define the pins XSHUT for the sensors
const int XSHUT1 = A1; // Pin XSHUT for the front sensor
const int XSHUT2 = A2; // Pin XSHUT for the second front sensor
const int XSHUT3 = A0; // Pin XSHUT for the back sensor

void setColor(int red, int green, int blue) { // Function to set the color of an RGB LED
  analogWrite(redOut, red);
  analogWrite(greenOut, green);
  analogWrite(blueOut, blue);
}

void setup() {
  // Initialize the serial port
  BTSerial.begin(38400); // Start Bluetooth serial communication at 38400 baud
  Serial.begin(38400); // Start serial communication at 38400 baud
  Serial.println("Mascareta begin"); // Print a start message
  
  // Set motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Turn off motors - Initial State
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  pinMode(redOut, OUTPUT); // Set RGB LED pins as outputs
  pinMode(greenOut, OUTPUT);
  pinMode(blueOut, OUTPUT);
  
  Wire.begin(); // Start the I2C communication

  // Initialize the sensors
  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  pinMode(XSHUT3, OUTPUT);
  
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  digitalWrite(XSHUT3, LOW);
  delay(10);

  pinMode(XSHUT1, INPUT);
  delay(10);
  sensor.setTimeout(500); // Set timeout for the sensor
  if (!sensor.init()) { // Initialize the sensor
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {} // Halt if initialization fails
  }
  sensor.setAddress(0x30); // Set I2C address for the sensor

  pinMode(XSHUT2, INPUT);
  delay(10);
  sensorFront.setTimeout(500); // Set timeout for the front sensor
  if (!sensorFront.init()) { // Initialize the front sensor
    Serial.println("Failed to detect and initialize sensor front!");
    while (1) {} // Halt if initialization fails
  }
  sensorFront.setAddress(0x31); // Set I2C address for the front sensor

  pinMode(XSHUT3, INPUT);
  delay(10);
  sensorBack.setTimeout(500); // Set timeout for the back sensor
  if (!sensorBack.init()) { // Initialize the back sensor
    Serial.println("Failed to detect and initialize sensor back!");
    while (1) {} // Halt if initialization fails
  }
  sensorBack.setAddress(0x32); // Set I2C address for the back sensor

  // Start continuous measurement for all sensors
  sensor.startContinuous();
  sensorFront.startContinuous();
  sensorBack.startContinuous();

  Serial.println("Started"); // Print a message when all sensors are successfully initialized
}

void loop() {
  // Set LED color to red
  setColor(255, 0, 0);
  
  // Wait for the command to start
  receiver("C");
  
  // Turn off LED
  setColor(0, 0, 0);
  
  // Check and send distance
  checkDistance();
  
  // Stop motors
  stop();
  
  // Turn off LED
  setColor(0, 0, 0);
  
  // Wait for 2 seconds
  delay(2000);
  
  // Turn right for 200ms
  aDestra(200);
  
  // Wait for 600ms
  delay(600);
  
  // Start moving forward
  directionControl(3000);
  
  // Infinite loop
  while (true);
}

void checkDistance() {
  // Function to check distance and react accordingly
  unsigned long startTime = millis(); // Record the start time
  int distance = 10000; // Initialize distance with a high value
  
  while ((distance / 10 >= 150) && (millis() - startTime < 3500)) {
    // Check if distance is above threshold and within time limit
    distance = sensor.readRangeContinuousMillimeters(); // Read distance from front sensor
    
    if (sensor.timeoutOccurred()) {
      Serial.print(" TIMEOUT"); // Print timeout message if sensor times out
    }
    
    cmFront = sensorFront.readRangeContinuousMillimeters() / 10.0; // Read front distance in cm

    // Check if the paddle passes in front of the front sensor
    if (cmFront < 50 && !frontSensorTriggered) {
      cmFront2 = sensorFront.readRangeContinuousMillimeters() / 10.0; // Read second front distance in cm
      delay(200);
      frontSensorTriggered = true; // Set the flag to true
      remataTime = millis(); // Record the time of the paddle stroke
    }

    // Read the back sensor
    cmBack = sensorBack.readRangeContinuousMillimeters() / 10.0; // Read back distance in cm

    // Check if the paddle passes in front of the back sensor within 1 second
    if (frontSensorTriggered) {
      if (cmBack < 50 && (millis() - remataTime <= 1000)) {
        Serial.println("Corro"); // Print a message indicating movement
        directionControl(1000); // Move forward for 1 second
        frontSensorTriggered = false; // Reset the flag
      } else if (millis() - startTime > 1000) {
        frontSensorTriggered = false; // Reset the flag after 1 second
      }
    }

    // Print current readings for debugging
    Serial.print("Anteriore: ");
    Serial.print(cmFront);
    Serial.println(" cm");

    Serial.print("Posteriore: ");
    Serial.print(cmBack);
    Serial.println(" cm");

    delay(100);

    Serial.println(distance / 10); // Print distance in cm
    BTSerial.print(distance / 10); // Send distance over Bluetooth
    BTSerial.print("!"); // Send delimiter
    setColor(255, 0, 0); // Set color to red
  }
}

void receiver(String mess) {
  // Function to receive a specific message over Bluetooth
  String message = "";
  
  while (message != mess) {
    if (BTSerial.available()) {
      // Check if data is available to read
      delay(200); // Wait for the rest of the data to arrive
      char receivedChar = BTSerial.read(); // Read the received character
      
      if (receivedChar == '$') {
        // Check if the character is the start of a message
        message = ""; // Reset message
        
        while (BTSerial.available()) {
          // Read until end of message
          char nextChar = BTSerial.read();
          
          if (nextChar == '#') {
            // Check if the character is the end of a message
            Serial.println("Messaggio ricevuto: " + message); // Print received message
            break;
          }
          
          message += nextChar; // Append character to message
        }
      }
    }
  }
}

void directionControl0() {
  // Function to control motor direction
  // Set maximum power to motors
  analogWrite(ENA, 85); // Right motor power
  analogWrite(ENB, 80); // Left motor power

  // Drive motors A and B forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stop() {
  // Function to stop motors
  // Set motor power to 0
  analogWrite(ENA, 0); // Right motor power off
  analogWrite(ENB, 0); // Left motor power off

  // Turn off all motor pins
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void directionControl(int time) {
  // Function to control motor direction for a specified time
  // Set maximum power to motors
  analogWrite(ENA, 75); // Right motor power
  analogWrite(ENB, 75); // Left motor power

  // Drive motors A and B forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
  delay(time); // Wait for the specified time
  
  // Stop motors
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void aSinistra(int time) {
  // Function to turn left for a specified time
  // Set power to motor A
  analogWrite(ENA, 100); // Right motor power

  // Drive motors to turn left
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
  delay(time); // Wait for the specified time
  
  // Stop motors
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void aDestra(int time) {
  // Function to turn right for a specified time
  // Set maximum power to motors
  analogWrite(ENA, 125); // Right motor power
  analogWrite(ENB, 20); // Left motor power

  // Drive motors A and B forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  delay(time); // Wait for the specified time

  // Stop motors
  analogWrite(ENA, 0); 
  analogWrite(ENB, 0);
}

void sender(String mess) {
  // Function to send a message over Bluetooth
  msgOut = mess; // Set the message to send
  int len = strlen(msgOut.c_str()); // Get the length of the message
  char a[len + 2]; // Create a buffer for the message
  a[0] = '$'; // Start of message delimiter
  
  for (int i = 1; i <= len; i++) {
    a[i] = msgOut.charAt(i - 1); // Copy the message into the buffer
  }
  
  a[len + 1] = '#'; // End of message delimiter
  msgOut = ""; // Reset the message variable
  
  for (int i = 0; i <= len + 1; i++) {
    msgOut = msgOut + a[i]; // Reconstruct the message with delimiters
    BTSerial.println(a[i]); // Send each character of the message
    delay(200); // Delay between sends for stability
  }
  
  Serial.println(msgOut); // Print the sent message
}
