#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <Servo.h>

const int servoPin = 9;  // Pin to which the servo is connected
float dist;  // Variable to store distance measured by ultrasonic sensor
Servo myServo;  // Create a Servo object
unsigned long startTime;  // Variable to store the start time

const int fountain = 5;  // Pin to control the fountain

// Define RX and TX pins for SoftwareSerial
SoftwareSerial ss(7, 6); // RX, TX
DFRobotDFPlayerMini mp3;  // Create a DFRobotDFPlayerMini object

const int trigger = 4;  // Pin for ultrasonic sensor trigger
const int echo = 3;  // Pin for ultrasonic sensor echo

int nfiles;  // Variable to store number of files
int i = 0;  // Counter variable
int dist = 120;  // Initial distance value

void setup() {
  ss.begin(9600);  // Begin software serial at 9600 baud
  Serial.begin(9600);  // Begin hardware serial at 9600 baud
  pinMode(fountain, OUTPUT);  // Set fountain pin as output
  pinMode(trigger, OUTPUT);  // Set trigger pin as output
  pinMode(echo, INPUT);  // Set echo pin as input
  digitalWrite(fountain, LOW);  // Ensure the fountain is off initially
  myServo.attach(servoPin);  // Attach the servo to pin 9
  myServo.write(360);  // Move servo to position 360
  startTime = millis();  // Store the start time
  delay(1000);  // Wait for 1 second

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer... (May take 3~5 seconds)"));

  // Initialize the MP3 module
  if (!mp3.begin(ss)) { // Use software serial to communicate with the MP3 module.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1. Please recheck the connection!"));
    Serial.println(F("2. Please insert the SD card!"));
    while (true)
      ;  // Stay in an infinite loop if initialization fails
  }
  Serial.println(F("DFPlayer Mini online."));

  uint8_t error = 0;  // Variable for error checking
}

void loop() {
  // If the distance is greater than or equal to 50 cm
  if (dist >= 50) {
    digitalWrite(trigger, LOW);  // Set trigger pin low
    delayMicroseconds(5);  // Wait for 5 microseconds
    digitalWrite(trigger, HIGH);  // Set trigger pin high
    delayMicroseconds(10);  // Wait for 10 microseconds
    digitalWrite(trigger, LOW);  // Set trigger pin low
    dist = pulseIn(echo, HIGH);  // Measure the pulse duration on the echo pin
    dist = dist / 58;  // Convert pulse duration to distance in cm
    Serial.print("Distance = ");  // Print distance to serial monitor
    Serial.print(dist);  // Print the distance value
    Serial.print(" cm");  // Print the unit (cm)
    Serial.write(10);  // Send a newline character
    delay(200);  // Wait for 200 milliseconds
  } else {
    mp3.play(1);  // Play the first track on the MP3 module
    delay(2000);  // Wait for 2 seconds
    myServo.write(50);  // Move the servo to position 50
    digitalWrite(fountain, HIGH);  // Turn on the fountain
    while (true);  // Stay in an infinite loop
  }
}
