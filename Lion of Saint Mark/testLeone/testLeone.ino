#include <Wire.h>
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include "paj7620.h"

// Predefined messages
/*
  1 - Always be careful.
  2 - Go slow
  3 - Hello, I am the Lion of Venice
  4 - Oissa
  5 - Hey, stop if it's my turn
  6 - Watch out for the boat
  7 - Long
  8 - I am driving the gondola
  9 - Piazza San Marco
*/

// Define constants for gesture recognition timing
#define GES_REACTION_TIME 500
#define GES_ENTRY_TIME 800
#define GES_QUIT_TIME 1000

// Define global variables
int distance2;  // Distance calculated by the ultrasonic sensor
unsigned long tempo;  // Time variable
long distance = 150;  // Maximum reading distance of the ultrasonic sensor
const int ritardo = 200;  // Delay time between commands
String msgIn;  // Message received from the HC05
String msgOut;  // Message to be sent to the HC05
String msgOut2;  // Response message
char a[100];  // Buffer for serial transmission

// Initialize software serial on pins 52 (RX) and 50 (TX)
SoftwareSerial mySoftwareSerial(52, 50);

DFRobotDFPlayerMini myDFPlayer;  // Initialize DFPlayer Mini object

void printDetail(uint8_t type, int value);  // Function prototype

// Motor control pins
int ENA = 8;
int IN1 = 9;
int IN2 = 10;
int ENB = 5;
int IN3 = 6;
int IN4 = 7;
int ENC = 2;
int IN5 = 4;
int IN6 = 3;

// RGB LED output pins
int redOut = A2;
int greenOut = A1;
int blueOut = A0;

// Function to set the color of the RGB LED
void setColor(int red, int green, int blue) {
  analogWrite(redOut, red);
  analogWrite(greenOut, green);
  analogWrite(blueOut, blue);
}

void setup() {
  // Initialize serial communication
  Serial1.begin(38400);
  Serial2.begin(38400);
  mySoftwareSerial.begin(9600);
  Serial.begin(38400);

  Serial.print("Leone begin");

  // Set motor control pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(ENC, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  
  // Turn off motors initially
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);

  // Set RGB LED pins as outputs
  pinMode(redOut, OUTPUT);
  pinMode(greenOut, OUTPUT);
  pinMode(blueOut, OUTPUT);

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer... (May take 3~5 seconds)"));

  // Initialize DFPlayer
  if (!myDFPlayer.begin(mySoftwareSerial)) {
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1. Please recheck the connection!"));
    Serial.println(F("2. Please insert the SD card!"));
    while (true);
  }
  Serial.println(F("DFPlayer Mini online."));

  uint8_t error = 0;
  Serial.println("\nPAJ7620U2 TEST DEMO: Recognize 9 gestures.");

  // Initialize PAJ7620 gesture sensor
  error = paj7620Init();
  if (error) {
    Serial.print("INIT ERROR,CODE:");
    Serial.println(error);
    setColor(255, 0, 0);
    while (true);
  } else {
    Serial.println("INIT OK");
  }
  Serial.println("Please input your gestures:\n");
  setColor(255, 0, 0);
  delay(1000);
  setColor(0, 0, 0);
  myDFPlayer.volume(30);
}

void loop() {
  // Main loop
  delay(2000);
  handleGesture();
  delay(1000);
  myDFPlayer.play(3);  // Play message 3
  delay(11000);
  myDFPlayer.play(1);  // Play message 1
  delay(2000);
  myDFPlayer.play(6);  // Play message 6
  delay(1000);
  Serial2.write("$C#");
  Serial.print("sended");
  receiveDistance();
  Serial.println("cadsj");
  receiver("U");
  turnAround2(500);
  delay(1000);
  directionControl(4000);
  myDFPlayer.play(9);  // Play message 9
  while (true);
}

// Function to turn around for a specified time
void turnAround(int time) {
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  analogWrite(ENC, 100);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);

  delay(time);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  analogWrite(ENC, 0);
}

// Function to receive and acknowledge a message
void receiver(String mess) {
  while (msgOut != "$ACK#") {
    while (!Serial1.available()) {}
    msgIn = "";
    while (Serial1.available()) {
      delay(200);
      Serial.print("cajds");
      char c = Serial1.read();
      msgIn = msgIn + c;
    }
    Serial.println(msgIn);
    String m = String(msgIn);
    if (m == mess) {
      Serial.print("INN");
      msgOut = "ACK";
      int len = strlen(msgOut.c_str());
      char a[len + 2];
      a[0] = '$';
      for (int i = 1; i <= len; i++) {
        a[i] = msgOut.charAt(i - 1);
      }
      a[len + 1] = '#';
      msgOut = "";
      for (int i = 0; i <= len + 1; i++) {
        msgOut = msgOut + a[i];
        Serial1.println(a[i]);
        delay(200);
      }
      Serial.println(msgOut);
    }
  }
}

// Function to receive and acknowledge a message on Serial2
void receiver0(String mess) {
  String message = "";
  while (message != mess) {
    if (Serial2.available()) {
      delay(200);
      char receivedChar = Serial2.read();
      if (receivedChar == '$') {
        message = "";
        while (Serial2.available()) {
          char nextChar = Serial2.read();
          if (nextChar == '#') {
            Serial.println("Message received: " + message);
            break;
          }
          message += nextChar;
        }
      }
    }
  }
}

// Function to receive distance data
void receiveDistance() {
  String receivedData = "";
  unsigned long startTime = millis();
  int distance = 10000;
  while (distance > 120 && millis() - startTime < 5000) {
    if (Serial2.available()) {
      char c = Serial2.read();
      Serial.println(c);
      if (c == '!') {
        distance = receivedData.toInt();
        if (distance != 0) {
          Serial.print("Distance received: ");
          Serial.println(distance);
        } else {
          distance = 100;
        }
        receivedData = "";
      } else {
        receivedData += c;
      }
    }
  }
  Serial.println("STOP");
  Serial1.print("$S#");
}

// Function to turn around for a specified time with different motor direction
void turnAround2(int time) {
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  analogWrite(ENC, 100);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);

  delay(time);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  analogWrite(ENC, 0);
}

// Function to control direction for a specified time
void directionControl(int time) {
  analogWrite(ENA, 0);
  analogWrite(ENB, 100);  // Right motor
  analogWrite(ENC, 100);  // Left motor

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);

  delay(time);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  analogWrite(ENC, 0);
}

// Function to handle gesture recognition
void handleGesture() {
  while (true) {
    uint8_t data = 0, data1 = 0, error;
    error = paj7620ReadReg(0x43, 1, &data);
    if (!error) {
      switch (data) {
        case GES_RIGHT_FLAG:
          delay(GES_ENTRY_TIME);
          paj7620ReadReg(0x43, 1, &data);
          if (data == GES_FORWARD_FLAG) {
            Serial.println("Forward");
            delay(GES_QUIT_TIME);
          } else if (data == GES_BACKWARD_FLAG) {
            Serial.println("Backward");
            delay(GES_QUIT_TIME);
          } else {
            Serial.println("Right");
            setColor(255, 0, 255);
            delay(1000);
            setColor(0, 0, 0);
            directionControl(1700);
            myDFPlayer.play(4);  // Play message 4
            turnAround(500);
            delay(2000);
            Serial1.println("$O#");
            return;
          }
          break;
        case GES_LEFT_FLAG:
          delay(GES_ENTRY_TIME);
          paj7620ReadReg(0x43, 1, &data);
          if (data == GES_FORWARD_FLAG) {
            Serial.println("Forward");
            delay(GES_QUIT_TIME);
          } else if (data == GES_BACKWARD_FLAG) {
            Serial.println("Backward");
            delay(GES_QUIT_TIME);
          } else {
            Serial.println("Left");
            setColor(0, 0, 255);
            delay(200);
            setColor(0, 0, 0);
          }
          break;
        case GES_UP_FLAG:
          delay(GES_ENTRY_TIME);
          paj7620ReadReg(0x43, 1, &data);
          if (data == GES_FORWARD_FLAG) {
            Serial.println("Forward");
            delay(GES_QUIT_TIME);
          } else if (data == GES_BACKWARD_FLAG) {
            Serial.println("Backward");
            delay(GES_QUIT_TIME);
          } else {
            Serial.println("Up");
            setColor(0, 0, 255);
            delay(200);
            setColor(0, 0, 0);
            delay(1000);
            Serial1.println("$V#");
          }
          break;
        case GES_DOWN_FLAG:
          delay(GES_ENTRY_TIME);
          paj7620ReadReg(0x43, 1, &data);
          if (data == GES_FORWARD_FLAG) {
            Serial.println("Forward");
            delay(GES_QUIT_TIME);
          } else if (data == GES_BACKWARD_FLAG) {
            Serial.println("Backward");
            delay(GES_QUIT_TIME);
          } else {
            Serial.println("Down");
            setColor(0, 255, 0);
            delay(1000);
            setColor(0, 0, 0);
          }
          break;
        case GES_FORWARD_FLAG:
          Serial.println("Forward");
          delay(GES_QUIT_TIME);
          break;
        case GES_BACKWARD_FLAG:
          Serial.println("Backward");
          delay(GES_QUIT_TIME);
          break;
        case GES_CLOCKWISE_FLAG:
          Serial.println("Clockwise");
          break;
        case GES_COUNT_CLOCKWISE_FLAG:
          Serial.println("anti-clockwise");
          break;
        default:
          paj7620ReadReg(0x44, 1, &data1);
          if (data1 == GES_WAVE_FLAG) {
            Serial.println("wave");
          }
          break;
      }
    }
    delay(100);
  }
}
