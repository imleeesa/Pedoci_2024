#include <Wire.h>
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include "paj7620.h"

/*
  1: Here we are in Saint Mark 7000
  2: Right now I'm steering a gondola 4000
  3: LONGO 2000
  4: WATCH OUT 2000
  5: AO STOP 4000
  6: OISSA 2 seconds
  7: Good Morning everyone 12 seconds
  8: Be careful
*/

#define GES_REACTION_TIME    500       // You can adjust the reaction time according to the actual circumstance.
#define GES_ENTRY_TIME      800       // When you want to recognize the Forward/Backward gestures, your gestures' reaction time must be less than GES_ENTRY_TIME (0.8s). 
#define GES_QUIT_TIME     1000

// Define global variables
int distance2;  // Distance measured by the ultrasonic sensor
unsigned long tempo;  // Time variable
long distance = 150;  // Maximum reading distance of the ultrasonic sensor
const int ritardo = 200;  // Delay time between commands
String msgIn;  // Message received from HC05
String msgOut;  // Message to send to HC05
String msgOut2;  // Response message
char a[100];  // Buffer for serial transmission

SoftwareSerial mySoftwareSerial(52, 50); // RX, TX

DFRobotDFPlayerMini myDFPlayer;

void printDetail(uint8_t type, int value);

int ENA = 4;
int IN1 = 2;
int IN2 = 3;
int ENB = 7;
int IN3 = 5;
int IN4 = 6;
int ENC = 10;
int IN5 = 8;
int IN6 = 9;

const int redOut = A1;
const int greenOut = A2;
const int blueOut = A0;

boolean canI = false;

void setColor(int red, int green, int blue) {
  analogWrite(redOut, red);
  analogWrite(greenOut, green);
  analogWrite(blueOut, blue);
}

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(38400);
  Serial2.begin(38400);
  mySoftwareSerial.begin(9600);
  Serial.begin(38400);

  Serial.print("Leone begin");

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(ENC, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  
  // Turn off motors - Initial State
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);

  pinMode(redOut, OUTPUT);
  pinMode(greenOut, OUTPUT);
  pinMode(blueOut, OUTPUT);

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer... (May take 3~5 seconds)"));

  if (!myDFPlayer.begin(mySoftwareSerial)) { // Use softwareSerial to communicate with the MP3 module.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1. Please recheck the connection!"));
    Serial.println(F("2. Please insert the SD card!"));
    while (true);
  }
  Serial.println(F("DFPlayer Mini online."));

  uint8_t error = 0;

  Serial.println("\nPAJ7620U2 TEST DEMO: Recognize 9 gestures.");

  error = paj7620Init();      // initialize Paj7620 registers

  if (error) {
    Serial.print("INIT ERROR, CODE:");
    Serial.println(error);
    setColor(0, 255, 255);
    while (true);
  } else {
    Serial.println("INIT OK");
  }
  Serial.println("Please input your gestures:\n");
  setColor(0, 255, 255);
  delay(1000);
  setColor(255, 255, 255);
  myDFPlayer.volume(30);
}

void loop() {
  // directionControl(10000);
  delay(2000);
  handleGesture();
  delay(1000);
  myDFPlayer.play(7);
  delay(12000);
  myDFPlayer.play(3);
  delay(3000);
  myDFPlayer.play(2);
  delay(3000);
  myDFPlayer.play(4);
  delay(3000);
  Serial2.write("$C#");
  Serial.print("sended");
  receiveDistance();
  // myDFPlayer.play(1);
  // delay(2000);
  Serial.println("cadsj");
  myDFPlayer.play(8);
  delay(3000);
  receiver("U");
  turnAround2(500);
  delay(1000);
  directionControl(2000);
  delay(1000);
  turnAround(500);
  myDFPlayer.play(1);
  while (true);
}

void turnAround(int time) {
  // Set the motors to maximum power
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  analogWrite(ENC, 100);

  // Move motors A, B, and C forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);

  // Wait for the specified time
  delay(time);

  // Stop the motors
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  analogWrite(ENC, 0);
}

void receiver(String mess) {
  // Loop until the confirmation message "$ACK#" is received
  while (msgOut != "$ACK#") {
    // Wait until data is available on the Bluetooth serial
    while (!Serial1.available()) {}

    // Read incoming data and store it in msgIn
    msgIn = "";
    while (Serial1.available()) {
      delay(200);
      Serial.print("cajds");
      char c = Serial1.read();
      msgIn = msgIn + c;
    }
    Serial.println(msgIn);
    String m = String(msgIn);
    
    // Send an ACK signal to Pinocchio
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

void receiver0(String mess) {
  String message = "";
  while (message != mess) {
    if (Serial2.available()) {
      delay(200);
      char receivedChar = Serial2.read(); // Read the received character
      
      // If the received character is '$', start storing the message
      if (receivedChar == '$') {
        message = "";
        while (Serial2.available()) {
          char nextChar = Serial2.read();
          if (nextChar == '#') {
            // End of message, print the received message
            Serial.println("Message received: " + message);
            break;
          }
          message += nextChar;
        }
      }
    }
  }
}

void receiveDistance() {
  String receivedData = "";
  unsigned long startTime = millis();
  int distance = 10000;
  while (distance > 50 && millis() - startTime < 5000) {
    if (Serial2.available()) {
      char c = Serial2.read();
      Serial.println(c);
      if (c == '!') {
        distance = receivedData.toInt();
        if (distance != 0) { // Ignore messages with zero distance
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

void turnAround2(int time) {
  // Set the motors to maximum power
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  analogWrite(ENC, 100);

  // Move motors A, B, and C backward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);

  // Wait for the specified time
  delay(time);

  // Stop the motors
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  analogWrite(ENC, 0);
}

void directionControl(int time) {
  // Set the motors to maximum power
  analogWrite(ENA, 0);
  analogWrite(ENB, 105); // right
  analogWrite(ENC, 105); // left from front

  // Move motors A, B, and C forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);

  // Wait for the specified time
  delay(time);

  // Stop the motors
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  analogWrite(ENC, 0);
}

void handleGesture() {
  while (true) {
    uint8_t data = 0, data1 = 0, error;
    error = paj7620ReadReg(0x43, 1, &data); // Read Bank_0_Reg_0x43/0x44 for gesture result.
    if (!error) {
      switch (data) { // When different gestures are detected, the variable 'data' will be set to different values by paj7620ReadReg(0x43, 1, &data).
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
            setColor(0, 255, 0);
            delay(1000);
            setColor(255, 255, 255);
            if (canI == true) {
              directionControl(1700);
              myDFPlayer.play(6);
              turnAround2(500);
              delay(2000);
              Serial1.println("$O#");
              return;
            }
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
            setColor(255, 255, 0); // GREEN
            delay(200);
            setColor(255, 255, 255);
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
            setColor(255, 255, 0); // BLUE
            delay(200);
            setColor(255, 255, 255); // BLUE
            delay(1000);
            canI = true;
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
            setColor(255, 0, 255);
            delay(1000);
            setColor(255, 255, 255);
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
          Serial.println("Anti-clockwise");
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