//Graham Skirucha // System Intergratiob Mini Project
//Section 4.5 Codefor Arduino

//Jaron
//make sure the encoder is connected to 5 and 6

#include <Encoder.h>
#include <Wire.h>
#define ENCODER_USE_INTERRUPTS
Encoder myEnc(2,3); //create encoder class
float lastRead; 
int degreesRead; //current degree pos. of the motor
int dutyCycle = 25;  // motor speed
int turnTo;   // where motor should turn (destination in degrees)
int tolerance = 2; //degree tolerance of the destination
unsigned char position_bytes[2];

const int motorPin1 = 7; //direction
const int motorPin2 = 9; //pwm voltage

//Set variables for control of motor and controlling
int pointSet = 0;  // used to determine the turnTo position

// void requestEvent() {
//   Wire.write("Hello from Arduino");
// }

// void receiveEvent(int numBytes) {
//   while (Wire.available()) {
//     char c = Wire.read();
//     // do something with received data
//   }
// }

void setup() {
  Wire.begin(1);
  Serial.begin(9600)
  lastRead = 0;
  turnTo = 0;

  pinMode(motorPin1, OUTPUT); // set motor pins for output
  pinMode(motorPin2, OUTPUT);

  digitalWrite(4, HIGH); // set enable pin to high to enable motor driver
}

void loop() {
  float encoderRead = myEnc.read();

  //read from pi
  while (Wire.available() >= sizeof(int))  {
    int data = 0;

    for (int i =0; i < sizeof(int); i++) {
      data |= (Wire.read() << (8 * i));  
    }

    pointSet = data;
    Serial.println(pointSet);
  }

  // get current position in degrees
  if(encoderRead != lastRead) {
    //convert to degrees
    degreesRead = encoderRead / 3000 * 360;

    lastRead = encoderRead;
  }

  // this if structure gives directions to the motor based on pointSet
  if (pointSet == 0) {
      turnTo = 0;
  } else if (pointSet == 1){
      turnTo = 90;
  } else if (pointSet == 2){
      turnTo = 270;
  } else if (pointSet == 3){
      turnTo = 180;
  }

  // determine if motor should move forward or back
  ///// TODO: Add integral control 
  if (degreesRead <= turnTo - tolerance) {
    digitalWrite(7, LOW); // set direction forward
    analogWrite(9, dutyCycle * 255 / 100); // run motor

  } else if ((degreesRead >= (turnTo - tolerance)) && (degreesRead <= turnTo + tolerance)) {
    analogWrite(9, 0); // stop motor

  } else if (degreesRead >= turnTo + tolerance) {
    digitalWrite(7, HIGH); // set direction forward
    analogWrite(9, dutyCycle * 255 / 100); // run motor
  }

// send to pi
  byte buffer[sizeof(int)];  // create a byte array with the same size as the integer
  memcpy(buffer, &degreesRead, sizeof(int));  // copy the integer to the byte array
  Wire.write(buffer, sizeof(int));  // send the byte array over I2C  //delay(1);
}