//Graham Skirucha // System Intergratiob Mini Project
//Section 4.5 Codefor Arduino

//Jaron
//make sure the encoder is connected to 5 and 6

#include <Encoder.h>
#define ENCODER_USE_INTERRUPTS
Encoder myEnc(2,3); //create encoder class
float lastRead; 
int degreesRead; //current degree pos. of the motor
int dutyCycle = 25;  // motor speed
int turnTo;   // where motor should turn (destination in degrees)
String data;
String sen;
int tolerance = 2; //degree tolerance of the destination
unsigned char position_bytes[2];

const int motorPin1 = 7; //direction
const int motorPin2 = 9; //pwm voltage

//Set variables for control of motor and controlling
int pointSet = 0;  // used to determine the turnTo position

void setup() {
  Serial.begin(115200);
  lastRead = 0;
  turnTo=90;
  pinMode(motorPin1, OUTPUT); // set motor pins for output
  pinMode(motorPin2, OUTPUT);

  digitalWrite(4, HIGH); // set enable pin to high to enable motor driver
}

void loop() {
  
  float encoderRead = myEnc.read();

  if (Serial.available() > 0){
    data = Serial.read(); // get destination from serial
    pointSet = data.toInt();
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
  }
  else if (pointSet == 1){
      turnTo = 90;
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
  }
  else if (pointSet == 2){
      turnTo = 270;
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
  }
  else{
      turnTo = 180;
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
  }

  sen = String(degreesRead);
  Serial.println(sen); // send current position back to raspberry Pi
  Serial.flush();
  delay(1);
}
