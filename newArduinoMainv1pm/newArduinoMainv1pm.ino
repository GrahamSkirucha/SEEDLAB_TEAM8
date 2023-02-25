#include <Encoder.h>
#define ENCODER_USE_INTERRUPTS
Encoder myEnc(2,3); //create encoder class
float lastRead; 
int degreesRead; //current degree pos. of the motor
int dutyCycle = 25;  // motor speed
int desiredAngle;   // where motor should turn (destination in degrees)
String data;
int sendPosition;
int tolerance = 1; //degree tolerance of the destination
unsigned char position_bytes[2];
bool DataRead;

const int motorPin1 = 7; //direction
const int motorPin2 = 9; //pwm voltage
const int enablePin = 4; 

int pointSet;  // used to determine the turnTo position
int tStart;
int tEnd;

void setup() {
  Serial.begin(38400);
  lastRead = 0;
  pointSet = 0;
  pinMode(motorPin1, OUTPUT); // set motor pins for output
  pinMode(motorPin2, OUTPUT);
  digitalWrite(enablePin, HIGH); // set enable pin to high to enable motor driver
}

void loop() {
  tStart = micros(); 
  // recieve position
  if (DataRead) {
    //pointSet = data;
    DataRead = false;
    
  }
  
  // move motor

  // send current position
    sendPosition = 0;
    Serial.println(data);
  // tEnd = micros();
  delay(100);
}

void serialEvent() {
  if (Serial.available() > 0) {
    data = Serial.readStringUntil('\n');
    DataRead = true;
  }
  Serial.flush();
}
