#include <Encoder.h>
#define ENCODER_USE_INTERRUPTS
Encoder myEnc(2,3); //create encoder class
float lastRead; 
int degreesRead; //current degree pos. of the motor
int dutyCycle = 25;  // motor speed
int desiredAngle;   // where motor should turn (destination in degrees)
String data;
String sendPosition;
int tolerance = 1; //degree tolerance of the destination
unsigned char position_bytes[2];
bool DataRead;

const int motorPin1 = 7; //direction
const int motorPin2 = 9; //pwm voltage
const int enablePin = 4; 

int pointSet;  // used to determine the turnTo position
int tStart;
int tEnd;
float Kp;
float Ki;
float dt = 0.033;
float integral;

int error;
int lastError;
float vOut;
int PWMout;
bool moveDir;
float encoderRead;

void setup() {
  Serial.begin(38400);
  lastRead = 0;
  pointSet = 0;
  degreesRead = 0;
  Kp = 0.1;
  Ki = 0.2;
  lastError = 0;
  
  pinMode(motorPin1, OUTPUT); // set motor pins for output
  pinMode(motorPin2, OUTPUT);
  digitalWrite(enablePin, HIGH); // set enable pin to high to enable motor driver
}

void loop() {
  
  //tStart = micros(); 
  // recieve position
  if (DataRead) {
    pointSet = data.toInt(); //unitless
    DataRead = false;
  }

  encoderRead = myEnc.read();

  // get current position
  if(encoderRead != lastRead) {
    //convert to degrees
    degreesRead = encoderRead * 360 / 3200;

    lastRead = encoderRead;
  }
  
  // move motor
  desiredAngle = 90*pointSet; //degrees
  error = desiredAngle - degreesRead;
  integral = (error + lastError)/2*dt;
  vOut = Kp * (error) + Ki * integral; //volts
  lastError = error;

  // set direction
  if (vOut <= 0) {
    moveDir = HIGH;
  }
  else {
    moveDir = LOW;
  }


  PWMout = vOut * 255/8;

  // set PWM value
  if (PWMout > 255) {
    PWMout = 255;
  } else if (PWMout < -255) {
    PWMout = -255;
  } 


  // write values to pins
  digitalWrite(motorPin1, moveDir);
  analogWrite(motorPin2, abs(PWMout));
  
  // send current position
  //  sendPosition = String("Pwm: " + String(PWMout) + "degrees: " + String(degreesRead));
  sendPosition = String(degreesRead);

  Serial.println(sendPosition);
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
