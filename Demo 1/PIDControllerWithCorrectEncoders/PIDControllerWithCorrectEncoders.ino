//Encoder Global Variables
#include <Encoder.h>
#define ENCODER_USE_INTERRUPTS
//two encoder objects
Encoder enc1 (2, 5);
Encoder enc2 (3, 6);
//time variables to keep track of when the encoder is being read
int prevRead1; //previous count for first encoder object
int prevRead2; //previous count for second encoder object
int timeKept; //holds the present time in the loop
int prevTime; //holds the previous time the encoders were read
int duration; //determines the time between encoder readings
double leftSpeed; //nubmer of counts per second on the left/first encoder
double rightSpeed; //nubmer of counts per second on the right/second encoder
double leftVelocity; //approximation of the velocity of left/first encoder
double rightVelocity; //approximation of the velocity of the right/second encoder
double x; //calculated x position from both encoder readings
double y; //calculated y position from both encoder readings
double phi = 0.0; //calculated angle from both encoder readings assuming the starting orientation is 0 degrees
#define WHEELRADIUS 0.0762 //constant wheel radius (calculates velocity)
#define BETWEENWHEELS 0.1524 //constant distance between the wheels (calculates phi)
#define ROTATIONCOUNTS 3000 //encoders are 3000 counts per every 360 degrees

//PID control values
double lkp = 1.0;
double rkp = 1.0;
double lkd = 0.0;
double rkd = 0.0;
double lki = 0.0;
double rki = 0.0;
//voltage and pwm values
double leftVOut;
double rightVOut;
double leftPWMOut;
double rightPWMOut;
double desiredDistance = 10; //cm

//PID Global Variables
#include <PID_v1.h>
PID myPID(&x,&leftPWMOut,&desiredDistance, lkp,lki,lkd,DIRECT);

#define enablePin 4
#define leftMotorPow 7
#define rightMotorPow 8
#define leftMotorDir 9
#define rightMotorDir 10

//max speed
double maxSpeed = 30;

#define enablePin 4
#define leftMotorPow 7
#define rightMotorPow 8
#define leftMotorDir 9
#define rightMotorDir 10

//adjustable
#define decelRate 10
#define startSpeed 10
#define accelRate 10

bool leftMoveDir;
bool rightMoveDir;
double curTime;
double waitTime = 0;
double loopTime = 0;
double deltaT = 200;

double leftError;
double rightError;
double leftDerivative;
double rightDerivative;
double leftIntegral;
double rightIntegral;
double prevLeftError = 0;
double prevRightError = 0;

double flatDist = 0; // cm
double decelDist; //cm
double accelDist; //cm
double desiredSpeed = 30;

//sets pinA and pinB to pullup inputs, attaches the isr to pinA/clk, and sets baud rate to 9600
void setup() {
  //encoder reading setup
  // ---
  prevRead1 = 0;
  prevRead2 = 0;
  // ---
  //motor control setup
  pinMode(leftMotorPow, OUTPUT);
  pinMode(rightMotorPow, OUTPUT);

  pinMode(leftMotorDir, OUTPUT);
  pinMode(rightMotorDir, OUTPUT); 

  digitalWrite(enablePin, HIGH); // set enable pin to high to enable motor driver
  Serial.begin(38400);
}

//prints the net counts given there has been a change in pinA
void loop() {
  //encoder reading loop code
  //---  
  //read the encoders
  int read1 = enc1.read();
  int read2 = enc2.read();
  //only compute values if the readings have changed
  if((read1 != prevRead1) || (read2 != prevRead2)){
    //compute the change in the readings
    int change1 = read1 - prevRead1;
    int change2 = read2 - prevRead2;
    //take the time and calculate the loop time
    timeKept = millis();
    duration = timeKept - prevTime;
    double durationSec = duration / 1000.0;
    //calculate velocity using the change in encoder counts and duration with constants
    leftVelocity = WHEELRADIUS*change1*2*PI/(ROTATIONCOUNTS * durationSec);
    leftSpeed = change1/durationSec;
    rightVelocity = WHEELRADIUS*change2*2*PI/(ROTATIONCOUNTS * durationSec);
    rightSpeed = change2/durationSec;
    //calculate the changes in position
    double deltaX = durationSec * cos(phi) * (leftVelocity + rightVelocity) / 2;
    double deltaY = durationSec * sin(phi) * (leftVelocity + rightVelocity) / 2;
    double deltaPhi = durationSec * (leftVelocity - rightVelocity) / BETWEENWHEELS;
    //calculate new position
    x = x + deltaX;
    y = y + deltaY;
    phi = phi + deltaPhi;
    //set bounds on phi - if phi is greater than 2PI subtract 2PI, and if phi is less than -2PI add 2*PI
    if(phi > 2 * PI){
      phi -= 2 * PI;
    }
    if(phi < (-2 * PI)){
      phi += 2 * PI;
    }
    prevRead1 = read1;
    prevRead2 = read2;
    prevTime = timeKept;
  }
  // ---
  // prevTime = millis();
  // Serial.println(prevTime);
  //   ////Define accel and decel Distances
  // accelDist = (maxSpeed - startSpeed) / accelRate;
  // decelDist = maxSpeed / decelRate;

  // if (accelDist + decelDist < desiredDistance) {
  //   flatDist = desiredDistance - accelDist - decelDist;
  // } else {
  //   // Redefine trapezoid
  //   maxSpeed = desiredDistance / (1 / accelRate + 1 / decelRate); // Max speed that will be reached in the trapezoid
  //   accelDist = maxSpeed / accelRate;
  //   decelDist = maxSpeed / decelRate;
  // }
 
  // //// Determine desiredSpeed
  // // Acceleration segment
  // if (x < accelDist) {
  //   desiredSpeed = startSpeed + accelRate * xpos;
  // }
  // // Flat segment
  // else if (xpos < flatDist + accelDist) {
  //   desiredSpeed = maxSpeed;
  // }
  // // Deceleration segment
  // else if (xpos < flatDist + accelDist + decelDist) {
  //   desiredSpeed = maxSpeed - (decelRate * (xpos - flatDist));
  // }
  // // Too far
  // else {
  //   desiredSpeed = 0;
  // }

  // //dt = (curTime - prevTime) / 1000;

  // leftError = desiredSpeed - leftSpeed;
  // rightError = desiredSpeed - rightSpeed;

  // leftDerivative = (leftError - prevLeftError) / deltaT;
  // rightDerivative = (rightError - prevRightError) / deltaT;

  // leftIntegral += (leftError - prevLeftError) / 2 * deltaT;
  // rightIntegral += (rightError - prevRightError) / 2 * deltaT;

  // leftVOut = lKp * leftError + lKd * leftDerivative + lKi * leftIntegral;
  
  // rightVOut = rKp * rightError + rKd * rightDerivative + rKi * rightIntegral;
  // //Serial.println(leftVOut);

  // prevLeftError = leftError;
  // prevRightError = rightError;
  // prevTime = curTime;

  // set direction left
  if (leftVOut <= 0) {
    leftMoveDir = HIGH;
  }
  else {
    leftMoveDir = LOW;
  }

  // set direction right
  if (rightVOut <= 0) {
    rightMoveDir = HIGH;
  }
  else {
    rightMoveDir = LOW;
  }

  leftPWMOut = leftVOut * 255/8;
  rightPWMOut = rightVOut * 255/8;

  // constrain PMWs
  leftPWMOut = constrain(leftPWMOut, -255, 255);
  rightPWMOut = constrain(leftPWMOut, -255, 255);

  // write values to pins
  digitalWrite(leftMotorDir, leftMoveDir);
  leftPWMOut = 128;
  analogWrite(leftMotorPow, leftPWMOut);
  digitalWrite(rightMotorDir, rightMoveDir);
  rightPWMOut = 128;
  analogWrite(rightMotorPow, abs(rightPWMOut));
  
  delay(deltaT);
  //Serial.println("loop finished");
  

  
}

