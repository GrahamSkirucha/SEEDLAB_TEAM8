/*
Author: Ryan Sims, Maximus Waring, and Jaron O'Grady
This code records velocity using external interrupts and then calculates position.
*/
const int FORW = 1;
const int BACK = -1;

#define enablePin 4
#define leftMotorPow 10
#define rightMotorPow 9
#define leftMotorDir 8
#define rightMotorDir 7
#define pushButton 12//enter button pin here

//need real values!!!
#define lKp 0.06
#define rKp 0.02
#define lKd 0
#define rKd 0
#define lKi 0.02
#define rKi 0.02

//adjustable
float decelRate = 0.5;
float startSpeed = 0.75; // must be lower than max Speed
float maxSpeed = 0.9;
float accelRate = 0.2;

//adjustabled

bool leftMoveDir;
bool rightMoveDir;
float leftVOut;
float rightVOut;
float leftPWMOut;
float rightPWMOut;
float curTime;
float prevTime = 0;
float waitTime = 0;
float loopTime = 0;
float deltaT = 200.0;

float leftError;
float rightError;
float leftDerivative;
float rightDerivative;
float leftIntegral;
float rightIntegral;
float prevLeftError = 0.0;
float prevRightError = 0.0;

float flatDist = 0.0; // m
float decelDist; //m
float accelDist; //m
float desiredSpeed;

//global variables include encoder pins, the states of the encoder pins, the net count of clockwise increments
int e1pinA = 2; //clk encoder 1
int e1pinB = 5; //dt encoder 1
int e2pinA = 3; //clk encoder 2
int e2pinB = 6; //dt encoder 2
int e1stateA = 0;
int e1stateB = 0;
int e2stateA = 0;
int e2stateB = 0;
double leftSpeed; // temp
double rightSpeed;
//leftspeed
double velocity1 = 0.0;
//rightspeed
double velocity2 = 0.0;
double time1Kept = 0.0;
double time2Kept= 0.0;
double prev1Time = 0.0;
double prev2Time = 0.0;
#define WHEELRADIUS 0.0762
#define BETWEENWHEELS 0.1524

#define desiredMeters 0.305 * 5 //input meters
float desiredDistance = 3200 * (1/(WHEELRADIUS*2*PI)) * desiredMeters * 0.5; //counts


//potential change on what is sent
double xpos = 0.0;
double ypos = 0.0;
double phi = 0.0;
int isrEntered = 0;

int enc1Counts = 0;
int enc2Counts = 0;



#define ROTATIONCOUNTS 3000

//isr measures the state of the pins and adjusts the counts and isr flag - only entered once pinA changes
void encoder1(){
  //read the pins
  int e1stateA = digitalRead(e1pinA);
  int e1stateB = digitalRead(e1pinB);
  //determine clockwise or counter-clockwise
  time1Kept = millis();
  double duration = (time1Kept - prev1Time);
  //makes sure that there are no infinite velocities
  if(duration == 0.0) return;
  //the value of pi in arduino is M_PI
  double v = 1/duration;
  //double v = WHEELRADIUS*2*2*PI/(ROTATIONCOUNTS * duration);
  if(e1stateA == e1stateB){
    leftSpeed = v;
  }
  else{
    leftSpeed = -1.0 * v;
  }
  //update the position variables
  xpos = (xpos + duration * cos(phi) * (leftSpeed + rightSpeed)) / 2;
  ypos = ypos + duration * sin(phi) * (leftSpeed + rightSpeed) / 2;
  phi = phi + duration * (leftSpeed - rightSpeed) / BETWEENWHEELS;
  //makes sure that phi stays in the range of -2PI to 2PI
  if((phi > 2 * PI) || (phi < -2 * PI)){
    phi = 0;
  }
  //adjust the flag to print the new value of counts in the loop
  isrEntered = 1;
  prev1Time = time1Kept;

  if (v >= 0) {
    enc1Counts++;
  } else {
    enc1Counts--;
  }}

//isr measures the state of the pins and adjusts the counts and isr flag - only entered once pinA changes
void encoder2(){
  //read the pins
  int e2stateA = digitalRead(e2pinA);
  int e2stateB = digitalRead(e2pinB);
  time2Kept = millis();
  double duration = (time2Kept - prev2Time);
  //make sure there are no infinite velocities that result in nan
  if(duration == 0.0) return;
  //the value of pi in arduino is M_PI
  double v = 1 / duration;
  //double v = WHEELRADIUS*2*2*PI/(ROTATIONCOUNTS * duration);
  if(e2stateA == e2stateB){
    rightSpeed = v;
  }
  else{
    rightSpeed = -1.0 * v;
  }
  //update the position variables
  xpos = (xpos + duration * cos(phi) * (leftSpeed + rightSpeed)) / 2;
  ypos = ypos + duration * sin(phi) * (leftSpeed + rightSpeed) / 2;
  phi = phi + duration * (leftSpeed - rightSpeed) / BETWEENWHEELS;
  //makes sure that phi stays in the range of -2PI to 2PI
  if((phi > 2 * PI) || (phi < -2 * PI)){
    phi = 0;
  }
  //adjust the flag to print the new value of counts in the loop
  isrEntered = 1;
  prev2Time = time2Kept;
  if (v >= 0) {
    enc2Counts++;
  } else {
    enc2Counts--;
  }
}

int angleToCounts(float THETA) { // input theta in degrees
  float arcLength = BETWEENWHEELS / 2 * THETA * PI / 180; //m
  float dist = 3200 * (1/(WHEELRADIUS*2*PI)) * arcLength;

  return dist;
}

void trapezoid(float DISTANCE) {
  ////Define accel and decel Distances
  accelDist = (maxSpeed - startSpeed) / accelRate;
  decelDist = maxSpeed / decelRate;

  if (accelDist + decelDist < DISTANCE) {
    flatDist = DISTANCE - accelDist - decelDist;
  } else {
    // Redefine trapezoid
    maxSpeed = DISTANCE / (1 / accelRate + 1 / decelRate); // Max speed that will be reached in the trapezoid
    accelDist = maxSpeed / accelRate;
    decelDist = maxSpeed / decelRate;
  }
}

void move(float DISTANCE, int LDIR, int RDIR) {
  // create motion profile
  trapezoid(DISTANCE);
  enc1Counts = 0;
  enc2Counts = 0;

  while (1 == 1) {
    prevTime = millis();
    Serial.println(String(enc1Counts));
    
    //determine desired speed
    if (enc1Counts < accelDist) {
      desiredSpeed = startSpeed + accelRate * enc1Counts;
      //Serial.println("accelerating");
    }
    // Flat segment
    else if (enc1Counts < flatDist + accelDist) {
      desiredSpeed = maxSpeed;
      //Serial.println("flat");
    }
    // Deceleration segment
    else if (enc1Counts < flatDist + accelDist + decelDist) {
      desiredSpeed = maxSpeed - (decelRate * (enc1Counts - flatDist));
      //Serial.println("decelerating");
    }
    // Too far
    else {
      desiredSpeed = 0;
    }

    //dt = (curTime - prevTime) / 1000;

    leftError = desiredSpeed - leftSpeed;
    rightError = desiredSpeed - rightSpeed;

    leftDerivative = (leftError - prevLeftError) / deltaT;
    rightDerivative = (rightError - prevRightError) / deltaT;

    leftIntegral += (leftError - prevLeftError) / 2 * deltaT;
    rightIntegral += (rightError - prevRightError) / 2 * deltaT;

    leftVOut = lKp * leftError + lKd * leftDerivative + lKi * leftIntegral;
    rightVOut = rKp * rightError + rKd * rightDerivative + rKi * rightIntegral;

    leftVOut *= LDIR;
    rightVOut *= RDIR;

    prevLeftError = leftError;
    prevRightError = rightError;
    prevTime = curTime;

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
    // if (leftPWMOut > 3) {
    //   leftPWMOut = constrain(leftPWMOut, 30, 255);
    // } else if (leftPWMOut < -3) {
    //   leftPWMOut = constrain(leftPWMOut, -30, -255);
    // } else {
    //   leftPWMOut = 0;
    // }
    // if (rightPWMOut > 3) {
    //   rightPWMOut = constrain(rightPWMOut, 30, 255);
    // } else if (rightPWMOut < -3) {
    //   rightPWMOut = constrain(rightPWMOut, -30, -255);
    // } else {
    //   rightPWMOut = 0;
    // }
    leftPWMOut = constrain(leftPWMOut, -255, 255);
    rightPWMOut = constrain(rightPWMOut, -255, 255);

    // write values to pins
    digitalWrite(leftMotorDir, leftMoveDir);
    analogWrite(leftMotorPow, abs(leftPWMOut));
    digitalWrite(rightMotorDir, rightMoveDir);
    
    analogWrite(rightMotorPow, abs(rightPWMOut));
      //delay(deltaT);
    Serial.println("enc1Counts");
    Serial.println("leftSpeed: :" + String(leftSpeed * 1000) + " rightspeed: " + String(rightSpeed * 1000));
        //Serial.println("lPWM: " + String(leftPWMOut) + " RPWM: " + String(rightPWMOut) + "\nrightErr: " + String(rightError) + " leftErr: " + String(leftError));
   
    if ((enc1Counts >= DISTANCE && enc1Counts <= DISTANCE + 30) && (leftVOut == 0 || rightVOut == 0)) {
      //Serial.println("here");
      return ;
    }

  }

  //analogWrite(leftMotorPow, 0);
  //analogWrite(rightMotorPow, 0);
  
}

//This stuff is for the straight line code
// ----------
//Encoder Global Variables
#include <Encoder.h>
#define ENCODER_USE_INTERRUPTS
//two encoder objects
Encoder enc1 (2, 5);
Encoder enc2 (3, 6);
//constants
#define FEETTOMETERS 0.3048 //conversion factor from feet provided by the user to meters that the robot can measure
//Dual MC Motor Shield moves the motors
#include "DualMC33926MotorShield.h"
DualMC33926MotorShield md;
int prevRead1; //previous count for first encoder object
int prevRead2; //previous count for second encoder object
int motor1Speed = 0; //value the mc motor shield library uses to apply a voltage to motor 1
int motor2Speed = 0; //value the mc motor shield library uses to apply a voltage to motor 2
double desiredDist; //desired distance in meters
double distWithFudge = 0; //distance the robot will use as a reference to drive to
double desiredFeet = 5; //desired distance in feet - will be converted to meters
double distanceFudgeFactor = 0.07; //fudge factor so that the robot moves the desired distance - scales with distance

void straight(){
  //these would be global variables, but there are too darn many of those
  double controllerThreshold = 0.1; //distance away from the destination that the controller will kick in
  int printOnce = 0; //flag for testing the final distance the robot travelled and comparing it to the input distance
  int motorMax = 350; //determines the maximum value the motor1Speed and motor2Speed variables can reach - value the controller uses to decrease the speed of the robot
  int motorDecayFactor = motorMax * 10;
  //time variables to keep track of when the encoder is being read
  int timeKept; //holds the present time in the loop
  int preTime; //holds the previous time the encoders were read
  int deltaT; //determines the time between encoder readings
  double leftVelocity; //approximation of the velocity of left/first encoder
  double rightVelocity; //approximation of the velocity of the right/second encoder
  double x = 0; //calculated x position from both encoder readings
  double y = 0; //calculated y position from both encoder readings
  double p = 0.0; //calculated angle from both encoder readings assuming the starting orientation is 0 degrees
  enc1.write(0); //clearing the encoders makes sure that the code thinks that there has been no disturbance to the x position, y position, or angle
  enc2.write(0);
  while(true /*insert loop condition here*/){
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
      deltaT = timeKept - preTime;
      double deltaTSec = deltaT / 1000.0;
      //calculate velocity using the change in encoder counts and deltaT with constants
      leftVelocity = WHEELRADIUS*change1*2*PI/(ROTATIONCOUNTS * deltaTSec);
      rightVelocity = WHEELRADIUS*change2*2*PI/(ROTATIONCOUNTS * deltaTSec);
      //calculate the changes in position
      double deltaX = deltaTSec * cos(p) * (leftVelocity + rightVelocity) / 2;
      double deltaY = deltaTSec * sin(p) * (leftVelocity + rightVelocity) / 2;
      double deltap = deltaTSec * (leftVelocity - rightVelocity) / BETWEENWHEELS;
      //calculate new position
      x = x + deltaX;
      y = y + deltaY;
      p = p + deltap;
      //set bounds on p - if p is greater than 2PI subtract 2PI, and if p is less than -2PI add 2*PI
      if(p > 2 * PI){
        p -= 2 * PI;
      }
      if(p < (-2 * PI)){
        p += 2 * PI;
      }
      //set the prev* variables so that the next loop will use these values as a reference
      prevRead1 = read1;
      prevRead2 = read2;
      preTime = timeKept;
    }
    // ---

    //driving the motors
    // ---
    //sets the speed of both motors using the mc motor shield library
    if(x < distWithFudge){
      md.setM1Speed(motor1Speed);
      md.setM2Speed(motor2Speed);
      delay(2);
    }
    //stops the robot if it has reached its destination
    else{
      md.setM1Speed(0);
      md.setM2Speed(0);
      delay(2);
      break; //once the robot has reached this, it will stop
      // printOnce = 1; //exit code for debugging purposes
    }
    // ---

    //stabilizing the velocity
    // ---
    //takes the difference of the velocity
    double velocityDiff = leftVelocity - rightVelocity;
    //turns up the motor speed of the motor that is going slower
    if(velocityDiff > 0){
      motor2Speed += 10; //positive difference means the right wheel needs to go faster
    }
    else if(velocityDiff < 0){
      motor1Speed += 10; //negative difference means the left wheel needs to go faster
    }
    //turns up the speed of the motor that needs to compensate for a change in robot angle - smaller change to motor speed
    if(p > 0){
      motor2Speed += 2; //positive angle means the right motor needs to go faster
    }
    if(p < 0){
      motor1Speed += 2; //negative angle means the left motor needs to go faster
    }
    //make sure both motor speed integers are below the max speed
    while((motor1Speed > motorMax) || (motor2Speed > motorMax)){
      motor1Speed -= 10;
      motor2Speed -= 10;
    }
    // ---

    //implementing the controller
    // ---
    //only implement the controller if the robot is within the distance specified by controllerThreshold
    if(x > distWithFudge - controllerThreshold){
      double error = motorDecayFactor * (distWithFudge - x); //calculate the amount of distance left and multiply it by the motorDecayFactor
      motorMax = int(error); //set the new maximum motor speed - the motor speed will decay quickly from 350 because of the speed of the loop and the responsiveness of the controller
      // Serial.println(motorMax);
    }
    // ---

    // printline debugging
    // ---
    // Serial.println(int(x * 10000));
    // if(printOnce){
    //   Serial.println(int(distWithFudge * 10000));
    //   exit(0);
    // } 
    // Serial.println("left speed: " + String(motor1Speed) + "\tright speed: " + String(motor2Speed));
    // Serial.println("Left Velocity: " + String(leftVelocity) + "\tRight Velocity: " + String(rightVelocity));
    // Serial.println("Motor 1: " + String(motor1Speed) + "\tMotor 2: " + String(motor2Speed));
    // Serial.println("cm: " + String(int(x * 100)));
    // ---
  }    
}

//sets pinA and pinB to pullup inputs, attaches the isr to pinA/clk, and sets baud rate to 9600
void setup() {
  //setup for straight line code
  // ----------
  //encoder reading setup
  // ---
  prevRead1 = 0;
  prevRead2 = 0;
  // ---

  //motor control setup
  // ---
  md.init();
  motor1Speed = 300;
  motor2Speed = 340;
  // ---

  //fudge factors
  // ---
  desiredDist = desiredFeet * FEETTOMETERS;
  distWithFudge = desiredDist + desiredDist * distanceFudgeFactor;
  // ----------

  //motor control setup
  pinMode(leftMotorPow, OUTPUT);
  pinMode(rightMotorPow, OUTPUT);

  pinMode(leftMotorDir, OUTPUT);
  pinMode(rightMotorDir, OUTPUT); 
  pinMode(pushButton, INPUT_PULLUP);

  digitalWrite(enablePin, HIGH); // set enable pin to high to enable motor driver
  pinMode(e1pinA, INPUT_PULLUP);
  pinMode(e1pinB, INPUT_PULLUP);
  pinMode(e2pinA, INPUT_PULLUP);
  pinMode(e2pinB, INPUT_PULLUP);
  attachInterrupt(0, encoder1, CHANGE);
  attachInterrupt(1, encoder2, CHANGE);
  Serial.begin(38400);
}

//prints the net counts given there has been a change in pinA
void loop() {


  //Serial.println("accel: " + String(accelDist) + "\nflat: " + String(flatDist) + "\ndecel: " + String(decelDist));
  //sei(); - uncomment if revisiting to see if this fixes the problem of not always seeing rotations/bouncing
  if(isrEntered){
    //sei(); - uncomment if revisiting to see if this fixes the problem of not always seeing rotations/bouncing
    //link for how to print doubles and floating points: https://www.programmingelectronics.com/dtostrf/
    // char buffer[144];
    // char printxbuffer[8];
    // char printybuffer[8];
    // char printphibuffer[8];
    // dtostrf(xpos, 4, 4, printxbuffer);
    // dtostrf(ypos, 4, 4, printybuffer);    
    // dtostrf(phi, 4, 4, printphibuffer);
    //Serial.println(phi);
    //sprintf(buffer, "X position: %s, Y Position: %s, Phi: %s", printxbuffer, printybuffer, printphibuffer);
    //Serial.println(buffer);
    isrEntered = 0;
    // Serial.println("isr");
  }
  


  // // serial
  float angleRecieved = 360 * 0.60;
  int ldir = (angleRecieved >= 0) ? -1 : 1; 
  int rdir = -ldir;
  int turnCounts = angleToCounts(abs(angleRecieved));

  move(turnCounts, ldir, rdir);
  straight(); //this calls the straight code
  // delay(100);
  //Serial.println("desD: " +String(desiredDistance));
  //move(desiredDistance, FORW, FORW);
  //Serial.println("done");

  //wait for button press
  while (digitalRead(pushButton) == 1); // wait for button push
  while (digitalRead(pushButton) == 0); // wait for button release
  // Serial.println("button");
  
}
