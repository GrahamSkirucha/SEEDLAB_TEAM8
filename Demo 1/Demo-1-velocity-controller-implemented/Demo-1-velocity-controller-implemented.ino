/*
Author: Jaron O'Grady
This code records velocity using external interrupts and then calculates position.
*/

#define enablePin 4
#define leftMotorPow 7
#define rightMotorPow 8
#define leftMotorDir 9
#define rightMotorDir 10

//need real values!!!
#define lKp 1
#define rKp 1
#define lKd 0
#define rKd 0
#define lKi 0
#define rKi 0

//adjustable
#define decelRate 10
#define startSpeed 10
#define accelRate 10

//adjustabled
double maxSpeed = 30;

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
float deltaT = 200;

float leftError;
float rightError;
float leftDerivative;
float rightDerivative;
float leftIntegral;
float rightIntegral;
float prevLeftError = 0;
float prevRightError = 0;

float flatDist = 0; // cm
float decelDist; //cm
float accelDist; //cm
float desiredDistance = 10; //cm
float desiredSpeed = 30;





//global variables include encoder pins, the states of the encoder pins, the net count of clockwise increments
int e1pinA = 2; //clk encoder 1
int e1pinB = 4; //dt encoder 1
int e2pinA = 3; //clk encoder 2
int e2pinB = 5; //dt encoder 2
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
double timeKept = 0.0;
double prev1Time = 0.0;
double prev2Time = 0.0;
double wheelRadius = 0.05;
double betweenWheels = 0.1;

//potential change on what is sent
double xpos = 0.0;
double ypos = 0.0;
double phi = 0.0;
int isrEntered = 0;




#define ROTATIONCOUNTS 80
#define DEBOUNCE 10

//isr measures the state of the pins and adjusts the counts and isr flag - only entered once pinA changes
void encoder1(){
  //read the pins
  int e1stateA = digitalRead(e1pinA);
  int e1stateB = digitalRead(e1pinB);
  //determine clockwise or counter-clockwise
  timeKept = millis();
  double duration = timeKept - prev1Time;
  //makes sure that there are no infinite velocities
  if(duration == 0.0) return;
  //the value of pi in arduino is M_PI
  double v = wheelRadius*2*2*PI/(ROTATIONCOUNTS * duration);
  if(e1stateA == e1stateB){
    leftSpeed = v;
  }
  else{
    leftSpeed = -1.0 * v;
  }
  //update the position variables
  xpos = xpos + duration * cos(phi) * (leftSpeed + rightSpeed) / 2;
  ypos = ypos + duration * sin(phi) * (leftSpeed + rightSpeed) / 2;
  phi = phi + duration * (leftSpeed - rightSpeed) / betweenWheels;
  //makes sure that phi stays in the range of -2PI to 2PI
  if((phi > 2 * PI) || (phi < -2 * PI)){
    phi = 0;
  }
  //adjust the flag to print the new value of counts in the loop
  isrEntered = 1;
  prev1Time = timeKept;
}

//isr measures the state of the pins and adjusts the counts and isr flag - only entered once pinA changes
void encoder2(){
  //read the pins
  int e2stateA = digitalRead(e2pinA);
  int e2stateB = digitalRead(e2pinB);
  timeKept = millis();
  double duration = timeKept - prev2Time;
  //make sure there are no infinite velocities that result in nan
  if(duration == 0.0) return;
  //the value of pi in arduino is M_PI
  double v = wheelRadius*2*2*PI/(ROTATIONCOUNTS * duration);
  if(e2stateA == e2stateB){
    rightSpeed = v;
  }
  else{
    rightSpeed = -1.0 * v;
  }
  //update the position variables
  xpos = xpos + duration * cos(phi) * (leftSpeed + rightSpeed) / 2;
  ypos = ypos + duration * sin(phi) * (leftSpeed + rightSpeed) / 2;
  phi = phi + duration * (leftSpeed - rightSpeed) / betweenWheels;
  //makes sure that phi stays in the range of -2PI to 2PI
  if((phi > 2 * PI) || (phi < -2 * PI)){
    phi = 0;
  }
  //adjust the flag to print the new value of counts in the loop
  isrEntered = 1;
  prev2Time = timeKept;
}


//sets pinA and pinB to pullup inputs, attaches the isr to pinA/clk, and sets baud rate to 9600
void setup() {
  //motor control setup
  pinMode(leftMotorPow, OUTPUT);
  pinMode(rightMotorPow, OUTPUT);

  pinMode(leftMotorDir, OUTPUT);
  pinMode(rightMotorDir, OUTPUT); 

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
  prevTime = millis();
  Serial.println(prevTime);
  //sei(); - uncomment if revisiting to see if this fixes the problem of not always seeing rotations/bouncing
  if(isrEntered){
    //sei(); - uncomment if revisiting to see if this fixes the problem of not always seeing rotations/bouncing
    //link for how to print doubles and floating points: https://www.programmingelectronics.com/dtostrf/
//    char buffer[144];
//    char printxbuffer[8];
//    char printybuffer[8];
//    char printphibuffer[8];
//    dtostrf(xpos, 4, 4, printxbuffer);
//    dtostrf(ypos, 4, 4, printybuffer);    
//    dtostrf(phi, 4, 4, printphibuffer);
//    //sprintf(buffer, "X position: %s, Y Position: %s, Phi: %s", printxbuffer, printybuffer, printphibuffer);
//    //Serial.println(buffer);
//    isrEntered = 0;
    Serial.println("isr");
    isrEntered = 0;
  }
    ////Define accel and decel Distances
  accelDist = (maxSpeed - startSpeed) / accelRate;
  decelDist = maxSpeed / decelRate;

  if (accelDist + decelDist < desiredDistance) {
    flatDist = desiredDistance - accelDist - decelDist;
  } else {
    // Redefine trapezoid
    maxSpeed = desiredDistance / (1 / accelRate + 1 / decelRate); // Max speed that will be reached in the trapezoid
    accelDist = maxSpeed / accelRate;
    decelDist = maxSpeed / decelRate;
  }
 
  //// Determine desiredSpeed
  // Acceleration segment
  if (xpos < accelDist) {
    desiredSpeed = startSpeed + accelRate * xpos;
  }
  // Flat segment
  else if (xpos < flatDist + accelDist) {
    desiredSpeed = maxSpeed;
  }
  // Deceleration segment
  else if (xpos < flatDist + accelDist + decelDist) {
    desiredSpeed = maxSpeed - (decelRate * (xpos - flatDist));
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
  //Serial.println(leftVOut);

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
  leftPWMOut = constrain(leftPWMOut, -255, 255);
  rightPWMOut = constrain(leftPWMOut, -255, 255);

  // write values to pins
  digitalWrite(leftMotorDir, leftMoveDir);
  leftPWMOut = 128;
  analogWrite(leftMotorPow, leftPWMOut);
  digitalWrite(rightMotorDir, rightMoveDir);
  
  analogWrite(rightMotorPow, abs(rightPWMOut));
  
  delay(deltaT);
  Serial.println("loop finished");
  

  
}
