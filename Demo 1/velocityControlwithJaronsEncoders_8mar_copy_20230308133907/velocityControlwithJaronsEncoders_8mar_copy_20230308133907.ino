/*
Author: Jaron O'Grady
This code records velocity using external interrupts and then calculates position.
*/
const int FORW = 1;
const int BACK = -1;

#define enablePin 4
#define leftMotorPow 9
#define rightMotorPow 10
#define leftMotorDir 7
#define rightMotorDir 8
#define pushButton 12//enter button pin here

//need real values!!!
#define lKp 1
#define rKp 1
#define lKd 0
#define rKd 0
#define lKi 0.7
#define rKi 0.7

//adjustable
float decelRate = 1.0;
float startSpeed = 3.0; // must be lower than max Speed
float maxSpeed = 10.0;
float accelRate = 1.0;

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
#define WHEELRADIUS 0.0762
#define BETWEENWHEELS 0.1524

#define desiredMeters 0.305 * 5 //input meters
float desiredDistance = 3200 * (1/(WHEELRADIUS*2*PI)) * desiredMeters * 0.18; //counts


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
  timeKept = millis();
  double duration = timeKept - prev1Time;
  //makes sure that there are no infinite velocities
  if(duration == 0.0) return;
  //the value of pi in arduino is M_PI
  double v = WHEELRADIUS*2*2*PI/(ROTATIONCOUNTS * duration);
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
  prev1Time = timeKept;

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
  timeKept = millis();
  double duration = timeKept - prev2Time;
  //make sure there are no infinite velocities that result in nan
  if(duration == 0.0) return;
  //the value of pi in arduino is M_PI
  double v = WHEELRADIUS*2*2*PI/(ROTATIONCOUNTS * duration);
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
  prev2Time = timeKept;
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
    leftPWMOut = constrain(leftPWMOut, -255, 255);
    rightPWMOut = constrain(rightPWMOut, -255, 255);

    // write values to pins
    digitalWrite(leftMotorDir, leftMoveDir);
    analogWrite(leftMotorPow, abs(leftPWMOut));
    digitalWrite(rightMotorDir, rightMoveDir);
    
    analogWrite(rightMotorPow, abs(rightPWMOut));
      //delay(deltaT);
    //Serial.println("lPWM: " + String(leftPWMOut) + " RPWM: " + String(rightPWMOut) + "\nrightErr: " + String(rightError) + " leftErr: " + String(leftError));
   
    if ((enc1Counts >= DISTANCE && enc1Counts <= DISTANCE + 30) && (leftVOut == 0 || rightVOut == 0)) {
      //Serial.println("here");
      return ;
    }

  }

  //analogWrite(leftMotorPow, 0);
  //analogWrite(rightMotorPow, 0);
  
}

//sets pinA and pinB to pullup inputs, attaches the isr to pinA/clk, and sets baud rate to 9600
void setup() {
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
    Serial.println(phi);
    //sprintf(buffer, "X position: %s, Y Position: %s, Phi: %s", printxbuffer, printybuffer, printphibuffer);
    //Serial.println(buffer);
    isrEntered = 0;
    // Serial.println("isr");
  }
  


  // // serial
  float angleRecieved = 60;
  int ldir = (angleRecieved >= 0) ? -1 : 1; 
  int rdir = -ldir;
  int turnCounts = angleToCounts(angleRecieved);
  move(turnCounts, ldir, rdir);
  // delay(100);
  //Serial.println("desD: " +String(desiredDistance));
  //move(desiredDistance, FORW, FORW);
  //Serial.println("done");

  //wait for button press
  while (digitalRead(pushButton) == 1); // wait for button push
  while (digitalRead(pushButton) == 0); // wait for button release
  // Serial.println("button");
  
}
