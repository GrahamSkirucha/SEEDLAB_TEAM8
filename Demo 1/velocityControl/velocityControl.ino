#define enablePin 4
#define leftMotorPow 7
#define rightMotorPow 8
#define leftMotorDir 9
#define rightMotorDir 10

#define lKp 1
#define rKp 1
#define lKd 0
#define rKd 0
#define lKi 0
#define rKi 0

#define decelRate 10
#define startSpeed 10
#define accelRate 10

int maxSpeed = 30;

bool leftMoveDir;
bool rightMoveDir;
float leftVOut;
float rightVOut;
float leftPWMOut;
float rightPWMOut;
int curTime;
int prevTime = 0;
float dt;

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
float desiredSpeed;

float xPos; // temp
float leftSpeed; // temp
float rightSpeed; // temp

void setup() {
  Serial.begin(9600);
  
  pinMode(leftMotorPow, OUTPUT);
  pinMode(rightMotorPow, OUTPUT);

  pinMode(leftMotorDir, OUTPUT);
  pinMode(rightMotorDir, OUTPUT); 

  digitalWrite(enablePin, HIGH); // set enable pin to high to enable motor driver
} 
   

void loop() {
  
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
  if (xPos < accelDist) {
    desiredSpeed = startSpeed + accelRate * xPos;
  }
  // Flat segment
  else if (xPos < flatDist + accelDist) {
    desiredSpeed = maxSpeed;
  }
  // Deceleration segment
  else if (xPos < flatDist + accelDist + decelDist) {
    desiredSpeed = maxSpeed - (decelRate * (xPos - flatDist));
  }
  // Too far
  else {
    desiredSpeed = 0;
  }

    
  curTime = millis();
  dt = (curTime - prevTime) / 1000;

  leftError = desiredSpeed - leftSpeed;
  rightError = desiredSpeed - rightSpeed;

  leftDerivative = (leftError - prevLeftError) / dt;
  rightDerivative = (rightError - prevRightError) / dt;

  leftIntegral += (leftError - prevLeftError) / 2 * dt;
  rightIntegral += (rightError - prevRightError) / 2 * dt;

  leftVOut = lKp * leftError + lKd * leftDerivative + lKi * leftIntegral;
  rightVOut = rKp * rightError + rKd * rightDerivative + rKi * rightIntegral;

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
  analogWrite(leftMotorPow, abs(leftPWMOut));
  digitalWrite(rightMotorDir, rightMoveDir);
  analogWrite(rightMotorPow, abs(rightPWMOut));
}
