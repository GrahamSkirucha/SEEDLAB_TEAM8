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
double x = 0; //calculated x position from both encoder readings
double y = 0; //calculated y position from both encoder readings
double phi = 0.0; //calculated angle from both encoder readings assuming the starting orientation is 0 degrees
#define WHEELRADIUS 0.0762 //constant wheel radius (calculates velocity)
#define BETWEENWHEELS 0.1524 //constant distance between the wheels (calculates phi)
#define ROTATIONCOUNTS 3000 //encoders are 3000 counts per every 360 degrees
#define FEETTOMETERS 0.3048 //conversion factor from feet provided by the user to meters that the robot can measure

double desiredFeet = 5; //desired distance in feet - will be converted to meters
double desiredDistance; //desired distance in meters
double distanceFudgeFactor = 0.07; //fudge factor so that the robot moves the desired distance - scales with distance
double distWithFudge = 0; //distance the robot will use as a reference to drive to
int motor1Speed = 0; //value the mc motor shield library uses to apply a voltage to motor 1
int motor2Speed = 0; //value the mc motor shield library uses to apply a voltage to motor 2
double controllerThreshold = 0.1; //distance away from the destination that the controller will kick in
int printOnce = 0; //flag for testing the final distance the robot travelled and comparing it to the input distance
int motorMax = 350; //determines the maximum value the motor1Speed and motor2Speed variables can reach - value the controller uses to decrease the speed of the robot
int motorDecayFactor = motorMax * 10;

//I was gonna use this PID controller library to control the robot but I did not find it useful - maybe you guys can pull it out of the ashes to use it something for later
//PID Global Variables
// #include <PID_v1.h>
// PID myPID(&x,&leftPWMOut,&desiredDistance, lkp,lki,lkd,DIRECT);

//Dual MC Motor Shield moves the motors
#include "DualMC33926MotorShield.h"
DualMC33926MotorShield md;

//motor will stop if there is a fault - code copied from the demo example
// void stopIfFault()
// {
//   if (md.getFault())
//   {
//     Serial.println("fault");
//     while(1);
//   }
// }

//sets the necessary variables for the encoders, mc motor shield library, fudge factors, and Serial
void setup() {
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
  desiredDistance = desiredFeet * FEETTOMETERS;
  distWithFudge = desiredDistance + desiredDistance * distanceFudgeFactor;
  // ---
  //Serial
  Serial.begin(115200);
}

//reads encoders and writes to motors (only moves in a straight line for now)
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
    //set the prev* variables so that the next loop will use these values as a reference
    prevRead1 = read1;
    prevRead2 = read2;
    prevTime = timeKept;
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
    printOnce = 1; //exit code for debugging purposes
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
  if(phi > 0){
    motor2Speed += 2; //positive angle means the right motor needs to go faster
  }
  if(phi < 0){
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
  // Serial.println(int(x * 10000));
  // if(printOnce){
  //   Serial.println(int(distWithFudge * 10000));
  //   exit(0);
  // } 
  // Serial.println("left speed: " + String(motor1Speed) + "\tright speed: " + String(motor2Speed));
  // Serial.println("Left Velocity: " + String(leftVelocity) + "\tRight Velocity: " + String(rightVelocity));
  // Serial.println("Motor 1: " + String(motor1Speed) + "\tMotor 2: " + String(motor2Speed));
  // Serial.println("cm: " + String(int(x * 100)));
}

