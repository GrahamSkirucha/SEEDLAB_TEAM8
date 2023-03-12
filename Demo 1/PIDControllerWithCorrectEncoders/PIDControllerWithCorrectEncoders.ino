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
#define FEETTOMETERS 0.3048

double desiredFeet = 3; //10 ft
double desiredDistance;
double distanceFudgeFactor = 0.05;
double distWithFudge = 0;
int motor1Speed = 0;
int motor2Speed = 0;
double halveAt = 0.9;

//PID Global Variables
// #include <PID_v1.h>
// PID myPID(&x,&leftPWMOut,&desiredDistance, lkp,lki,lkd,DIRECT);

//Dual MC Motor Shield moves the motors
#include "DualMC33926MotorShield.h"
DualMC33926MotorShield md;
//motor will stop if there is a fault
// void stopIfFault()
// {
//   if (md.getFault())
//   {
//     Serial.println("fault");
//     while(1);
//   }
// }

//sets pinA and pinB to pullup inputs, attaches the isr to pinA/clk, and sets baud rate to 9600
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
  halveAt = distWithFudge * halveAt;
  // ---
  //Serial
  Serial.begin(115200);
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
  if(x < distWithFudge){
    md.setM1Speed(motor1Speed);
    md.setM2Speed(motor2Speed);
    delay(2);
  }
  else{
    md.setM1Speed(0);
    md.setM2Speed(0);
    delay(2);
  }
  double velocityDiff = leftVelocity - rightVelocity;
  if(velocityDiff > 0){
    motor2Speed += 10;
  }
  else if(velocityDiff < 0){
    motor1Speed += 10;
  }
  if(phi > 0){
    motor2Speed += 1;
    // desiredDistance += sin(phi);
  }
  if(phi < 0){
    motor1Speed += 1;
    // desiredDistance -= sin(phi);
  }
  if((motor1Speed > 350) || (motor2Speed > 350)){
    motor1Speed -= 10;
    motor2Speed -= 10;
  }
  Serial.println("left speed: " + String(motor1Speed) + "\tright speed: " + String(motor2Speed));
  // Serial.println("Left Velocity: " + String(leftVelocity) + "\tRight Velocity: " + String(rightVelocity));
  // Serial.println("Motor 1: " + String(motor1Speed) + "\tMotor 2: " + String(motor2Speed));
  // Serial.println("cm: " + String(int(x * 100)));
}

