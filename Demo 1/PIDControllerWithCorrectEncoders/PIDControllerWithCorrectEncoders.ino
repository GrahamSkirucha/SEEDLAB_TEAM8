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

//PID control values
double lkp = 1.0;
double rkp = 1.0;
double lkd = 0.0;
double rkd = 0.0;
double lki = 0.0;
double rki = 0.0;

double desiredDistance = 5.0; //cm
int loopCount = 0;
int countsTillPrint = 200;

//PID Global Variables
// #include <PID_v1.h>
// PID myPID(&x,&leftPWMOut,&desiredDistance, lkp,lki,lkd,DIRECT);

//Dual MC Motor Shield moves the motors
#include "DualMC33926MotorShield.h"
DualMC33926MotorShield md;
//motor will stop if there is a fault
void stopIfFault()
{
  if (md.getFault())
  {
    Serial.println("fault");
    while(1);
  }
}

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
  // ---
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
  if(x < desiredDistance){
    md.setM1Speed(290);
    md.setM2Speed(300);
    // Serial.println("Set speed");
  }
  else{
    md.setM1Speed(0);
    md.setM2Speed(0);
    // Serial.println("Speed to zero");
  }
  loopCount ++;
  if(loopCount == countsTillPrint){ 
    double speedDiff = leftVelocity - rightVelocity;
    double distDiff = double(desiredDistance) - x;
         
    Serial.println(x);
    // Serial.println(x)    
    // Serial.println(speedDiff);
    // Serial.println(int(distDiff));
    loopCount = 0;    
  }

}

