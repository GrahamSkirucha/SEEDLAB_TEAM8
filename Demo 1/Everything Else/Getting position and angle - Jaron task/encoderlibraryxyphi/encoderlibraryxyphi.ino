//copied from one of the example codes in the zip file
#include <Encoder.h>
//make sure the encoder is connected to 2 and 3
#define ENCODER_USE_INTERRUPTS
Encoder enc1 (2, 5);
Encoder enc2 (3, 6);
int prevRead1;
int prevRead2;
int timeKept;
int prevTime;
int duration;
float leftSpeed;
float rightSpeed;
float leftVelocity;
float rightVelocity;
float x;
float y;
float phi = 0;
#define WHEELRADIUS 0.0762
#define BETWEENWHEELS 0.1524
#define ROTATIONCOUNTS 3000

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  prevRead1 = 0;
  prevRead2 = 0;
}

void loop() {
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
    float durationSec = duration / 1000.0;
    //calculate velocity using the change in encoder counts and duration with constants
    leftVelocity = WHEELRADIUS*change1*2*PI/(ROTATIONCOUNTS * durationSec);
    leftSpeed = change1/durationSec;
    rightVelocity = WHEELRADIUS*change2*2*PI/(ROTATIONCOUNTS * durationSec);
    rightSpeed = change2/durationSec;
    //calculate the changes in position
    float deltaX = durationSec * cos(phi) * (leftVelocity + rightVelocity) / 2;
    float deltaY = durationSec * sin(phi) * (leftVelocity + rightVelocity) / 2;
    float deltaPhi = durationSec * (leftVelocity - rightVelocity) / BETWEENWHEELS;
    //calculate new position
    x = x + deltaX;
    y = y + deltaY;
    phi = phi + deltaPhi;
    //set bounds on phi - if phi is greater than 2PI subtract 2PI, and if phi is less than -2PI add 2*PI
    if(phi > 2 * PI){
      Serial.println("Greater than 2PI");
      phi -= 2 * PI;
    }
    if(phi < (-2 * PI)){
      Serial.println("Less than 2PI");
      phi += 2 * PI;
    }
    //print everything
    Serial.print("Time: ");
    Serial.print(durationSec);
    Serial.print("\t");
    Serial.print("Left Counts: ");
    Serial.print(change1);
    Serial.print("\t");
    Serial.print("Right Counts: ");
    Serial.print(change2);
    Serial.print("\t");
    Serial.print("Left Counts per Second: ");
    Serial.print(leftSpeed);
    Serial.print("\t");
    Serial.print("Right Counts per Second: ");
    Serial.print(rightSpeed);
    Serial.print("\t");
    Serial.print("X: ");
    Serial.print(x);
    Serial.print("\t");
    Serial.print("Y: ");
    Serial.print(y);
    Serial.print("\t");
    Serial.print("Phi: ");
    Serial.print(phi);
    Serial.print("\tDelta phi: ");
    Serial.print(deltaPhi);
    //update prev* variables
    prevRead1 = read1;
    prevRead2 = read2;
    prevTime = timeKept;
    //take loop speed and print it
    int loopSpeed = millis() - timeKept;
    Serial.print("\tLoop Speed (ms): ");
    Serial.println(loopSpeed);
    // Serial.println(PI);
  }
  
}
