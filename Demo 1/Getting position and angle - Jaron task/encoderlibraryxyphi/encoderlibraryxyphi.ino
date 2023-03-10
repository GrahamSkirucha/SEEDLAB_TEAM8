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
float x;
float y;
float phi;
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
  int read1 = enc1.read();
  int read2 = enc2.read();
  if((read1 != prevRead1) || (read2 != prevRead2)){
    //read the encoders
    //take the time and calculate the loop time
    timeKept = millis();
    duration = timeKept - prevTime;
    float durationSec = duration / 1000.0;
    //calculate velocity using encoder counts and duration with constants
    leftSpeed = WHEELRADIUS*read1*2*PI/(ROTATIONCOUNTS * durationSec);
    rightSpeed = WHEELRADIUS*read2*2*PI/(ROTATIONCOUNTS * durationSec);
    //calculate the changes in position
    float deltaX = durationSec * cos(phi) * (leftSpeed + rightSpeed) / 2;
    float deltaY = durationSec * sin(phi) * (leftSpeed + rightSpeed) / 2;
    float deltaPhi = durationSec * (leftSpeed - rightSpeed) / BETWEENWHEELS;
    //calculate new position
    x = x + deltaX;
    y = y + deltaY;
    phi = phi + deltaPhi;
    //print everything
    Serial.print("Left: ");
    Serial.print(leftSpeed);
    Serial.print("\t");
    Serial.print("Right: ");
    Serial.print(rightSpeed);
    Serial.print("\t");
    Serial.print("X: ");
    Serial.print(x);
    Serial.print("\t");
    Serial.print("Y: ");
    Serial.print(y);
    Serial.print("\t");
    Serial.print("Phi: ");
    Serial.println(phi);
    //update prev* variables
    prevRead1 = read1;
    prevRead2 = read2;
    prevTime = timeKept;
  }
  
}
