//copied from one of the example codes in the zip file
#include <Encoder.h>
//make sure the encoder is connected to 2 and 3
#define ENCODER_USE_INTERRUPTS
Encoder myEnc(2,3);
int lastRead;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  lastRead = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  int encoderRead = myEnc.read();
  if(encoderRead != lastRead){
    Serial.println(encoderRead);
    lastRead = encoderRead;
  }
  
}
