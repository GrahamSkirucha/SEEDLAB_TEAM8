#include <DualMC33926MotorShield.h>

DualMC33926MotorShield md;

// Pin A and pin B define the CLK and DT outputs of the encoder, respectively
const int PIN_A = 2;
const int PIN_B = 3;

// This program uses a state machine to track the encoder’s position
bool A_LAST_STATE = 0;
bool A_CURRENT_STATE = 0;
bool B_CURRENT_STATE = 0;

// Position of the encoder is stored in this variable 
// +1 represents 1 tick clockwise and -1 represents 1 tick counterclockwise
int degree = 0;
int beginTime = 0;
int currMyTime = 0;
int prev_pos = 0;
double motorVal = 0;
double current_pos = 0;
int wait = 0;
bool start = 1;

void setup() {
  
  pinMode(PIN_A, INPUT);
  pinMode(PIN_B, INPUT);

  Serial.begin(115200);
  
  // Setting up the interrupt handler 
  // Activates whenever the CLK pin on the encoder changes voltage
  // I arbitrarily chose the CLK pin, I could have also used the DT pin
  attachInterrupt(digitalPinToInterrupt(2), TICK, CHANGE);
  md.init();
  delay(500);
  wait = millis();
  beginTime = wait;
}

void loop() {
// Nothing happens in the main loop due to interrupt handler
  if(start && ((wait-beginTime) >995)){
    md.setM1Speed(1024);
    //beginTime = millis();
    start = 0;
  }
  else if (start)wait = millis();
  current_pos = double(degree)/4.44;//2.22
  currMyTime = millis() - beginTime;
 
  Serial.print(currMyTime);
  Serial.print(", ");
  Serial.print(current_pos);
  Serial.print('\n');//*/
 
  delay(5);
  
  if (millis() >=3000+wait){
    md.setM1Speed(0);
    while(1);
  }
}


// Interrupt handler
void TICK() {

  A_CURRENT_STATE = digitalRead(PIN_A);//read state of signal A
  B_CURRENT_STATE = digitalRead(PIN_B);//read state of signal B
  
  if (A_CURRENT_STATE==B_CURRENT_STATE){ //if the signals are the same

    degree-=1;//subract from the count
  }
  else {//if signals are different
    degree+=1;//add 2 to the count
  }
}
  
