/*
Author: Jaron O'Grady
This code records velocity and time.
*/
//global variables include encoder pins, the states of the encoder pins, the net count of clockwise increments
//copied from one of the example codes in the zip file
//make sure the encoder is connected to 2 and 3
int e1pinA = 2; //clk encoder 1
int e1pinB = 4 //dt encoder 1
int e2pinA = 3; //clk encoder 2
int e2pinB = 5; //dt encoder 2
int e1stateA = 0;
int e1stateB = 0;
int e2stateA = 0;
int e2stateB = 0;
double velocity1 = 0.0;
double velocity2 = 0.0;
double timeKept = 0.0;
double prev1Time = 0.0;
double prev2Time = 0.0;
double wheelRadius = 0.05;
double betweenWheels = 0.1;
//necessary only if we want to control position
double xpos = 0.0;
double ypos = 0.0;
double phi = 0.0;
//interrupt flag
int isrEntered = 0;
#define ROTATIONCOUNTS 3000
#define WHEELRADIUS 0.05
#define BETWEENWHEELS 0.1

//isr measures the state of the pins and adjusts the counts and isr flag - only entered once pinA changes
void encoder1(){
  //read the pins
  int e1stateA = digitalRead(e1pinA);
  int e1stateB = digitalRead(e1pinB);
  //determine clockwise or counter-clockwise
  timeKept = millis() / 1000.0;
  double duration = timeKept - prev1Time;
  //makes sure that there are no infinite velocities
  if(duration == 0.0) return;
  //the value of pi in arduino is M_PI
  double v = wheelRadius*2*2*PI/(ROTATIONCOUNTS * duration);
  if(e1stateA == e1stateB){
    velocity1 = v;
  }
  else{
    velocity1 = -1.0 * v;
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
  //record time in seconds
  timeKept = millis / 1000.0;
  double duration = timeKept - prev2Time;
  //make sure there are no infinite velocities that result in nan
  if(duration == 0.0) return;
  //the value of pi in arduino is M_PI
  double v = wheelRadius*2*2*PI/(ROTATIONCOUNTS * duration);
  if(e2stateA == e2stateB){
    velocity2 = v;
  }
  else{
    velocity2 = -1.0 * v;
  }
  //adjust the flag to print the new value of counts in the loop
  isrEntered = 1;
  prev2Time = timeKept;
}


//sets pinA and pinB to pullup inputs, attaches the isr to pinA/clk, and sets baud rate to 9600
void setup() {
  pinMode(e1pinA, INPUT_PULLUP);
  pinMode(e1pinB, INPUT_PULLUP);
  pinMode(e2pinA, INPUT_PULLUP);
  pinMode(e2pinB, INPUT_PULLUP);
  attachInterrupt(0, encoder1, CHANGE);
  attachInterrupt(1, encoder2, CHANGE);
  Serial.begin(9600);
}

//prints the net counts given there has been a change in pinA
void loop() {
  //sei(); - uncomment if revisiting to see if this fixes the problem of not always seeing rotations/bouncing
  if(isrEntered){
    //sei(); - uncomment if revisiting to see if this fixes the problem of not always seeing rotations/bouncing
    //link for how to print doubles and floating points: https://www.programmingelectronics.com/dtostrf/
    char buffer[100];
    char timebuffer[6];
    char lbuffer[7];
    char rbuffer[7];
    dtostrf(timeKept, 4, 3, timebuffer);
    dtostrf(velocity1, 4, 3, lbuffer);    
    dtostrf(velocity2, 4, 3, rbuffer);
    sprintf(buffer, "%s\t%s\t%s", timebuffer, lbuffer, rbuffer);
    Serial.println(buffer);
    isrEntered = 0;
  }
}