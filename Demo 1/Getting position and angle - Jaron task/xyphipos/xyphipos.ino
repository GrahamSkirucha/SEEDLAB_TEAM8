/*
Author: Jaron O'Grady
This code records velocity using external interrupts and then calculates position.
*/
//global variables include encoder pins, the states of the encoder pins, the net count of clockwise increments
int e1pinA = 2; //clk encoder 1
int e1pinB = 4; //dt encoder 1
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
double xpos = 0.0;
double ypos = 0.0;
double phi = 0.0;
int isrEntered = 0;
#define ROTATIONCOUNTS 80
#define DEBOUNCE 10

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
  double v = wheelRadius*2*2*PI/(ROTATIONCOUNTS * duration);
  if(e1stateA == e1stateB){
    velocity1 = v;
  }
  else{
    velocity1 = -1.0 * v;
  }
  //update the position variables
  xpos = xpos + duration * cos(phi) * (velocity1 + velocity2) / 2;
  ypos = ypos + duration * sin(phi) * (velocity1 + velocity2) / 2;
  phi = phi + duration * (velocity1 - velocity2) / betweenWheels;
  //makes sure that phi stays in the range of -2PI to 2PI
  if((phi > 2 * PI) || (phi < -2 * PI)){
    phi = 0;
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
  timeKept = millis();
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
  //update the position variables
  xpos = xpos + duration * cos(phi) * (velocity1 + velocity2) / 2;
  ypos = ypos + duration * sin(phi) * (velocity1 + velocity2) / 2;
  phi = phi + duration * (velocity1 - velocity2) / betweenWheels;
  //makes sure that phi stays in the range of -2PI to 2PI
  if((phi > 2 * PI) || (phi < -2 * PI)){
    phi = 0;
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
    char buffer[144];
    char printxbuffer[8];
    char printybuffer[8];
    char printphibuffer[8];
    dtostrf(xpos, 4, 4, printxbuffer);
    dtostrf(ypos, 4, 4, printybuffer);    
    dtostrf(phi, 4, 4, printphibuffer);
    sprintf(buffer, "X position: %s, Y Position: %s, Phi: %s", printxbuffer, printybuffer, printphibuffer);
    Serial.println(buffer);
    isrEntered = 0;
  }
}


