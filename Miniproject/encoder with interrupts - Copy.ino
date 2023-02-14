/*
Author: Jaron O'Grady
This code records the net number of encoder rotation increments clockwise using external interrupts.
*/
//global variables include encoder pins, the states of the encoder pins, the net count of clockwise increments
int pinA = 2; //clk
int pinB = 3; //dt
int stateA = 0;
int stateB = 0;
int count = 0;
int isrEntered = 0;

//isr measures the state of the pins and adjusts the counts and isr flag - only entered once pinA changes
void isr(){
  //read the pins
  int stateA = digitalRead(pinA);
  int stateB = digitalRead(pinB);
  //determine clockwise or counter-clockwise
  if(stateA == stateB){
    count += 2; //clockwise
  }
  else{
    count -= 2; //counter-clockwise
  }
  //adjust the flag to print the new value of counts in the loop
  isrEntered = 1;
}

//sets pinA and pinB to pullup inputs, attaches the isr to pinA/clk, and sets baud rate to 9600
void setup() {
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  attachInterrupt(0, isr, CHANGE);
  Serial.begin(9600);
}

//prints the net counts given there has been a change in pinA
void loop() {
  //sei(); - uncomment if revisiting to see if this fixes the problem of not always seeing rotations/bouncing
  if(isrEntered){
    //sei(); - uncomment if revisiting to see if this fixes the problem of not always seeing rotations/bouncing
    Serial.print("Count: ");
    Serial.println(count);
    isrEntered = 0;
  }
}

