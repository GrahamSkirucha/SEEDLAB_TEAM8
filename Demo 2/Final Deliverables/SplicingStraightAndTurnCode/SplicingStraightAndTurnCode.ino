/*  Demo 2 Arduino sketch
    Authors: Jaron O'Grady, Ryan Sims, Graham Skirucha, Maximus Waring, and Ryan Young
*/

//Encoder.h library used to read encoders
#include <Encoder.h>
#define ENCODER_USE_INTERRUPTS
//two encoder objects which correspond to each wheel's pinout
Encoder enc1 (2, 5);
Encoder enc2 (3, 6);
//variables stored and changed with information from the encoders
int prevRead1; //previous count for first encoder object - used by readEncoders() - set by readEncoders() and setup()
int prevRead2; //previous count for second encoder object - used by readEncoders() - set by readEncoders() and setup()
int timeKept; //holds the present time in the loop - used by readEncoders() - set by readEncoders()
int prevTime; //holds the previous time the encoders were read - used by readEncoders - set by readEncoders()
int duration; //determines the time between encoder readings - used by readEncoders - set by readEncoders()
double leftSpeed; //nubmer of counts per second on the left/first encoder - set by readEncoders() - maybe used later if we develop a function based on counts instead of velocity
double rightSpeed; //nubmer of counts per second on the right/second encoder - set by readEncoders() - maybe used later if we develop a function based on counts instead of velocity
double leftVelocity; //approximation of the velocity of left/first encoder - used by readEncoders() and stabilizeVelocity() - set by readEncoders()
double rightVelocity; //approximation of the velocity of the right/second encoder - used by readEncoders() and stabilizeVelocity() - set by readEncoders()
double x = 0; //calculated x position from both encoder readings - used by readEncoders(), forward(), and maxVelocityController() - set by clear() and readEncoders()
double y = 0; //calculated y position from both encoder readings - used by readEncoders() - set by readEncoders() and clear()
double phi = 0.0; //calculated angle from both encoder readings assuming the starting orientation is 0 degrees - used by readEncoders(), turn(), stabilizeVelocity(), and maxVelocityController() - set by readEncoders() and clear()
#define WHEELRADIUS 0.0762 //constant wheel radius (calculates velocity) - used by readEncoders()
#define BETWEENWHEELS 0.2713 //constant distance between the wheels (calculates phi) - used by readEncoders()
#define ROTATIONCOUNTS 3000 //encoders are 3000 counts per every 360 degrees - used by readEncoders()
#define FEETTOMETERS 0.3048 //conversion factor from feet provided by the user to meters that the robot can measure - used by setDistance()
const double CONVERTDEGREESRADIANS = PI / 180.0; //conversion factor from degrees provided by the user to radians that the robot can measure - used by setDegrees()

//DualMC33926MotorShield.h library used to move the wheels
#include "DualMC33926MotorShield.h"
//one motor shield object controls both wheels with a default pinout
DualMC33926MotorShield md;
//motor controller variables
int motor1Speed = 0; //value the mc motor shield library uses to apply a voltage to motor 1 - used by clear(), idleSettings(), turn(), forward(), and stabilizeVelocity() - set by clear(), turnSettings(), forwardSettings(), idleSettings(), and stabilizeVelocity()
int motor2Speed = 0; //value the mc motor shield library uses to apply a voltage to motor 2 - used by clear(), idleSettings(), turn(), forward(), and stabilizeVelocity() - set by clear(), turnSettings(), forwardSettings(), idleSettings(), and stabilizeVelocity()
double controllerThreshold = 0.1; //distance away from the destination that the controller will kick in - used by maxVelocityController()
int motorMax = 150; //determines the maximum value the motor1Speed and motor2Speed variables can reach - used by turnSettings(), forwardSettings(), idleSettings(), and stabilizeVelocity() - set by turnSettings(), forwardSettings(), idleSettings(), and maxVelocityController()
int motorDecayFactor = motorMax * 10; //determines how quickly the motorMax value will decay according to the maxVelocityController() function - used by maxVelocityController() - set by turnSettings(), forwardSettings(), and idleSettings()
int printOnce = 0; //flag for testing the final distance the robot travelled and comparing it to the input distance - used by loop() for printline debugging

//flags for driveMotors()
int turnFlag = true; //flag which indicates that the robot should be executing turn() - used by driveMotors(), stabilizeVelocity(), and maxVelocityController() - set by turn(), turnInstruction(), forwardInstruction(), and serialEvent()
int turnLeft = true; //flag which indicates that the robot turns left or right - used by turn() and stabilizeVelocity() - set by setDegrees()
int forwardFlag = false; //flag which indicates that the robot should be moving forward - used by driveMotors() - set by forward(), turnInstruction(), forwardInstruction(), and serialEvent()
int transitionFlag = false; //flag which indicates what to do next given some instruction - used by transition() and driveMotors() - set by turn(), forward(), transition(), and serialEvent()

//angle calculating variables
float defaultDegrees = 45.0; //default degrees to send to the robot to start it turning - used by setup()
double phiAndPhudge; //angle in radians the robot will use as a reference to drive to - used by setDegrees(), turn(), and maxVelocityController() - set by setDegrees()

//distance  calculating variables
float defaultDistance = 2.0; //desired distance in feet - will be converted to meters - used by setup()
double distWithFudge; //distance the robot will use as a reference to drive to - used by forward() and maxVelocityController() - set by setDistance()

//variables changed and used after a pi serial input (affects transition() method)
int scanFlag = true; //flag which indicates that the robot has identified the marker and is ready to move toward it - used by instruction() - set by serialEvent()
int instructionIndex = 0; //flag which determines which instruction to complete in the instruction() state machine - used by instruction() - set by instruction(), serialEvent(), and setup()
float degreesFromSerial; //degree value that the robot needs to turn to face the marker - used by instruction() - set by serialEvent() 
float distanceFromSerial; //distance value that the robot needs to turn to face the marker - used by instruction() - set by serialEvent()
int numberEvents = 2; //constant which determines how many steps the robot needs to take before it stops listening to the serial - used by serialEvent()
int currentEvent; //counter which counts the number of serial events left to read - used by serialEvent() - set by serialEvent() and setup()
//additional settings for optimizing speed and cv effectiveness
#define SCAN_DELAY 700 //amount of delay between instructions while the robot is scanning for the marker - used by setup()
#define TARGET_DELAY 150 //amount of delay between instructions while the robot is moving toward the target - used by serialEvent()
int currentDelay; //variable which switches between the scan and target delay values to put the robot in two distinct states - used by transition() - set by serialEvent() and setup()

//I was gonna use this PID controller library to control the robot but I did not find it useful - maybe you guys can pull it out of the ashes to use it something for later
//PID Global Variables
// #include <PID_v1.h>
// PID myPID(&x,&leftPWMOut,&variableToControl, lkp,lki,lkd,DIRECT);

//main functions
/*  setup()
    - sets the starting values for encoders, mc motor shield library, robot movement settings, and Serial
*/
void setup() {
  //encoder reading setup
  prevRead1 = 0;
  prevRead2 = 0;
  //motor control setup
  md.init();
  turnSettings();
  //boot settings
  setDegrees(defaultDegrees);
  setDistance(defaultDistance);
  currentDelay = SCAN_DELAY;
  instructionIndex = 0;
  currentEvent = 0;
  //Serial
  Serial.begin(115200);
}

/*  loop()
    - uses readEncoders() to read the encoders and adjust global variables
    - uses driveMotors() to enter a finite state machine that moves the motors according to flags
    - uses stabilizeVelocity() to ensure that the motors are moving at the same speed and correcting for small errors
    - uses maxVelocityController() to slow down the robot to a stop at the appropriate distance or angle
    - has optional printline statements that can be commented or uncommented for debugging purposes
*/
//reads encoders and writes to motors (only moves in a straight line for now)
void loop() {
  readEncoders();
  driveMotors();
  stabilizeVelocity();
  maxVelocityController();

  // printline debugging
  // if(turnFlag){
  //   Serial.println(int(phi * 10000));
  // }
  // if(printOnce){
  //   Serial.println(int(distWithFudge * 10000));
  //   exit(0);
  // } 
  // Serial.println("left speed: " + String(motor1Speed) + "\tright speed: " + String(motor2Speed));
  // Serial.println("Left Velocity: " + String(leftVelocity) + "\tRight Velocity: " + String(rightVelocity));
  // Serial.println("Motor 1: " + String(motor1Speed) + "\tMotor 2: " + String(motor2Speed));
  // Serial.println("cm: " + String(int(x * 100)));
}

/*  serialEvent()
    - checks to see if the serial has surpassed the number of instructions the code will listen to
    - extracts the string from the serial and converts them to floats to edit degreesFromSerial and distanceFromSerial
    - sets scanFlag, turnFlag, forwardFlag, and transitionFlag to values that ensure that the robot will immediately transition to moving toward the marker
    - sets the instruction index to the first instruction listed in instruction() (it is indexed at 1, sorry)
    - sets the delay variable to the target value so that the robot moves faster
    - ends serial communication (right now it kills it forever, but for the next project it will end it and then reopen serial, adjusting settings accordingly) 
*/
void serialEvent() {
  if(currentEvent < numberEvents){
    if (Serial.available() > 0) {
      String totalString = Serial.readStringUntil('\n');
      String degString = totalString.substring(0,totalString.indexOf(','));
      String distString = totalString.substring(totalString.indexOf(',') + 1);
      degreesFromSerial = degString.toFloat();
      //degreesFromSerial = degreesFromSerial + *fudge*; //adding a fudge factor to degreesFromSerial in the event our CV needs a consistent fudge
      distanceFromSerial = distString.toFloat();
      // Serial.println(degString);
      // Serial.println(distString);
      scanFlag = false;
      turnFlag = false;
      forwardFlag = false;
      transitionFlag = true;
      instructionIndex ++;
      currentDelay = TARGET_DELAY;
      currentEvent ++;
      Serial.end();
    }
    Serial.flush();
  }
}

/*  setDegrees(degIn)
    - takes in a degree value and converts it into a radian value with a fudge factor - value stored in phiAndPhudge
    - if the degree value is negative, the turnLeft flag will be set to false so that the robot turns in the correct direction
*/
void setDegrees(float degIn){
  double desiredPhi = degIn * CONVERTDEGREESRADIANS;
  double degreeFudgeFactor = 0.0;
  phiAndPhudge = desiredPhi + desiredPhi * degreeFudgeFactor;
  if(phiAndPhudge < 0){
    turnLeft = false;
  }
  else{
    turnLeft = true;
  }
}

/*  setDistance(distIn)
    - takes in a distance value and converts it into a meter value with a fudge factor - value stored in distWithFudge
*/
void setDistance(float distIn){
  double desiredDistance = distIn * FEETTOMETERS;
  double distanceFudgeFactor = 0.1;
  distWithFudge = desiredDistance + desiredDistance * distanceFudgeFactor; //needed for forward() and maxVelocityController()
}

/*  clear()
    - reset global position variables
    - reset motor speed variables
    - set the motor speeds to zero
    - send a speed of zero to the motors
*/
void clear(){
  x = 0;
  y = 0;
  phi = 0;
  motor1Speed = 0;
  motor2Speed = 0;
  md.setM1Speed(motor1Speed);
  md.setM2Speed(motor2Speed);
  delay(2);
}

/*  turnSettings()
    - global motorMax slower for less overshoot and less potential mechanical error from the back wheel
    - setting motorDecayFactor accordingly for the maxVelocityController() later
    - setting motor1Speed and motor2Speed to arbitrary default values below the max - these values will be changed by stabilizeVelocity()
*/
void turnSettings(){
  motorMax = 300;
  motorDecayFactor = motorMax * 10;
  motor1Speed = 280;
  motor2Speed = 290;
}

/*  forwardSettings()
    - global motorMax faster for faster movement due to lower potential for overshoot
    - setting motorDecayFactor accordingly for the maxVelocityController() later
    - setting motor1Speed and motor2Speed to arbitrary default values below the max - these values will be changed by stabilizeVelocity()
*/
void forwardSettings(){
  motorMax = 350;
  motorDecayFactor = motorMax * 10;
  motor1Speed = 300;
  motor2Speed = 340;
}

/*  idleSettings()
    - setting global variables motorMax, motorDecayFactor, motor1Speed, and motor2Speed to zero in case something goes wrong with the state machine - partly defensive coding
    - sends a value of zero to the motors in case they have not fully stopped - also partly defensive coding
*/
void idleSettings(){
  motorMax = 0;
  motorDecayFactor = motorMax * 10;
  motor1Speed = 0;
  motor2Speed = 0;
  md.setM1Speed(motor1Speed);
  md.setM2Speed(motor2Speed);
  delay(2);
}

/*  readEncoders()
    - takes readings from each encoder object
    - determines whether the encoder readings are different - if they are, the following code is executed:
      - record the number of changes in counts
      - check the time and performs a duration calculation to determine speeds
      - set the values of leftVelocity, leftSpeed, rightVelocity, rightSpeed, using speed calculations and physical characteristics of the robot
      - determine the changes in position using values in the block diagram of our simulink code created in an earlier assignment
      - record changes in position in global variables x, y, and phi
      - set bounds on phi - defensive coding (phi should never go above 2PI or below -2PI)
      - set global variables prevRead1, prevRead2, and prevTime to accurately calculate the changes in counts and time the on the next call to readEncoders()
*/
void readEncoders(){
  int read1 = enc1.read();
  int read2 = enc2.read();
  if((read1 != prevRead1) || (read2 != prevRead2)){
    int change1 = read1 - prevRead1;
    int change2 = prevRead2 - read2; //check that the robot is correctly computing positive velocities!!!
    //take the time and calculate the loop time
    timeKept = millis();
    duration = timeKept - prevTime;
    double durationSec = duration / 1000.0;
    leftVelocity = WHEELRADIUS*change1*2*PI/(ROTATIONCOUNTS * durationSec);
    // leftSpeed = change1/durationSec; //left commented in case another algorithm needs these values later
    rightVelocity = WHEELRADIUS*change2*2*PI/(ROTATIONCOUNTS * durationSec);
    // rightSpeed = change2/durationSec; //left commented in case another algorithm needs these values later
    double deltaX = durationSec * cos(phi) * (leftVelocity + rightVelocity) / 2;
    double deltaY = durationSec * sin(phi) * (leftVelocity + rightVelocity) / 2;
    double deltaPhi = durationSec * (leftVelocity - rightVelocity) / BETWEENWHEELS; //this is impacted massively by the accuracy of the BETWEENWHEELS constant
    x = x + deltaX;
    y = y + deltaY;
    phi = phi + deltaPhi;
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
}

/*  turn()
    - if the motor speed values being sent to the motors are low enough, the robot is sent to transition state
    - if the angle is off, the robot sends the values of motor1Speed and motor2Speed to the motors with the sign flipped on one value to ensure a turn
*/
void turn(){
  if((motor1Speed <= 10) || (motor2Speed <= 10)){
    turnFlag = false;
    transitionFlag = true; 
  }
  if(abs(phi) < abs(phiAndPhudge)){
    if(turnLeft){
      md.setM1Speed(motor1Speed);
      md.setM2Speed(-motor2Speed);
    }
    else{
      md.setM1Speed(-motor1Speed);
      md.setM2Speed(motor2Speed);
    }
    delay(2);
  }
}

/*  forward()
    - if the motor speed values being sent to the motors are low enough, the robot is sent to transition state
    - if the distance is off, the robot sends the values of motor1Speed and motor2Speed to the motors
*/
void forward(){
  if((motor1Speed <= 10) || (motor2Speed <= 10)){
    forwardFlag = false;
    transitionFlag = true;
  }
  if(x < distWithFudge){
    md.setM1Speed(motor1Speed);
    md.setM2Speed(motor2Speed);
    delay(2);
  }
}

/*  transition()
    - if the transition flag is set the following code executes (again some more defensive coding):
    - uses clear() to clear global variables and stop the motors
    - uses instruction() to set settings for the next robot action (or inaction)
    - resets transitionFlag to false so the code is only executed once at the end of every action
    - delays the robot so that it has time between scanning or moving to complete the next action without overshoot or inaccuracy
*/
void transition(){
  if(transitionFlag){
    clear();
    instruction();
    transitionFlag = false;
    delay(currentDelay);
  }
}

/*  instruction()
    - if the robot is set to scanning, the robot configures settings to a 45 degree turn instruction until a serial event changes the scanFlag
    - if the robot is moving toward the target, the robot determines which instruction to execute based on the number of instructions it has already executed (determined by instructionIndex)
    - if the robot is not scanning and has completed all of the move instructions - the robot's settings are configured to idle
*/
void instruction(){
  if(scanFlag){
    turnInstruction(45); //default state
  }
  else{ //finite state machine
    if(instructionIndex == 1){
      turnInstruction(degreesFromSerial);
      instructionIndex ++;
    }
    else if(instructionIndex == 2){
      forwardInstruction(distanceFromSerial);
      instructionIndex ++;
    }
    else{
      idleSettings();
    }
  }
}

/*  turnInstruction(degIn)
    - sets flags so that the robot executes a turn when the driveMotors() function is called
    - uses turnSettings() to make sure the robot goes slower
    - uses setDegrees() to set the next angle the robot must turn toward
*/
void turnInstruction(float degIn){
  turnFlag = true;
  forwardFlag = false;
  turnSettings();
  setDegrees(degIn);
}

/*  forwardInstruction(distIn)
    - sets flags so that the robot moves forward when the driveMotors() function is called
    - uses forwardSettings() to make sure the robot goes faster
    - uses setDistance() to set the next distance the robot must move toward
*/
void forwardInstruction(float distIn){
  turnFlag = false;
  forwardFlag = true;
  forwardSettings();
  setDistance(distIn);
}

/*  driveMotors()
    - implements a state machine determined by flags to make the robot turn, move forward, or transition
*/
void driveMotors(){
  if(turnFlag){
    turn();
  }
  if(forwardFlag){
    forward();
  }
  if(transitionFlag){
    transition();
  }
}

/*  stabilizeVelocity()
    - separates code into 'turning' and 'moving straight' blocks which both stabilize the velocity in slightly different ways
    - separates code in the 'turning' block into 'left turn' and 'right turn' blocks which adjust different motors depending on the same difference in velocity
    - in each block:
      - the velocity difference is calculated
      - the motor speed values are adjusted (and overcorrected) to account for small differences in angle and displacement
      - the motor speed values are reduced so that they remain under the threshold of the motorMax variable 
*/
void stabilizeVelocity(){
  //'moving straight' block
  if(forwardFlag){
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
  }
  //'turning' block
  if(turnFlag){
    //'left turn' block
    if(turnLeft){
      double velocityDiff = leftVelocity + rightVelocity;
      //place to add extra motor speed like forward stabilizer
      if(velocityDiff > 0){
        motor2Speed += 10;
      }
      else if(velocityDiff < 0){
        motor1Speed += 10;
      }
      //place to correct changes in x and y
      while((motor1Speed > motorMax) || (motor2Speed > motorMax)){
        motor1Speed -= 10;
        motor2Speed -= 10;
      } 
    }
    //'right turn' block
    else{
      double velocityDiff = leftVelocity + rightVelocity;
      //place to add extra motor speed like forward stabilizer
      if(velocityDiff < 0){
        motor2Speed += 10;
      }
      else if(velocityDiff > 0){
        motor1Speed += 10;
      }
      //place to correct changes in x and y
      while((motor1Speed > motorMax) || (motor2Speed > motorMax)){
        motor1Speed -= 10;
        motor2Speed -= 10;
      } 
    }
  }
}

/*  maxVelocityController()
    - separates the code into 'turn' and 'straight' blocks
    - for each block:
      - the distance or angle is checked to see if it is within a threshold, and then executes the following:
      - the error in the max speed is calculated using the desired distance/angle, the current distance/angle, and a motorDecayFactor
      - this error subtracts the motorMax to lower the speed of the robot on the next iteration of stabilizeVelocity()
*/
void maxVelocityController(){
  if(turnFlag){
    if(abs(phi) > abs(phiAndPhudge) - controllerThreshold){
      double error = motorDecayFactor * (abs(phiAndPhudge) - abs(phi));
      motorMax = int(error);
    }
  }
  else{
    if(x > distWithFudge - controllerThreshold){
      double error = motorDecayFactor * (distWithFudge - x);
      motorMax = int(error);
    }    
  }
}