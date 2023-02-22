#include <DualMC33926MotorShield.h>

DualMC33926MotorShield md;

int dutyCycle = 75;

void setup()
{
  md.init(); // initialize motor to stopped
  md.setM1Speed(0);
  md.setM2Speed(0);

  pinMode(4, OUTPUT); // define pin modes
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(12, INPUT);
  digitalWrite(4, HIGH); // set enable pin to high to enable motor driver
}

void loop()
{
  digitalWrite(7, HIGH); // set direction forward
  analogWrite(9, dutyCycle * 255 / 100); // run motor
  delay(2000);

  analogWrite(9, 0); // stop motor
  delay(2000);
  
  digitalWrite(7, LOW); // set direction to reverse
  analogWrite(9, dutyCycle * 255 / 100); // run motor
  delay(2000);
  
  analogWrite(9, 0); // stop motor
  delay(2000);
 
}
