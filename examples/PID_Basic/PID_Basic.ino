/********************************************************
   PID Basic Example
   Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include "QuickPID.h"

#define PIN_INPUT 0
#define PIN_OUTPUT 3

//Define Variables we'll be connecting to
float Setpoint, Input, Output;

//Specify the links and initial tuning parameters
float Kp = 2, Ki = 5, Kd = 1;

/* Instantiate PID with simplified constructor, remaining parameters use defaults:
   Parameter  Default                Description
           4  Kp = 0                 proportional gain
           5  Ki = 0                 integral gain
           6  Kd = 0                 derivative gain
           7  pMode::pOnError        proportional on error
           8  dMode::dOnMeas         derivative on measurement
           9  iAwMode::iAwCondition  advanced integral anti-windup
          10  Action::direct         direct acting controller
*/
QuickPID myPID(&Input, &Output, &Setpoint);

void setup()
{
  //initialize the variables we're linked to
  Input = analogRead(PIN_INPUT);
  Setpoint = 100;

  //apply PID gains
  myPID.SetTunings(Kp, Ki, Kd);

  //turn the PID on
  myPID.SetMode(myPID.Control::automatic);
}

void loop()
{
  Input = analogRead(PIN_INPUT);
  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
}
