/********************************************************
   PID Adaptive Tuning Example
   One of the benefits of the PID library is that you can
   change the tuning parameters at any time.  this can be
   helpful if we want the controller to be agressive at some
   times, and conservative at others.   in the example below
   we set the controller to use Conservative Tuning Parameters
   when we're near setpoint and more agressive Tuning
   Parameters when we're farther away.
 ********************************************************/

#include <QuickPID.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3

//Define Variables we'll be connecting to
int Setpoint, Input, Output;

//Define the aggressive and conservative and POn Tuning Parameters
float aggKp = 4, aggKi = 0.2, aggKd = 1;
float consKp = 1, consKi = 0.05, consKd = 0.25;
float aggPOn = 1.0; // Range is 0.0 to 1.0 (1.0 is 100% P on Error, 0% P on Measurement)
float consPOn = 0.0; // Range is 0.0 to 1.0 (0.0 is 0% P on Error, 100% P on Measurement)

//Specify the links and initial tuning parameters
QuickPID myQuickPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, aggPOn, DIRECT);

void setup()
{
  //initialize the variables we're linked to
  Input = myQuickPID.analogReadFast(PIN_INPUT);
  Setpoint = 100;

  //turn the PID on
  myQuickPID.SetMode(AUTOMATIC);
}

void loop()
{
  Input = myQuickPID.analogReadFast(PIN_INPUT);

  float gap = abs(Setpoint - Input); //distance away from setpoint
  if (gap < 10) { //we're close to setpoint, use conservative tuning parameters
    myQuickPID.SetTunings(consKp, consKi, consKd, consPOn);
  } else {
    //we're far from setpoint, use aggressive tuning parameters
    myQuickPID.SetTunings(aggKp, aggKi, aggKd, aggPOn);
  }
  myQuickPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
}
