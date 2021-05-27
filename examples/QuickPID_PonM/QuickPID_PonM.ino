/********************************************************
   QuickPID Proportional on measurement Example
   Setting the PID to use Proportional on measurement will
   make the output move more smoothly when the setpoint
   is changed.  In addition, it can eliminate overshoot
   in certain processes like sous-vides.
 ********************************************************/

#include "QuickPID.h"

//Define Variables we'll be connecting to
float Setpoint, Input, Output;
float POn = 0.0; // Range is 0.0 to 1.0 (0.0 is 0% P on Error, 100% P on Measurement)

//Specify the links and initial tuning parameters
QuickPID myQuickPID(&Input, &Output, &Setpoint, 2, 5, 1, POn, QuickPID::DIRECT);

void setup()
{
  //initialize the variables we're linked to
  Input = myQuickPID.analogReadFast(0);
  Setpoint = 100;

  //turn the PID on
  myQuickPID.SetMode(QuickPID::AUTOMATIC);
}

void loop()
{
  Input = myQuickPID.analogReadFast(0);
  myQuickPID.Compute();
  analogWrite(3, Output);
}
