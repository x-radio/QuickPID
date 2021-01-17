/********************************************************
   PID Basic Example
   Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <QuickPID.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3

//Define Variables we'll be connecting to
int Setpoint, Input, Output;

//Specify the links and initial tuning parameters
float Kp = 2, Ki = 5, Kd = 1;
float POn = 1.0; // Range is 0.0 to 1.0 (1.0 is 100% P on Error, 0% P on Measurement)

QuickPID myQuickPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, POn, DIRECT);

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
  myQuickPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
}
