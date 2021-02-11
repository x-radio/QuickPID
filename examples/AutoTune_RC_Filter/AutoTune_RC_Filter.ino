/******************************************************************************
   AutoTune RC Filter Example
   Use Serial Monitor and Serial Plotter to view results.
   Reference: https://github.com/Dlloydev/QuickPID/wiki/AutoTune
   Circuit: https://github.com/Dlloydev/QuickPID/wiki/AutoTune_RC_Filter

   TUNING RULE             RECOMMENED FOR
   0 ZIEGLER_NICHOLS_PI    Good noise and disturbance rejection
   1 ZIEGLER_NICHOLS_PID   Good noise and disturbance rejection
   2 TYREUS_LUYBEN_PI      Time-constant (lag) dominant processes (conservative)
   3 TYREUS_LUYBEN_PID     Time-constant (lag) dominant processes (conservative)
   4 CIANCONE_MARLIN_PI    Delay (dead-time) dominant processes
   5 CIANCONE_MARLIN_PID   Delay (dead-time) dominant processes
   6 AMIGOF_PI             TURNS CONTROLLER OFF (reserved for future version)
   7 PESSEN_INTEGRAL_PID   Similar to ZN_PID but with better dynamic response
   8 SOME_OVERSHOOT_PID    ZN_PID with lower proportional and integral gain
   9 NO_OVERSHOOT_PID      ZN_PID with much lower P,I,D gain settings
 ******************************************************************************/

#include "QuickPID.h"

const byte inputPin = 0;
const byte outputPin = 3;

// Specify the initial tuning parameters
int Print = 1;               // on(1), off(0)
int tuningRule = 2;          // see above table
float POn = 1.0;             // Mix of PonE to PonM (0.0-1.0)
unsigned long timeout = 120; // AutoTune timeout (sec)

int Input, Output, Setpoint;
float Kp = 0, Ki = 0, Kd = 0;

QuickPID myQuickPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, POn, DIRECT);

void setup()
{
  Serial.begin(115200);
  myQuickPID.SetSampleTimeUs(5000); // 5ms (ideally 10x faster than shortest RC time constant under test)
  myQuickPID.AutoTune(inputPin, outputPin, tuningRule, Print, timeout);
  myQuickPID.SetMode(AUTOMATIC);
  Setpoint = 700;
}

void loop()
{
  Serial.print("Setpoint:");  Serial.print(Setpoint);  Serial.print(",");
  Serial.print("Input:");     Serial.print(Input);     Serial.print(",");
  Serial.print("Output:");    Serial.print(Output);    Serial.print(",");
  Serial.println(",");

  Input = myQuickPID.analogReadFast(inputPin);
  myQuickPID.Compute();
  analogWrite(outputPin, Output);
}
