/**************************************************************
   PID AutoTune RC Filter Example:
   One 47µF capacitor connected from GND to a 10K-100K resistor
   terminated at pwm pin 3. Junction point of the RC filter
   is connected to A0. Use Serial Plotter to view results.
   https://github.com/Dlloydev/QuickPID/wiki/AutoTune_RC_Filter
 **************************************************************/

#include "QuickPID.h"

const byte inputPin = 0;
const byte outputPin = 3;

// Define Variables
int Input, Output, Setpoint;

unsigned long before, after, timeout = 30;
int loopCount = 0;
int Print = 0; // on(1), off(0)
int tuningRule = 0; // PID(0), PI(1)

// Specify the initial tuning parameters
float Kp = 0, Ki = 0, Kd = 0;
float POn = 0.5; // Mix of PonE to PonM (0.0-1.0)

QuickPID myQuickPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, POn, DIRECT);

void setup()
{
  Serial.begin(115200);
  myQuickPID.AutoTune(inputPin, outputPin, tuningRule, Print, timeout);
  myQuickPID.SetMode(AUTOMATIC);
  analogWrite(outputPin, 0); // discharge capacitor
  delay(1000);
  Setpoint = 700;
}

void loop()
{
  Serial.print("Setpoint:");
  Serial.print(Setpoint);
  Serial.print(",");
  Serial.print("Input:");
  Serial.print(Input);
  Serial.print(",");
  Serial.print("Output:");
  Serial.print(Output);
  Serial.print(",");
  Serial.print("Runtime:");
  Serial.print(after - before);
  Serial.println(",");

  Input = myQuickPID.analogReadFast(inputPin);
  before = micros();
  myQuickPID.Compute();
  after = micros();
  analogWrite(outputPin, Output);

  delay(20);      // sets loop timing
  loopCount++;
  if (loopCount >= 250) {
    analogWrite(outputPin, 0);
    delay(1000);  // discharge capacitor
    loopCount = 0;
  }
}
