/************************************************************
   PID RC Filter Self Test Example:
   One 47µF capacitor connected from GND to a 27K resistor
   terminated at pwm pin 3. Junction point of the RC filter
   is connected to A0. Use Serial Plotter to view results.
 ************************************************************/

#include "QuickPID.h"

#define PIN_INPUT  A0
#define PIN_OUTPUT  3

//Define Variables
int16_t Setpoint = 700;
int16_t Input;
uint8_t Output;

uint32_t before, after;
uint16_t cnt = 0;

//Specify the initial tuning parameters
float Kp = 2.0, Ki = 15.0, Kd = 0.05;

QuickPID myQuickPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  myQuickPID.SetTunings(Kp, Ki, Kd, P_ON_M);
  Serial.begin(115200);
  analogWrite(PIN_OUTPUT, 0);
  delay(1000); // discharge capacitor
  Input = myQuickPID.analogReadFast(PIN_INPUT);
  myQuickPID.SetMode(AUTOMATIC);
}

void loop()
{
  //Serial.println("Min:0,Max:1000"); // set scale

  // Stretch the plot x2
  for (int i = 0; i <= 1; i++) {
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
  }

  Input = myQuickPID.analogReadFast(PIN_INPUT);
  before = micros();
  myQuickPID.Compute();
  after = micros();
  analogWrite(PIN_OUTPUT, Output);

  delay(20);
  cnt++;
  if (cnt == 100) {
    analogWrite(PIN_OUTPUT, 0);
    delay(1000); // discharge capacitor
    cnt = 0;
  }
}
