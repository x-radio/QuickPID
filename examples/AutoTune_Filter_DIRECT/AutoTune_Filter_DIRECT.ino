/******************************************************************************
   AutoTune Filter DIRECT Example
   Reference: https://github.com/Dlloydev/QuickPID/wiki/AutoTune
   Circuit: https://github.com/Dlloydev/QuickPID/wiki/AutoTune_RC_Filter
 ******************************************************************************/

#include "QuickPID.h"

const byte inputPin = 0;
const byte outputPin = 3;

const int outputMax = 255;
const int outputMin = 0;

float POn = 1.0;          // mix of PonE to PonM (0.0-1.0)

bool printOrPlotter = 0;  // on(1) monitor, off(0) plotter
byte tuningRule = 1;      // see reference link
byte outputStep = 1;
byte hysteresis = 1;
int setpoint = 341;       // 1/3 of 10-bit ADC range for symetrical waveform
int output = 85;          // 1/3 of 8-bit PWM range for symetrical waveform

float Input, Output, Setpoint;
float Kp = 0, Ki = 0, Kd = 0;
bool pidLoop = false;

QuickPID _myPID = QuickPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, POn, QuickPID::DIRECT);

void setup() {
  Serial.begin(115200);
  Serial.println();
  if (constrain(output, outputMin, outputMax - outputStep - 5) < output) {
    Serial.println(F("AutoTune test exceeds outMax limit. Check output, hysteresis and outputStep values"));
    while (1);
  }
  _myPID.AutoTune(tuningMethod::ZIEGLER_NICHOLS_PID);
  _myPID.autoTune->autoTuneConfig(outputStep, hysteresis, setpoint, output, QuickPID::DIRECT, printOrPlotter);
}

void loop() {

  if (_myPID.autoTune) // Avoid dereferencing nullptr after _myPID.clearAutoTune()
  {
    switch (_myPID.autoTune->autoTuneLoop()) {
      case _myPID.autoTune->AUTOTUNE:
        Input = avg(_myPID.analogReadFast(inputPin));
        analogWrite(outputPin, Output);
        break;

      case _myPID.autoTune->TUNINGS:
        _myPID.autoTune->setAutoTuneConstants(&Kp, &Ki, &Kd); // set new tunings
        _myPID.SetMode(QuickPID::AUTOMATIC); // setup PID
        _myPID.SetSampleTimeUs(10000); // 10ms
        _myPID.SetTunings(Kp, Ki, Kd, POn); // apply new tunings to PID
        Setpoint = 500;
        break;

      case _myPID.autoTune->CLR:
        if (!pidLoop) {
          _myPID.clearAutoTune(); // releases memory used by AutoTune object
          pidLoop = true;
        }
        break;
    }
  }
  if (pidLoop) {
    if (printOrPlotter == 0) { // plotter
      Serial.print("Setpoint:");  Serial.print(Setpoint);  Serial.print(",");
      Serial.print("Input:");     Serial.print(Input);     Serial.print(",");
      Serial.print("Output:");    Serial.print(Output);    Serial.println();
    }
    Input = _myPID.analogReadFast(inputPin);
    _myPID.Compute();
    analogWrite(outputPin, Output);
  }
  delay(1); // adjust loop speed
}

float avg(int inputVal) {
  static int arrDat[16];
  static int pos;
  static long sum;
  pos++;
  if (pos >= 16) pos = 0;
  sum = sum - arrDat[pos] + inputVal;
  arrDat[pos] = inputVal;
  return (float)sum / 16.0;
}
