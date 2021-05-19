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

QuickPID _myPID = QuickPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, POn, QuickPID::DIRECT);

void setup() {
  Serial.begin(115200);
  Serial.println();
  if (constrain(output, outputMin, outputMax - outputStep - 5) < output) {
    Serial.println(F("AutoTune test exceeds outMax limit. Check output, hysteresis and outputStep values"));
    while (1);
  }
  _myPID.AutoTune(tuningRule);
  _myPID.autoTune->autoTuneConfig(outputStep, hysteresis, setpoint, output, QuickPID::DIRECT, printOrPlotter);
}

void loop() {

if (_myPID.autoTune) // Avoid deferencing nullptr after _myPID.clearAutoTune()
{	
	uint8_t autoTuningCurrentStage = autoTuneLoop();
	if(autoTuningCurrentStage < _myPID.autoTune->RUN_PID)
	{
		Input = avg(_myPID.analogReadFast(inputPin)); // filtered input
		analogWrite(outputPin, Output);
		
		if (autoTuningCurrentStage == _myPID.autoTune->NEW_TUNINGS) // get new tunings
		{
			_myPID.autoTune->setAutoTuneConstants(&Kp, &Ki, &Kd); // set new tunings
			_myPID.SetMode(QuickPID::AUTOMATIC); // setup PID
			_myPID.SetTunings(Kp, Ki, Kd, POn); // apply new tunings to PID
			Setpoint = 500;
		}
	}
	else // RUN_PID stage
	{
		if (printOrPlotter == 0) // plotter
		{
			_myPID.clearAutoTune(); // releases memory used by AutoTune object
			Serial.print("Setpoint:");  Serial.print(Setpoint);  Serial.print(",");
			Serial.print("Input:");     Serial.print(Input);     Serial.print(",");
			Serial.print("Output:");    Serial.print(Output);    Serial.println();
       }
	}
	
}
else // Autotune already done (or not created)
{
	Input = _myPID.analogReadFast(inputPin);
    _myPID.Compute();
    analogWrite(outputPin, Output);
}
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
