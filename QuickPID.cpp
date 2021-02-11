/**********************************************************************************
   QuickPID Library for Arduino - Version 2.2.0
   by dlloydev https://github.com/Dlloydev/QuickPID
   Based on the Arduino PID Library by Brett Beauregard

   This Library is licensed under the MIT License
 **********************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "QuickPID.h"

/* Constructor ********************************************************************
   The parameters specified here are those for for which we can't set up
   reliable defaults, so we need to have the user set them.
 **********************************************************************************/
QuickPID::QuickPID(int16_t* Input, int16_t* Output, int16_t* Setpoint,
                   float Kp, float Ki, float Kd, float POn = 1, bool ControllerDirection = 0)
{
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  inAuto = false;

  QuickPID::SetOutputLimits(0, 255);  // default is same as the arduino PWM limit
  sampleTimeUs = 100000;              // default is 0.1 seconds

  QuickPID::SetControllerDirection(ControllerDirection);
  QuickPID::SetTunings(Kp, Ki, Kd, POn);

  lastTime = micros() - sampleTimeUs;
}

/* Constructor ********************************************************************
   To allow backwards compatability for v1.1, or for people that just want
   to use Proportional on Error without explicitly saying so.
 **********************************************************************************/

QuickPID::QuickPID(int16_t* Input, int16_t* Output, int16_t* Setpoint,
                   float Kp, float Ki, float Kd, bool ControllerDirection)
  : QuickPID::QuickPID(Input, Output, Setpoint, Kp, Ki, Kd, pOn = 1, ControllerDirection = 0)
{

}

/* Compute() **********************************************************************
   This, as they say, is where the magic happens. This function should be called
   every time "void loop()" executes. The function will decide whether a new
   PID Output needs to be computed. Returns true when the output is computed,
   false when nothing has been done.
 **********************************************************************************/
bool QuickPID::Compute()
{
  if (!inAuto) return false;
  uint32_t now = micros();
  uint32_t timeChange = (now - lastTime);

  if (timeChange >= sampleTimeUs) {  // Compute the working error variables
    int16_t input = *myInput;
    int16_t dInput = input - lastInput;
    error = *mySetpoint - input;

    if (kpi < 31 && kpd < 31) outputSum += FX_MUL(FL_FX(kpi) , error) - FX_MUL(FL_FX(kpd), dInput); // fixed-point
    else outputSum += (kpi * error) - (kpd * dInput); // floating-point

    outputSum = QuickPID::Saturate(outputSum);
    *myOutput = outputSum;

    lastInput = input;
    lastTime = now;
    return true;
  }
  else return false;
}

/* AutoTune(...)***************************************************************************
   This function uses the Relay Method to tune the loop without operator intervention.
   It determines the ultimate gain (Ku) and ultimate period (Tu) which is used in the
   Ziegler-Nichols tuning rules to compute Kp, Ki and Kd.
 ******************************************************************************************/
void QuickPID::AutoTune(int inputPin, int outputPin, int tuningRule, int Print = 0, uint32_t timeout = 120) {

  uint32_t  stepPrevTime, stepTime;
  float Ku, Tu;

  const int Rule[10][3] =
  { //ckp,  cki, ckd x 1000
    { 450,  540,   0 },  // ZIEGLER_NICHOLS_PI
    { 600, 1176,  75 },  // ZIEGLER_NICHOLS_PID
    { 313,  142,   0 },  // TYREUS_LUYBEN_PI
    { 454,  206,  72 },  // TYREUS_LUYBEN_PID
    { 303, 1212,   0 },  // CIANCONE_MARLIN_PI
    { 303, 1333,  37 },  // CIANCONE_MARLIN_PID
    {   0,    0,   0 },  // AMIGOF_PI (reserved)
    { 700, 1750, 105 },  // PESSEN_INTEGRAL_PID
    { 333,  667, 111 },  // SOME_OVERSHOOT_PID
    { 200,  400,  67 },  // NO_OVERSHOOT_PID
  };
  const byte ckp = 0, cki = 1, ckd = 2; // c = column
  peakHigh = atSetpoint;
  peakLow = atSetpoint;
  timeout *= 1000;
  if (Print == 1) Serial.println();
  if (Print == 1) Serial.print("Stabilizing (33%) →");
  QuickPID::Stabilize(inputPin, outputPin, timeout);
  if (Print == 1) Serial.print("→ Running AutoTune");
  QuickPID::StepUp(inputPin, outputPin, timeout);
  stepPrevTime = micros();
  if (Print == 1) Serial.print(" ↑");
  QuickPID::StepDown(inputPin, outputPin, timeout);
  if (Print == 1) Serial.print(" ↓");
  QuickPID::StepUp(inputPin, outputPin, timeout);
  stepTime = micros();
  if (Print == 1) Serial.print(" ↑");
  QuickPID::StepDown(inputPin, outputPin, timeout);
  if (Print == 1) Serial.print(" ↓");
  QuickPID::StepUp(inputPin, outputPin, timeout);
  if (Print == 1) Serial.print(" ↑");
  stepTime = micros();
  Tu = (float)(stepTime - stepPrevTime) / 2000000.0; // ultimate period based on 2 cycles
  dispTu = Tu;
  Ku = (float)(4 * outputStep * 2) / (float)(3.14159 * sqrt (sq (peakHigh - peakLow) - sq (hysteresis))); // ultimate gain
  dispKu = Ku;

  if (Print == 1) {
    Serial.println(); Serial.print("Ultimate Period Tu: "); Serial.print(Tu, 3); Serial.println("s");
    Serial.print("Ultimate Gain Ku: "); Serial.println(Ku, 3);

    Serial.print("peakHigh: "); Serial.println(peakHigh);
    Serial.print("peakLow: "); Serial.println(peakLow);
  }

  kp = Rule[tuningRule][ckp] / 1000.0 * Ku;
  ki = Rule[tuningRule][cki] / 1000.0 * Ku / Tu;
  kd = Rule[tuningRule][ckd] / 1000.0 * Ku * Tu;

  if (Print == 1) {
    Serial.print("Kp: "); Serial.print(kp, 3);
    Serial.print("  Ki: "); Serial.print(ki, 3);
    Serial.print("  Kd: "); Serial.println(kd, 3);
  }
  SetTunings(kp, ki, kd);
}

/* SetTunings(....)************************************************************
  This function allows the controller's dynamic performance to be adjusted.
  it's called automatically from the constructor, but tunings can also
  be adjusted on the fly during normal operation
******************************************************************************/
void QuickPID::SetTunings(float Kp, float Ki, float Kd, float POn = 1)
{
  if (Kp < 0 || Ki < 0 || Kd < 0) return;

  pOn = POn;
  dispKp = Kp; dispKi = Ki; dispKd = Kd;

  float SampleTimeSec = (float)sampleTimeUs / 1000000;
  kp = Kp;
  ki = Ki * SampleTimeSec;
  kd = Kd / SampleTimeSec;
  kpi = (kp * pOn) + ki;
  kpd = (kp * (1 - pOn)) + kd;

  if (controllerDirection == REVERSE)
  {
    kp = (0 - kp);
    ki = (0 - ki);
    kd = (0 - kd);
  }
}

/* SetTunings(...)*************************************************************
  Set Tunings using the last remembered POn setting.
******************************************************************************/
void QuickPID::SetTunings(float Kp, float Ki, float Kd) {
  SetTunings(Kp, Ki, Kd, pOn);
}

/* SetSampleTime(...) *********************************************************
  Sets the period, in microseconds, at which the calculation is performed
******************************************************************************/
void QuickPID::SetSampleTimeUs(uint32_t NewSampleTimeUs)
{
  if (NewSampleTimeUs > 0) {
    float ratio  = (float)NewSampleTimeUs / (float)sampleTimeUs;
    ki *= ratio;
    kd /= ratio;
    sampleTimeUs = NewSampleTimeUs;
  }
}

/* SetOutputLimits(...)********************************************************
  The PID controller is designed to vary its output within a given range.
  By default this range is 0-255, the Arduino PWM range.
******************************************************************************/
void QuickPID::SetOutputLimits(int16_t Min, int16_t Max)
{
  if (Min >= Max) return;
  outMin = Min;
  outMax = Max;

  if (inAuto)
  {
    *myOutput = QuickPID::Saturate(*myOutput);
    outputSum = QuickPID::Saturate(outputSum);
  }
}

/* SetMode(...)****************************************************************
  Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
  when the transition from manual to auto occurs, the controller is
  automatically initialized
******************************************************************************/
void QuickPID::SetMode(bool Mode)
{
  bool newAuto = (Mode == AUTOMATIC);
  if (newAuto && !inAuto)
  { /*we just went from manual to auto*/
    QuickPID::Initialize();
  }
  inAuto = newAuto;
}

/* Initialize()****************************************************************
  Does all the things that need to happen to ensure a bumpless transfer
  from manual to automatic mode.
******************************************************************************/
void QuickPID::Initialize()
{
  outputSum = *myOutput;
  lastInput = *myInput;
  outputSum = QuickPID::Saturate(outputSum);
}

/* SetControllerDirection(...)*************************************************
  The PID will either be connected to a DIRECT acting process (+Output leads
  to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
  know which one, because otherwise we may increase the output when we should
  be decreasing.  This is called from the constructor.
******************************************************************************/
void QuickPID::SetControllerDirection(bool Direction)
{
  if (inAuto && Direction != controllerDirection)
  {
    kp = (0 - kp);
    ki = (0 - ki);
    kd = (0 - kd);
  }
  controllerDirection = Direction;
}

/* Status Functions************************************************************
  Just because you set the Kp=-1 doesn't mean it actually happened. These
  functions query the internal state of the PID. They're here for display
  purposes. These are the functions the PID Front-end uses for example.
******************************************************************************/
float QuickPID::GetKp() {
  return  dispKp;
}
float QuickPID::GetKi() {
  return  dispKi;
}
float QuickPID::GetKd() {
  return  dispKd;
}
float QuickPID::GetKu() {
  return  dispKu;
}
float QuickPID::GetTu() {
  return  dispTu;
}
bool QuickPID::GetMode() {
  return  inAuto ? AUTOMATIC : MANUAL;
}
bool QuickPID::GetDirection() {
  return controllerDirection;
}

// Utility functions **********************************************************

int QuickPID::analogReadFast(int ADCpin) {
#if defined(__AVR_ATmega328P__)
  byte ADCregOriginal = ADCSRA;
  ADCSRA = (ADCSRA & B11111000) | 5; //32 prescaler
  int adc = analogRead(ADCpin);
  ADCSRA = ADCregOriginal;
  return adc;
#elif defined(__AVR_ATtiny_Zero_One__) || defined(__AVR_ATmega_Zero__)
  byte ADCregOriginal = ADC0_CTRLC;
  ADC0_CTRLC = 0x54; //reduced cap, Vdd ref, 32 prescaler
  int adc = analogRead(ADCpin);
  ADC0_CTRLC = ADCregOriginal;
  return adc;
#elif defined(__AVR_DA__)
  byte ADCregOriginal = ADC0.CTRLC;
  ADC0.CTRLC = ADC_PRESC_DIV32_gc; //32 prescaler
  int adc = analogRead(ADCpin);
  ADC0.CTRLC = ADCregOriginal;
  return adc;
#else
  return analogRead(ADCpin);
# endif
}

float QuickPID::analogReadAvg(int ADCpin) {
  static int arrDat[16];
  static int dat;
  static int pos;
  static long sum;
  dat = QuickPID::analogReadFast(ADCpin);
  pos++;
  if (pos >= 16) pos = 0;
  sum = sum - arrDat[pos] + dat;
  arrDat[pos] = dat;
  return (float)sum / 16.0;
}

int16_t QuickPID::Saturate(int16_t Out) {
  if (Out > outMax) Out = outMax;
  else if (Out < outMin) Out = outMin;
  return Out;
}

void QuickPID::StepUp(int inputPin, int outputPin, uint32_t timeout) {
  analogWrite (outputPin, (atOutput + outputStep)); // step up output, wait for input to reach +'ve hysteresis
  while ((analogReadAvg(inputPin) < (atSetpoint + hysteresis)) && (millis() <= timeout)) {
    float rdAvg = analogReadAvg(inputPin);
    if (rdAvg > peakHigh) peakHigh = rdAvg;
    if ((rdAvg < peakLow) && (peakHigh >= (atSetpoint + hysteresis))) peakLow = rdAvg;
    delayMicroseconds(readPeriod);
  }
}

void QuickPID::StepDown(int inputPin, int outputPin, uint32_t timeout) {
  analogWrite (outputPin, (atOutput - outputStep)); // step down output, wait for input to reach -'ve hysteresis
  while ((analogReadAvg(inputPin) > (atSetpoint - hysteresis)) && (millis() <= timeout)) {
    float rdAvg = analogReadAvg(inputPin);
    if (rdAvg > peakHigh) peakHigh = rdAvg;
    if ((rdAvg < peakLow) && (peakHigh >= (atSetpoint + hysteresis))) peakLow = rdAvg;
    delayMicroseconds(readPeriod);
  }
}

void QuickPID::Stabilize(int inputPin, int outputPin, uint32_t timeout) {
  // initial reading
  for (int i = 0; i <= 16; i++) {
    analogReadAvg(inputPin);
  }
  // coarse adjust
  analogWrite (outputPin, 0);
  while ((analogReadAvg(inputPin) > atSetpoint) && (millis() <= timeout));
  analogWrite(outputPin, atOutput * 2);
  while ((analogReadAvg(inputPin) < atSetpoint) && (millis() <= timeout));

  // fine adjust
  analogWrite (outputPin, atOutput - outputStep - 1);
  while ((analogReadAvg(inputPin) > atSetpoint) && (millis() <= timeout));
  analogWrite(outputPin, atOutput + outputStep + 1);
  while ((analogReadAvg(inputPin) < atSetpoint) && (millis() <= timeout));
  analogWrite(outputPin, atOutput);
}
