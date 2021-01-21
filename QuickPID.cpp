/**********************************************************************************
   QuickPID Library for Arduino - Version 2.0.5
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
  SampleTimeUs = 100000;              // default is 0.1 seconds

  QuickPID::SetControllerDirection(ControllerDirection);
  QuickPID::SetTunings(Kp, Ki, Kd, POn);

  lastTime = micros() - SampleTimeUs;
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
  if (timeChange >= SampleTimeUs)
  {
    /*Compute all the working error variables*/
    int16_t input = *myInput;
    int16_t dInput = input - lastInput;
    error = *mySetpoint - input;

    /*Working error, Proportional distribution and Remaining PID output*/
    if (kpi < 31 && kpd < 31) outputSum += FX_MUL(FL_FX(kpi) , error) - FX_MUL(FL_FX(kpd), dInput);
    else outputSum += (kpi * error) - (kpd * dInput);

    if (outputSum > outMax) outputSum = outMax;
    if (outputSum < outMin) outputSum = outMin;
    *myOutput = outputSum;

    /*Remember some variables for next time*/
    lastInput = input;
    lastTime = now;
    return true;
  }
  else return false;
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

  float SampleTimeSec = (float)SampleTimeUs / 1000000;
  kp = Kp;
  ki = Ki * SampleTimeSec;
  kd = Kd / SampleTimeSec;
  kpi = kp * pOn + ki;
  kpd = kp * (1 - pOn) + kd;

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
  if (NewSampleTimeUs > 0)
  {
    float ratio  = (float)NewSampleTimeUs / (float)SampleTimeUs;
    ki *= ratio;
    kd /= ratio;

    SampleTimeUs = NewSampleTimeUs;
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
    if (*myOutput > outMax) *myOutput = outMax;
    else if (*myOutput < outMin) *myOutput = outMin;

    if (outputSum > outMax) outputSum = outMax;
    else if (outputSum < outMin) outputSum = outMin;
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
  if (outputSum > outMax) outputSum = outMax;
  else if (outputSum < outMin) outputSum = outMin;
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
bool QuickPID::GetMode() {
  return  inAuto ? AUTOMATIC : MANUAL;
}
bool QuickPID::GetDirection() {
  return controllerDirection;
}
int16_t QuickPID::GetError() {
  return  error;
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
