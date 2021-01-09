/**********************************************************************************
   QuickPID Library for Arduino - Version 2.0.1
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
QuickPID::QuickPID(int16_t* Input, uint8_t* Output, int16_t* Setpoint,
                   float Kp, float Ki, float Kd, bool POn, bool ControllerDirection)
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

QuickPID::QuickPID(int16_t* Input, uint8_t* Output, int16_t* Setpoint,
                   float Kp, float Ki, float Kd, bool ControllerDirection)
  : QuickPID::QuickPID(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection)
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

    /*Working error, Proportional on Measurement and Remaining PID output*/
    if (!pOnE) {
      if (ki < 31) outputSum += FX_INT(FX_MUL(FL_FX(ki), INT_FX(error)));
      else outputSum += (ki * error);
      if (kpd < 31) outputSum -= FX_INT(FX_MUL(FL_FX(kpd), INT_FX(dInput)));
      else outputSum -= (kpd * dInput);
    }
    /*Working error, Proportional on Error and remaining PID output*/
    if (pOnE) {
      if (kpi < 31) outputSum += FX_INT(FX_MUL(FL_FX(kpi), INT_FX(error)));
      else  outputSum += (kpi * error);
      if (kd < 31) outputSum -= FX_INT(FX_MUL(FL_FX(kd), INT_FX(dInput)));
      else outputSum -= (kd * dInput);
    }
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
void QuickPID::SetTunings(float Kp, float Ki, float Kd, bool POn)
{
  if (Kp < 0 || Ki < 0 || Kd < 0) return;

  pOn = POn;
  pOnE = POn == P_ON_E;
  dispKp = Kp; dispKi = Ki; dispKd = Kd;

  float SampleTimeSec = (float)SampleTimeUs / 1000000;
  kp = Kp;
  ki = Ki * SampleTimeSec;
  kd = Kd / SampleTimeSec;
  kpd = kp + kd;
  kpi = kp + ki;

  if (controllerDirection == REVERSE)
  {
    kp = (0 - kp);
    ki = (0 - ki);
    kd = (0 - kd);
  }
}

/* SetTunings(...)*************************************************************
   Set Tunings using the last-rembered POn setting
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
   This function will be used far more often than SetInputLimits. While
   the input to the controller will generally be in the 0-1023 range, which is
   the default already, the required output limits might be unique, like using
   a time window of 0-8000 needing to clamp it from 0-125.
 ******************************************************************************/
void QuickPID::SetOutputLimits(uint8_t Min, uint8_t Max)
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
int16_t QuickPID::GetError() {
  return  error;
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
  ADCSRA = (ADCSRA & B11111000) | 6; //64 prescaler
  int adc = analogRead(ADCpin);
  ADCSRA = ADCregOriginal;
  return adc;
#elif defined(__AVR_ATtiny_Zero_One__) || defined(__AVR_ATmega_Zero__)
  byte ADCregOriginal = ADC0_CTRLC;
  ADC0_CTRLC = 0x55; //reduced cap, Vdd ref, 64 prescaler
  int adc = analogRead(ADCpin);
  ADC0_CTRLC = ADCregOriginal;
  return adc;
#elif defined(__AVR_DA__)
  byte ADCregOriginal = ADC0.CTRLC;
  ADC0.CTRLC = ADC_PRESC_DIV64_gc; //64 prescaler
  int adc = analogRead(ADCpin);
  ADC0.CTRLC = ADCregOriginal;
  return adc;
#else
  return analogRead(ADCpin);
# endif
}
