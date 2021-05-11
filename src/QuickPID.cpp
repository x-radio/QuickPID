/**********************************************************************************
   QuickPID Library for Arduino - Version 2.2.8
   by dlloydev https://github.com/Dlloydev/QuickPID
   Based on the Arduino PID Library, licensed under the MIT License
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
QuickPID::QuickPID(float* Input, float* Output, float* Setpoint,
                   float Kp, float Ki, float Kd, float POn = 1,
                   QuickPID::direction_t ControllerDirection = DIRECT) {

  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  inAuto = false;

  QuickPID::SetOutputLimits(0, 255);  // same default as Arduino PWM limit
  sampleTimeUs = 100000;              // 0.1 sec default
  QuickPID::SetControllerDirection(ControllerDirection);
  QuickPID::SetTunings(Kp, Ki, Kd, POn);

  lastTime = micros() - sampleTimeUs;
}

/* Constructor ********************************************************************
   To allow backwards compatability for v1.1, or for people that just want
   to use Proportional on Error without explicitly saying so.
 **********************************************************************************/
QuickPID::QuickPID(float* Input, float* Output, float* Setpoint,
                   float Kp, float Ki, float Kd, direction_t ControllerDirection = DIRECT)
  : QuickPID::QuickPID(Input, Output, Setpoint, Kp, Ki, Kd, pOn = 1, ControllerDirection = DIRECT) {
}

/* Compute() **********************************************************************
   This function should be called every time "void loop()" executes. The function
   will decide whether a new PID Output needs to be computed. Returns true
   when the output is computed, false when nothing has been done.
 **********************************************************************************/
bool QuickPID::Compute() {
  if (!inAuto) return false;
  uint32_t now = micros();
  uint32_t timeChange = (now - lastTime);

  if (timeChange >= sampleTimeUs) {  // Compute the working error variables
    int input = *myInput;
    int dInput = input - lastInput;
    error = *mySetpoint - input;
    if (controllerDirection == REVERSE) {
      error = -error;
      dInput = -dInput;
    }
      if (kpi < 31 && kpd < 31) outputSum += FX_MUL(FL_FX(kpi) , error) - FX_MUL(FL_FX(kpd), dInput); // fixed point
      else outputSum += (kpi * error) - (kpd * dInput); // floating-point

    outputSum = CONSTRAIN(outputSum, outMin, outMax);
    *myOutput = outputSum;

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
void QuickPID::SetTunings(float Kp, float Ki, float Kd, float POn = 1) {
  if (Kp < 0 || Ki < 0 || Kd < 0) return;
  pOn = POn;
  dispKp = Kp; dispKi = Ki; dispKd = Kd;

  float SampleTimeSec = (float)sampleTimeUs / 1000000;
  kp = Kp;
  ki = Ki * SampleTimeSec;
  kd = Kd / SampleTimeSec;
  kpi = (kp * pOn) + ki;
  kpd = (kp * (1 - pOn)) + kd;
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
void QuickPID::SetSampleTimeUs(uint32_t NewSampleTimeUs) {
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
void QuickPID::SetOutputLimits(int Min, int Max) {
  if (Min >= Max) return;
  outMin = Min;
  outMax = Max;

  if (inAuto) {
    *myOutput = CONSTRAIN(*myOutput, outMin, outMax);
    outputSum = CONSTRAIN(outputSum, outMin, outMax);
  }
}

/* SetMode(...)****************************************************************
  Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
  when the transition from manual to auto occurs, the controller is
  automatically initialized
******************************************************************************/
void QuickPID::SetMode(mode_t Mode) {
  bool newAuto = (Mode == AUTOMATIC);
  if (newAuto && !inAuto) { //we just went from manual to auto
    QuickPID::Initialize();
  }
  inAuto = newAuto;
}

/* Initialize()****************************************************************
  Does all the things that need to happen to ensure a bumpless transfer
  from manual to automatic mode.
******************************************************************************/
void QuickPID::Initialize() {
  outputSum = *myOutput;
  lastInput = *myInput;
  outputSum = CONSTRAIN(outputSum, outMin, outMax);
}

/* SetControllerDirection(...)*************************************************
  The PID will either be connected to a DIRECT acting process (+Output leads
  to +Input) or a REVERSE acting process(+Output leads to -Input.)
******************************************************************************/
void QuickPID::SetControllerDirection(direction_t ControllerDirection) {
  controllerDirection = ControllerDirection;
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

QuickPID::mode_t QuickPID::GetMode() {
  return  inAuto ? QuickPID::AUTOMATIC : QuickPID::MANUAL;
}

QuickPID::direction_t QuickPID::GetDirection() {
  return controllerDirection;
}

/* Other Functions************************************************************/

int QuickPID::analogReadFast(int ADCpin) {
#if defined(__AVR_ATmega328P__)
  byte ADCregOriginal = ADCSRA;
  ADCSRA = (ADCSRA & B11111000) | 5; // 32 prescaler
  int adc = analogRead(ADCpin);
  ADCSRA = ADCregOriginal;
  return adc;
#elif defined(__AVR_ATtiny_Zero_One__) || defined(__AVR_ATmega_Zero__)
  byte ADCregOriginal = ADC0_CTRLC;
  ADC0_CTRLC = 0x54; // reduced cap, Vdd ref, 32 prescaler
  int adc = analogRead(ADCpin);
  ADC0_CTRLC = ADCregOriginal;
  return adc;
#elif defined(__AVR_DA__)
  byte ADCregOriginal = ADC0.CTRLC;
  ADC0.CTRLC = ADC_PRESC_DIV32_gc; // 32 prescaler
  int adc = analogRead(ADCpin);
  ADC0.CTRLC = ADCregOriginal;
  return adc;
#else
  return analogRead(ADCpin);
# endif
}
