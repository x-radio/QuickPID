/**********************************************************************************
   QuickPID Library for Arduino - Version 3.0.0
   by dlloydev https://github.com/Dlloydev/QuickPID
   Based on the Arduino PID_v1 Library. Licensed under the MIT License.
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
                   float Kp, float Ki, float Kd, uint8_t pMode = PE, uint8_t dMode = DM,
                    uint8_t awMode = CLAMP, uint8_t Action = DIRECT) {

  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  mode = MANUAL;

  QuickPID::SetOutputLimits(0, 255);  // same default as Arduino PWM limit
  sampleTimeUs = 100000;              // 0.1 sec default
  QuickPID::SetControllerDirection(Action);
  QuickPID::SetTunings(Kp, Ki, Kd, pMode, dMode, awMode);

  lastTime = micros() - sampleTimeUs;
}

/* Constructor *********************************************************************
   To allow using Proportional on Error without explicitly saying so.
 **********************************************************************************/
QuickPID::QuickPID(float* Input, float* Output, float* Setpoint,
                   float Kp, float Ki, float Kd, uint8_t Action = DIRECT)
  : QuickPID::QuickPID(Input, Output, Setpoint, Kp, Ki, Kd, pmode = PE, dmode = DM, awmode = CLAMP, Action = DIRECT) {
}

/* Compute() ***********************************************************************
   This function should be called every time "void loop()" executes. The function
   will decide whether a new PID Output needs to be computed. Returns true
   when the output is computed, false when nothing has been done.
 **********************************************************************************/
bool QuickPID::Compute() {
  if (mode == MANUAL) return false;
  uint32_t now = micros();
  uint32_t timeChange = (now - lastTime);
  if (mode == TIMER || timeChange >= sampleTimeUs) {

    float input = *myInput;
    float dInput = input - lastInput;
    if (action == REVERSE) dInput = -dInput;

    error = *mySetpoint - input;
    if (action == REVERSE) error = -error;
    float dError = error - lastError;

    float peTerm = kp * error;
    float pmTerm = kp * dInput;
    if (pmode == PE) pmTerm = 0;
    else if (pmode == PM) peTerm = 0;
    else { //PEM
      peTerm *= 0.5;
      pmTerm *= 0.5;
    }
    pTerm = peTerm - pmTerm; // used by GetDterm()
    iTerm =  ki  * error;
    if (dmode == DE) dTerm = kd * dError;
    else dTerm = -kd * dInput; // DM

    //condition anti-windup
    if (awmode == CONDITION) {
      bool aw = false;
      float iTermOut = (peTerm - pmTerm) + ki * (iTerm + error);
      if (iTermOut > outMax && dError > 0) aw = true;
      else if (iTermOut < outMin && dError < 0) aw = true;
      if (aw && ki) iTerm = constrain(iTermOut, -outMax, outMax);
    }

    // by default, compute output as per PID_v1
    outputSum += iTerm;                                                 // include integral amount
    if (awmode == OFF) outputSum -= pmTerm;                             // include pmTerm (no anti-windup)
    else outputSum = constrain(outputSum - pmTerm, outMin, outMax);     // include pmTerm and clamp (default)
    *myOutput = constrain(outputSum + peTerm + dTerm, outMin, outMax);  // include dTerm, clamp and drive output

    lastError = error;
    lastInput = input;
    lastTime = now;
    return true;
  }
  else return false;
}

/* SetTunings(....)************************************************************
  This function allows the controller's dynamic performance to be adjusted.
  it's called automatically from the constructor, but tunings can also
  be adjusted on the fly during normal operation.
******************************************************************************/
void QuickPID::SetTunings(float Kp, float Ki, float Kd, uint8_t pMode = PE, uint8_t dMode = DM, uint8_t awMode = CLAMP) {
  if (Kp < 0 || Ki < 0 || Kd < 0) return;
  pmode = pMode; dmode = dMode; awmode = awMode;
  dispKp = Kp; dispKi = Ki; dispKd = Kd;
  float SampleTimeSec = (float)sampleTimeUs / 1000000;
  kp = Kp;
  ki = Ki * SampleTimeSec;
  kd = Kd / SampleTimeSec;
}

/* SetTunings(...)************************************************************
  Set Tunings using the last remembered pMode and dMode setting.
******************************************************************************/
void QuickPID::SetTunings(float Kp, float Ki, float Kd) {
  SetTunings(Kp, Ki, Kd, pmode, dmode, awmode);
}

/* SetSampleTime(.)***********************************************************
  Sets the period, in microseconds, at which the calculation is performed.
******************************************************************************/
void QuickPID::SetSampleTimeUs(uint32_t NewSampleTimeUs) {
  if (NewSampleTimeUs > 0) {
    float ratio  = (float)NewSampleTimeUs / (float)sampleTimeUs;
    ki *= ratio;
    kd /= ratio;
    sampleTimeUs = NewSampleTimeUs;
  }
}

/* SetOutputLimits(..)********************************************************
  The PID controller is designed to vary its output within a given range.
  By default this range is 0-255, the Arduino PWM range.
******************************************************************************/
void QuickPID::SetOutputLimits(float Min, float Max) {
  if (Min >= Max) return;
  outMin = Min;
  outMax = Max;

  if (mode != MANUAL) {
    *myOutput = constrain(*myOutput, outMin, outMax);
    outputSum = constrain(outputSum, outMin, outMax);
  }
}

/* SetMode(.)*****************************************************************
  Sets the controller mode to MANUAL (0), AUTOMATIC (1) or TIMER (2)
  when the transition from MANUAL to AUTOMATIC or TIMER occurs, the
  controller is automatically initialized.
******************************************************************************/
void QuickPID::SetMode(uint8_t Mode) {
  if (mode == MANUAL && Mode != MANUAL) { // just went from MANUAL to AUTOMATIC or TIMER
    QuickPID::Initialize();
  }
  mode = Mode;
}

/* Initialize()****************************************************************
  Does all the things that need to happen to ensure a bumpless transfer
  from manual to automatic mode.
******************************************************************************/
void QuickPID::Initialize() {
  outputSum = *myOutput;
  lastInput = *myInput;
  outputSum = constrain(outputSum, outMin, outMax);
}

/* SetControllerDirection(.)**************************************************
  The PID will either be connected to a DIRECT acting process (+Output leads
  to +Input) or a REVERSE acting process(+Output leads to -Input).
******************************************************************************/
void QuickPID::SetControllerDirection(uint8_t Action) {
  action = Action;
}

/* Status Functions************************************************************
  These functions query the internal state of the PID.
******************************************************************************/
float QuickPID::GetKp() {
  return dispKp;
}
float QuickPID::GetKi() {
  return dispKi;
}
float QuickPID::GetKd() {
  return dispKd;
}
float QuickPID::GetPterm() {
  return pTerm;
}
float QuickPID::GetIterm() {
  return iTerm;
}
float QuickPID::GetDterm() {
  return dTerm;
}
uint8_t QuickPID::GetMode() {
  return  mode;
}
uint8_t QuickPID::GetDirection() {
  return action;
}
uint8_t QuickPID::GetPmode() {
  return pmode;
}
uint8_t QuickPID::GetDmode() {
  return dmode;
}
uint8_t QuickPID::GetAwMode() {
  return awmode;
}
