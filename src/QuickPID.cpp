/**********************************************************************************
   QuickPID Library for Arduino - Version 2.3.0
   by dlloydev https://github.com/Dlloydev/QuickPID
   Based on the Arduino PID Library and work on AutoTunePID class
   by gnalbandian (Gonzalo). Licensed under the MIT License
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
    float input = *myInput;
    float dInput = input - lastInput;
    error = *mySetpoint - input;
    if (controllerDirection == REVERSE) {
      error = -error;
      dInput = -dInput;
    }
    if (kpi < 31 && kpd < 31) outputSum += FX_MUL(FL_FX(kpi) , error) - FX_MUL(FL_FX(kpd), (int)dInput); // fixed point
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

void QuickPID::AutoTune(uint8_t tuningRule)
{
  autoTune = new AutoTunePID(myInput, myOutput, (uint8_t)tuningRule);
}

void QuickPID::clearAutoTune()
{
  if (autoTune)
    delete autoTune;
}

AutoTunePID::AutoTunePID()
{
  _input = nullptr;
  _output = nullptr;
  reset();
}

AutoTunePID::AutoTunePID(float *input, float *output, uint8_t tuningRule)
{
  AutoTunePID();
  _input = input;
  _output = output;
  _tuningRule = tuningRule;
}

void AutoTunePID::reset()
{
  _t0 = 0;
  _t1 = 0;
  _t2 = 0;
  _t3 = 0;
  _Ku = 0.0;
  _Tu = 0.0;
  _td = 0.0;
  _kp = 0.0;
  _ki = 0.0;
  _kd = 0.0;
  _rdAvg = 0.0;
  _peakHigh = 0.0;
  _peakLow = 0.0;
}

void AutoTunePID::autoTuneConfig(const byte outputStep, const byte hysteresis, const int atSetpoint, const int atOutput, const bool dir, const bool printOrPlotter)
{
  _outputStep = outputStep;
  _hysteresis = hysteresis;
  _atSetpoint = atSetpoint;
  _atOutput = atOutput;
  _direction = dir;
  _printOrPlotter = printOrPlotter;
}

byte AutoTunePID::autoTuneLoop()
{
  switch (_autoTuneStage) {
    case STABILIZING:
      _peakHigh = _atSetpoint;
      _peakLow = _atSetpoint;
      if (_printOrPlotter == 1) Serial.print(F("Stabilizing →"));
      *_output = 0;
      _autoTuneStage++;
      break;
    case COARSE: // coarse adjust
      if (*_input < (_atSetpoint - _hysteresis)) {
        if (_printOrPlotter == 1) Serial.print(F(" coarse →"));
        (!_direction) ? *_output = _atOutput + _outputStep + 5 : *_output = _atOutput - _outputStep - 5;
        _autoTuneStage++;
      }
      break;
    case FINE: // fine adjust
      if (*_input > _atSetpoint) {
        if (_printOrPlotter == 1) Serial.print(F(" fine →"));
        (!_direction) ? *_output = _atOutput - _outputStep : *_output = _atOutput + _outputStep;
        _autoTuneStage++;
      }
      break;
    case AUTOTUNE: // run AutoTune relay method
      if (*_input < _atSetpoint) {
        if (_printOrPlotter == 1) Serial.print(F(" autotune →"));
        (!_direction) ? *_output = _atOutput + _outputStep : *_output = _atOutput - _outputStep;
        _autoTuneStage++;
      }
      break;
    case T0: // get t0
      if (*_input > _atSetpoint) {
        _t0 = micros();
        if (_printOrPlotter == 1) Serial.print(F(" t0 →"));
        _autoTuneStage++;
      }
      break;
    case T1: // get t1
      if (*_input > _atSetpoint + 0.2) {
        _t1 = micros();
        if (_printOrPlotter == 1) Serial.print(F(" t1 →"));
        _autoTuneStage++;
      }
      break;
    case T2: // get t2
      _rdAvg = *_input;
      if (_rdAvg > _peakHigh) _peakHigh = _rdAvg;
      if ((_rdAvg < _peakLow) && (_peakHigh >= (_atSetpoint + _hysteresis))) _peakLow = _rdAvg;

      if (_rdAvg > _atSetpoint + _hysteresis) {
        _t2 = micros();
        if (_printOrPlotter == 1) Serial.print(F(" t2 →"));
        (!_direction) ? *_output = _atOutput - _outputStep : *_output = _atOutput + _outputStep;
        _autoTuneStage++;
      }
      break;
    case T3: // get t3
      _rdAvg = *_input;
      if (_rdAvg > _peakHigh) _peakHigh = _rdAvg;
      if ((_rdAvg < _peakLow) && (_peakHigh >= (_atSetpoint + _hysteresis))) _peakLow = _rdAvg;
      if (_rdAvg < _atSetpoint - _hysteresis) {
        if (_printOrPlotter == 1) Serial.print(F(" t3 →"));
        (!_direction) ? *_output = _atOutput + _outputStep : *_output = _atOutput - _outputStep;
        _autoTuneStage++;
      }
      break;
    case DONE: // calculate everything
      _rdAvg = *_input;
      if (_rdAvg > _peakHigh) _peakHigh = _rdAvg;
      if ((_rdAvg < _peakLow) && (_peakHigh >= (_atSetpoint + _hysteresis))) _peakLow = _rdAvg;
      if (_rdAvg > _atSetpoint + _hysteresis) {
        _t3 = micros();
        if (_printOrPlotter == 1) Serial.println(F(" done."));
        _autoTuneStage++;
        _td = (float)(_t1 - _t0) / 1000000.0; // dead time (seconds)
        _Tu = (float)(_t3 - _t2) / 1000000.0; // ultimate period (seconds)
        _Ku = (float)(4 * _outputStep * 2) / (float)(3.14159 * sqrt (sq (_peakHigh - _peakLow) - sq (_hysteresis))); // ultimate gain
        if (_tuningRule == 6) { //AMIGO_PID
          (_td < 0.1) ? _td = 0.1 : _td = _td;
          _kp = (0.2 + 0.45 * (_Tu / _td)) / _Ku;
          float Ti = (((0.4 * _td) + (0.8 * _Tu)) / (_td + (0.1 * _Tu)) * _td);
          float Td = (0.5 * _td * _Tu) / ((0.3 * _td) + _Tu);
          _ki = _kp / Ti;
          _kd = Td * _kp;
        } else { //other rules
          _kp = RulesContants[_tuningRule][0] / 1000.0 * _Ku;
          _ki = RulesContants[_tuningRule][1] / 1000.0 * _Ku / _Tu;
          _kd = RulesContants[_tuningRule][2] / 1000.0 * _Ku * _Tu;
        }
        if (_printOrPlotter == 1) {
          // Controllability https://blog.opticontrols.com/wp-content/uploads/2011/06/td-versus-tau.png
          if ((_Tu / _td + 0.0001) > 0.75) Serial.println(F("This process is easy to control."));
          else if ((_Tu / _td + 0.0001) > 0.25) Serial.println(F("This process has average controllability."));
          else Serial.println(F("This process is difficult to control."));
          Serial.print(F("Tu: ")); Serial.print(_Tu);    // Ultimate Period (sec)
          Serial.print(F("  td: ")); Serial.print(_td);  // Dead Time (sec)
          Serial.print(F("  Ku: ")); Serial.print(_Ku);  // Ultimate Gain
          Serial.print(F("  Kp: ")); Serial.print(_kp);
          Serial.print(F("  Ki: ")); Serial.print(_ki);
          Serial.print(F("  Kd: ")); Serial.println(_kd);
          Serial.println();
        }
      }
      break;
    case NEW_TUNINGS: // ready to apply tunings
      *_output = 0;
      _autoTuneStage++;
      return NEW_TUNINGS;
      break;
  }
  return RUN_PID;
}

void AutoTunePID::setAutoTuneConstants(float* kp, float* ki, float* kd)
{
  *kp = _kp;
  *ki = _ki;
  *kd = _kd;
}
