# QuickPID   [![arduino-library-badge](https://www.ardu-badge.com/badge/QuickPID.svg?)](https://www.ardu-badge.com/QuickPID)

QuickPID is an updated implementation of the Arduino PID library with additional features for PID control. By default, this implementation closely follows the method of processing the p,i,d terms as in the PID_v1 library. One of the additional features includes integral anti-windup which can be based on conditionally using PI terms to provide some integral correction, prevent deep saturation and reduce overshoot. Anti-windup can also be based on clamping only, or it can be turned completely off. Also, the proportional term can be based on error, measurement, or both. The derivative term can be based on error or measurement.  PID controller modes include `timer`, which allows external timer or ISR timing control.

### Features

Development began with a fork of the Arduino PID Library. Modifications and new features have been added as described in the [releases](https://github.com/Dlloydev/QuickPID/releases).

#### New feature Summary

- [x] New functions added: `SetProportionalMode`, `SetDerivativeMode` and `SetAntiWindupMode`
- [x] `timer` mode for calling PID compute by an external timer function or ISR
- [x] `analogWrite()` support for ESP32 and ESP32-S2
- [x] Proportional on error `pOnError`, measurement `pOnMeas` or both `pOnErrorMeas` options
- [x] Derivative on error `dOnError` and measurement `dOnMeas` options
- [x] New PID Query Functions `GetPterm`, `GetIterm`, `GetDterm`, `GetPmode`, `GetDmode` and `GetAwMode`
- [x] New integral anti-windup options `iAwCondition`, `iAwClamp` and `iAwOff`
- [x] New `reverse` mode only changes sign of `error` and `dInput`
- [x] Uses `float` instead of `double`

#### Direct and Reverse Controller Action

Direct controller action leads the output to increase when the input is larger than the setpoint (i.e. heating process). Reverse controller leads the output to decrease when the input is larger than the setpoint (i.e. cooling process).

When the controller is set to `reverse` acting, the sign of the `error` and `dInput` (derivative of Input) is internally changed. All operating ranges and limits remain the same. To simulate a `reverse` acting process from a process that's  `direct` acting, the Input value needs to be "flipped". That is, if your reading from a 10-bit ADC with 0-1023 range, the input value used is (1023 - reading).

### Functions

#### QuickPID_Constructor

```c++
QuickPID::QuickPID(float* Input, float* Output, float* Setpoint, float Kp, float Ki, float Kd,
                   pMode pMode = pMode::pOnError, dMode dMode = dMode::dOnMeas,
                   iAwMode iAwMode = iAwMode::iAwCondition, Action action = Action::direct)
```

- `Input`, `Output`, and `Setpoint` are pointers to the variables holding these values.
- `Kp`, `Ki`, and `Kd` are the PID proportional, integral, and derivative gains.
- `pMode` is the proportional mode parameter with options for `pOnError` proportional on error (default), `pOnMeas`  proportional on measurement and `pOnErrorMeas` which is 0.5 `pOnError` + 0.5 `pOnMeas`.
- `dMode` is the derivative mode parameter with options for `dOnError` derivative on error, `dOnMeas` derivative on measurement (default).
- `awMode` is the integral anti-windup parameter with an option for `iAwCondition` which is based on PI terms to provide some integral correction, prevent deep saturation and reduce overshoot. The`iAwClamp` option (default), clamps the summation of the pmTerm and iTerm. The `iAwOff` option turns off all anti-windup.
- `Action` is the controller action parameter which has `direct` (default)  and `reverse` options. These options set how the controller responds to a change in input.  `direct` action is used if the input moves in the same direction as the controller output (i.e. heating process). `reverse` action is used if the input moves in the opposite direction as the controller output (i.e. cooling process).

```c++
QuickPID::QuickPID(float* Input, float* Output, float* Setpoint,
                   float Kp, float Ki, float Kd, Action action)
```

This allows you to use Proportional on Error without explicitly saying so.

```c++
QuickPID::QuickPID(float *Input, float *Output, float *Setpoint)
```

This simplified version allows you to define the remaining parameters later via specific setter functions. By default, Kp, Ki, and Kd will be initialized to zero, and should be later set via `SetTunings` to relevant values.

#### Compute

```c++
bool QuickPID::Compute();
```

This function contains the PID algorithm and it should be called once every loop(). Most of the time it will just return false without doing anything. However, at a  frequency specified by `SetSampleTime` it will calculate a new Output and return true.

#### SetTunings

```c++
void QuickPID::SetTunings(float Kp, float Ki, float Kd, pMode pMode = pMode::pOnError,
                          dMode dMode = dMode::dOnMeas, iAwMode iAwMode = iAwMode::iAwCondition)
```

This function allows the controller's dynamic performance to be adjusted. It's called automatically from the constructor, but tunings can also be adjusted on the fly during normal operation. The parameters are as described in the constructor.

```c++
void QuickPID::SetTunings(float Kp, float Ki, float Kd);
```

Set Tunings using the last remembered `pMode`, `dMode` and `iAwMode` settings. See example [PID_AdaptiveTunings.ino](https://github.com/Dlloydev/QuickPID/blob/master/examples/PID_AdaptiveTunings/PID_AdaptiveTunings.ino)

#### SetSampleTime

```c++
void QuickPID::SetSampleTimeUs(uint32_t NewSampleTimeUs);
```

Sets the period, in microseconds, at which the calculation is performed. The default is 100000µs (100ms).

#### SetOutputLimits

```c++
void QuickPID::SetOutputLimits(float Min, float Max);
```

The PID controller is designed to vary its output within a given range.  By default this range is 0-255, the Arduino PWM range.

#### SetMode

```c++
void QuickPID::SetMode(Control Mode);
```

Allows the controller Mode to be set to `manual` (0) or `automatic` (1) or `timer` (2). when the transition from manual to automatic  or timer occurs, the controller is automatically initialized.

`timer` mode is used when the PID compute is called by an external timer function or ISR. In this mode, the timer function and SetSampleTimeUs use the same time period value. The PID compute and timer will always remain in sync because the sample time variable and calculations remain constant. See examples:

- [PID_AVR_Basic_Interrupt_TIMER.ino](https://github.com/Dlloydev/QuickPID/blob/master/examples/PID_AVR_Basic_Interrupt_TIMER/PID_AVR_Basic_Interrupt_TIMER.ino)
- [PID_AVR_Basic_Software_TIMER.ino](https://github.com/Dlloydev/QuickPID/blob/master/examples/PID_AVR_Basic_Software_TIMER/PID_AVR_Basic_Software_TIMER.ino)

#### Initialize

```c++
void QuickPID::Initialize();
```

Does all the things that need to happen to ensure a bump-less transfer from manual to automatic mode.

#### SetControllerDirection

```c++
void QuickPID::SetControllerDirection(Action Action);
```

The PID will either be connected to a `direct` acting process (+Output leads to +Input) or a `reverse` acting process (+Output leads to -Input.) We need to know which one, because otherwise we may increase the output when we should be decreasing. This is called from the constructor.

#### SetProportionalMode

```c++
void QuickPID::SetProportionalMode(pMode pMode);
```

Sets the computation method for the proportional term, to compute based either on error (default), on measurement, or the average of both.

#### SetDerivativeMode

```c++
void QuickPID::SetDerivativeMode(dMode dMode);
```

Sets the computation method for the derivative term, to compute based either on error or on measurement (default).

#### SetAntiWindupMode

```c++
void QuickPID::SetAntiWindupMode(iAwMode iAwMode);
```

Sets the integral anti-windup mode to one of iAwClamp, which clamps the output after adding integral and proportional (on measurement) terms, or iAwCondition (default), which provides some integral correction, prevents deep saturation and reduces overshoot. Option iAwOff disables anti-windup altogether.

#### PID Query Functions

```c++
    float GetKp();            // proportional gain
    float GetKi();            // integral gain
    float GetKd();            // derivative gain
    float GetPterm();         // proportional component of output
    float GetIterm();         // integral component of output
    float GetDterm();         // derivative component of output
    uint8_t GetMode();        // manual (0), automatic (1) or timer (2)
    uint8_t GetDirection();   // direct (0), reverse (1)
    uint8_t GetPmode();       // pOnError (0), pOnMeas (1), pOnErrorMeas (2)
    uint8_t GetDmode();       // dOnError (0), dOnMeas (1)
    uint8_t GetAwMode();      // iAwCondition (0, iAwClamp (1), iAwOff (2)
```

These functions query the internal state of the PID.

#### AnalogWrite (PWM and DAC) for ESP32

If you're using QuickPID with an ESP32 and need analogWrite compatibility, there's no need to install a library as this feature is already included. AnalogWrite [documentation](https://github.com/Dlloydev/ESP32-ESP32S2-AnalogWrite)

### Original README (Arduino PID)

```
***************************************************************
* Arduino PID Library - Version 1.2.1
* by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
*
* This Library is licensed under the MIT License
***************************************************************
```

 - For a detailed explanation of the original code, please visit:
   http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

 - For function documentation see:  http://playground.arduino.cc/Code/PIDLibrary

------
