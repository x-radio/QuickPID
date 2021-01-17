# QuickPID

A fast hybrid fixed-point and floating-point PID controller for Arduino. 

### About

This PID controller provides a faster *read-compute-write* cycle than alternatives as it has a more efficient and reduced algorithm that avoids time consuming partial calculations, it takes advantage of fixed point math and has a faster analog read function. The `Ki` and `Kd` are scaled by time (µs) and the new `kpi` and `kpd` parameters are calculated in the `SetTunings()` function. This results in a simple and fast algorithm with only two multiply operations required in `Compute()`.

### Features

Development began with a fork of the Arduino PID Library. Some modifications and new features have been added as follows:

- Quicker hybrid fixed/floating point math in compute function
- `POn` parameter now controls the setpoint weighting of Proportional on Error and Proportional on Measurement
- Reorganized and more efficient PID algorithm
- micros() timing resolution
- Faster analog read function
- `GetError()` function added
- Runs a complete PID cycle (*read-compute-write*) faster than just an `analogRead()` command  in Arduino

### Performance

| `Compute()`                                                  | Kp   | Ki   | Kd   | Step Time (µS) |
| :----------------------------------------------------------- | ---- | ---- | ---- | -------------- |
| QuickPID                                                     | 2.0  | 15.0 | 0.05 | 72             |
| Arduino PID                                                  | 2.0  | 15.0 | 0.05 | 104            |
| **Full PID cycle:** **`analogRead(), Compute(), analogWrite()`** |      |      |      |                |
| QuickPID using `analogReadFast()`                            | 2.0  | 5.0  | 0.2  | 96             |
| Arduino PID using `analogRead()`                             | 2.0  | 5.0  | 0.2  | 224            |

### Functions

[QuickPID Constructor](#QuickPID Constructor)

[Compute](#Compute)

[SetTunings](#SetTunings)

[SetSampleTime](#SetSampleTime)

[SetOutputLimits](#SetOutputLimits)

[SetMode](#SetMode)

[Initialize](#Initialize)

[SetControllerDirection](#SetControllerDirection)

[Display Functions](#Display Functions)

[Utility Functions](#Utility Functions)

### Self Test Example (RC Filter):

[This example](https://github.com/Dlloydev/QuickPID/wiki/QuickPID_RC_Filter) allows you to experiment with the four tuning parameters.

![pid_self_test_pom](https://user-images.githubusercontent.com/63488701/104389509-a66a8f00-5509-11eb-927b-1190231a1ee9.gif)

### Simplified PID Algorithm

```c++
outputSum += (kpi * error) - (kpd * dInput);
```

The new `kpi` and `kpd` parameters are calculated in the `SetTunings()` function. This results in a simple and fast algorithm with only two multiply operations required The POn parameter controls the setpoint weighting of Proportional on Error and Proportional on Measurement. The gains for `error` (`kpi`) and measurement `dInput` (`kpd`) are calculated in the `QuickPID::SetTunings()` function as follows:

```c++
 kpi = kp * pOn + ki;
 kpd = kp * (1 - pOn) + kd;
```

#### QuickPID Constructor

```c++
QuickPID::QuickPID(int16_t* Input, int16_t* Output, int16_t* Setpoint,
                   float Kp, float Ki, float Kd, float POn, bool ControllerDirection)
```

- `Input`, `Output`, and `Setpoint` are pointers to the variables holding these values.
- `Kp`, `Ki`, and `Kd` are the PID proportional, integral, and derivative gains.
- `POn` is the Proportional on Error weighting value (0.0-1.0). This controls the amount of Proportional on Error and  Proportional on Measurement factor that's used in the compute algorithm.
- `ControllerDirection` Either DIRECT or REVERSE determines which direction the output will move for a given error. DIRECT is most common.

```c++
QuickPID::QuickPID(int16_t* Input, int16_t* Output, int16_t* Setpoint,
                   float Kp, float Ki, float Kd, bool ControllerDirection)
```

This allows you to use Proportional on Error without explicitly saying so.

#### Compute

```c++
bool QuickPID::Compute()
```

This function contains the PID algorithm and it should be called once every loop(). Most of the time it will just return false without doing anything. However, at a  frequency specified by `SetSampleTime` it will calculate a new Output and return true.

#### SetTunings

```c++
void QuickPID::SetTunings(float Kp, float Ki, float Kd, float POn)
```

This function allows the controller's dynamic performance to be adjusted. It's called automatically from the constructor, but tunings can also be adjusted on the fly during normal operation. The parameters are as described in the constructor.

```c++
void QuickPID::SetTunings(float Kp, float Ki, float Kd)
```

Set Tunings using the last remembered POn setting.

#### SetSampleTime

```c++
void QuickPID::SetSampleTimeUs(uint32_t NewSampleTimeUs)
```

Sets the period, in microseconds, at which the calculation is performed. The default is 100ms.

#### SetOutputLimits

```c++
void QuickPID::SetOutputLimits(int16_t Min, int16_t Max)
```

The PID controller is designed to vary its output within a given range.  By default this range is 0-255, the Arduino PWM range.

#### SetMode

```c++
void QuickPID::SetMode(bool Mode)
```

Allows the controller Mode to be set to `MANUAL` (0) or `AUTOMATIC` (non-zero). when the transition from manual to automatic occurs, the controller is automatically initialized.

#### Initialize

```c++
void QuickPID::Initialize()
```

Does all the things that need to happen to ensure a bumpless transfer from manual to automatic mode.

#### SetControllerDirection

```c++
void QuickPID::SetControllerDirection(bool Direction)
```

The PID will either be connected to a DIRECT acting process (+Output leads to +Input) or a REVERSE acting process (+Output leads to -Input.) We need to know which one, because otherwise we may increase the output when we should be decreasing. This is called from the constructor.

#### Display Functions

```c++
float QuickPID::GetKp()
float QuickPID::GetKi()
float QuickPID::GetKd()
bool QuickPID::GetMode()
bool QuickPID::GetDirection()
int16_t QuickPID::GetError()
```

These functions query the internal state of the PID. They're here for display purposes.

#### Utility Functions

```c++
int QuickPID::analogReadFast(int ADCpin)
```

A faster configuration of `analogRead()`where a preset of 32 is used. Works with the following defines:

`__AVR_ATmega328P__`
`__AVR_ATtiny_Zero_One__`
`__AVR_ATmega_Zero__`
`__AVR_DA__`

Uses Arduino's default settings and returns `analogRead()` if the definition isn't found. 

### Original README (Arduino PID)

```
***************************************************************
* Arduino PID Library - Version 1.2.1
* by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
*
* This Library is licensed under the MIT License
***************************************************************
```

 - For an ultra-detailed explanation of why the code is the way it is, please visit:
   http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

 - For function documentation see:  http://playground.arduino.cc/Code/PIDLibrary
