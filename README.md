# QuickPID   [![arduino-library-badge](https://www.ardu-badge.com/badge/QuickPID.svg?)](https://www.ardu-badge.com/QuickPID)

QuickPID is a fast fixed/floating point implementation of the Arduino PID library with built-in [AutoTune](https://github.com/Dlloydev/QuickPID/wiki/AutoTune) function. This controller can automatically determine and set parameters (Kp, Ki, Kd) and additionally determine the ultimate gain `Ku` and ultimate period `Tu`. There are 8 tuning rules available to choose from. Also, there is a POn setting that controls the mix of Proportional on Error to Proportional on Measurement.   

### About

This PID controller provides a faster *read-compute-write* cycle than alternatives as it has a more efficient and reduced algorithm that avoids time consuming partial calculations, it takes advantage of fixed point math and has a faster analog read function. The `Ki` and `Kd` are scaled by time (µs) and the new `kpi` and `kpd` parameters are calculated in the `SetTunings()` function. This results in a simple and fast algorithm with only two multiply operations required in `Compute()`.

### Features

Development began with a fork of the Arduino PID Library. Modifications and new features have been added as described in the change log and below:

- Quicker hybrid fixed/floating point math in compute function
- Built-in `AutoTune()` function automatically determines and sets `Kp`, `Ki` and `Kd`. and also ultimate gain `Ku` and ultimate period `Tu` of the control system. There are 8 tuning rules to choose from
- [AutoTune](https://github.com/Dlloydev/QuickPID/wiki/AutoTune) uses a precise and low control effort sequence that gets results quickly
- `POn` parameter now controls the setpoint weighting and mix of Proportional on Error to Proportional on Measurement
- Reorganized and more efficient PID algorithm, faster analog read function, micros() timing resolution
- Runs a complete PID cycle (*read-compute-write*) faster than just an `analogRead()` command  in Arduino

### Performance

| `Compute()`                                                  | Kp   | Ki   | Kd   | Step Time (µS) |
| :----------------------------------------------------------- | ---- | ---- | ---- | -------------- |
| QuickPID                                                     | 2.0  | 15.0 | 0.05 | 72             |
| Arduino PID                                                  | 2.0  | 15.0 | 0.05 | 104            |
| **Full PID cycle:** **`analogRead(), Compute(), analogWrite()`** |      |      |      |                |
| QuickPID using `analogReadFast()`                            | 2.0  | 5.0  | 0.2  | 96             |
| Arduino PID using `analogRead()`                             | 2.0  | 5.0  | 0.2  | 224            |

### [AutoTune RC Filter](https://github.com/Dlloydev/QuickPID/wiki/AutoTune_RC_Filter)

This example allows you to experiment with the AutoTune, various tuning rules and the POn control on an RC filter.

#### [QuickPID WiKi ...](https://github.com/Dlloydev/QuickPID/wiki)

### Simplified PID Algorithm

```c++
outputSum += (kpi * error) - (kpd * dInput);
```

The new `kpi` and `kpd` parameters are calculated in the `SetTunings()` function. This results in a simple and fast algorithm with only two multiply operations required. The pOn variable controls the setpoint weighting of Proportional on Error and Proportional on Measurement. The gains for `error` (`kpi`) and measurement `dInput` (`kpd`) are calculated as follows:

```c++
 kpi = kp * pOn + ki;
 kpd = kp * (1 - pOn) + kd;
```

### Functions

#### QuickPID_Constructor

```c++
QuickPID::QuickPID(int16_t* Input, int16_t* Output, int16_t* Setpoint,
                   float Kp, float Ki, float Kd, float POn, bool ControllerDirection)
```

- `Input`, `Output`, and `Setpoint` are pointers to the variables holding these values.
- `Kp`, `Ki`, and `Kd` are the PID proportional, integral, and derivative gains.
- `POn` is the Proportional on Error weighting value (0.0-1.0). This controls the mix of Proportional on Error (PonE) and  Proportional on Measurement (PonM) that's used in the compute algorithm. Note that POn controls the PonE amount, where the remainder (1-PonE) is the PonM amount. Also, the default POn is 1

![POn](https://user-images.githubusercontent.com/63488701/104958919-fe3c4680-599e-11eb-851e-73f26291d3e5.gif)

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

#### [AutoTune](https://github.com/Dlloydev/QuickPID/wiki/AutoTune)

```c++
void QuickPID::AutoTune(int inputPin, int outputPin, int tuningRule, int Print = 0, uint32_t timeout = 30)
```

The `AutoTune()` function automatically determines and sets `Kp`, `Ki` and `Kd`. It also determines the critical gain `Ku` and critical period `Tu` of the control system. 

`int Print = 0; // on(1), off(0)`

When using Serial Monitor, turn on serial print output to view the AutoTune status and results. When using the Serial Plotter, turn off the AutoTune print output to prevent plot labels from being overwritten.

`uint32_t timeout = 120`

Sets the AutoTune timeout where the default is 120 seconds.

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

Does all the things that need to happen to ensure a bump-less transfer from manual to automatic mode.

#### SetControllerDirection

```c++
void QuickPID::SetControllerDirection(bool Direction)
```

The PID will either be connected to a DIRECT acting process (+Output leads to +Input) or a REVERSE acting process (+Output leads to -Input.) We need to know which one, because otherwise we may increase the output when we should be decreasing. This is called from the constructor.

#### Display_Functions

```c++
float QuickPID::GetKp()
float QuickPID::GetKi()
float QuickPID::GetKd()
float QuickPID::GetKu()
float QuickPID::GetTu()
bool QuickPID::GetMode()
bool QuickPID::GetDirection()
```

These functions query the internal state of the PID. They're here for display purposes.

#### Utility_Functions

```c++
int QuickPID::analogReadFast(int ADCpin)
```

A faster configuration of `analogRead()`where a preset of 32 is used.  If the architecture definition isn't found, normal `analogRead()`is used to return a value.

### Change Log

#### [![arduino-library-badge](https://www.ardu-badge.com/badge/QuickPID.svg?)](https://www.ardu-badge.com/QuickPID)

- Improved AutoTune function
- Added 8 tuning rules
- Added `GetKu()` function to display ultimate gain
- Added `GetTu()` function to display ultimate period (time constant)
- Updated example and documentation

#### Version 2.1.0

- Added AutoTune function and documentation

- Added AutoTune_RC_Filter example and documentation

#### Version 2.0.5

- Added MIT license text file
- POn defaults to 1

#### Version 2.0.4

- Added `QuickPID_AdaptiveTunings.ino`, `QuickPID_Basic.ino`, `QuickPID_PonM.ino` and `QuickPID_RelayOutput.ino` to the examples folder.
- `QuickPID_RelayOutput.ino` has the added feature of `minWindow` setting that sets the minimum on time for the relay.

#### Version 2.0.3

- Initial Version with modifications as listed in [features.](#Features) 

------

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

------

