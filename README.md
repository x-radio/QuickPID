# QuickPID   [![arduino-library-badge](https://www.ardu-badge.com/badge/QuickPID.svg?)](https://www.ardu-badge.com/QuickPID)

QuickPID is an updated implementation of the Arduino PID library with a built-in [AutoTune](https://github.com/Dlloydev/QuickPID/wiki/AutoTune) class as a dynamic object  to reduce memory if not used, thanks to contributions by [gnalbandian (Gonzalo)](https://github.com/gnalbandian). This controller can automatically determine and set parameters `Kp, Ki, Kd`. Additionally the Ultimate Gain `Ku`, Ultimate Period `Tu`, Dead Time `td` and determine how easy the process is to control. There are 10 tuning rules available to choose from. Also available is a POn setting that controls the mix of Proportional on Error to Proportional on Measurement. 

### Features

Development began with a fork of the Arduino PID Library. Modifications and new features have been added as described in the [change log](https://github.com/Dlloydev/QuickPID/wiki/Change-Log).

#### New feature Summary

- [x] `analogReadFast()` support for AVR (4x faster)
- [x] `analogWrite()` support for ESP32 and ESP32-S2 
- [x] Variable Proportional on Error Proportional on Measurement parameter `POn`
- [x]  Integral windup prevention when output exceeds limits
- [x] New PID query functions that return the P, I and D terms of the calculation
- [x] New AutoTune class added as a dynamic object and includes 10 tuning methods
- [x] AutoTune is compatible with reverse acting controllers
- [x] AutoTune's fast, non-blocking tuning sequence completes in only 1.5 oscillations 
- [x] AutoTune determines how easy the process is to control
- [x] AutoTune determines ultimate period `Tu`, dead time `td`, ultimate gain `Ku`, and tuning parameters `Kp, Ki, Kd`
- [x] New REVERSE mode only changes sign of `error` and `dInput`
- [x] Uses `float` instead of `double`

### [AutoTune RC Filter](https://github.com/Dlloydev/QuickPID/wiki/AutoTune_RC_Filter)

This example allows you to experiment with the AutoTunePID class, various tuning rules and the POn control using ADC and PWM with RC filter. It automatically determines and sets the tuning parameters and works with both DIRECT and REVERSE acting controllers.

#### [QuickPID WiKi ...](https://github.com/Dlloydev/QuickPID/wiki)

### Direct and Reverse Controller Action

If a positive error increases the controller's output, the controller is said to be direct acting (i.e. heating process). When a positive error decreases the controller's output, the controller is said to be reverse acting (i.e. cooling process). When the controller is set to `REVERSE` acting, the sign of the `error` and `dInput` (derivative of Input) is internally changed. All operating ranges and limits remain the same. To simulate a `REVERSE` acting process from a process that's  `DIRECT` acting, the Input value needs to be "flipped". That is, if your reading from a 10-bit ADC with 0-1023 range, the input value used is (1023 - reading). See the examples  [AutoTune_Filter_DIRECT.ino](https://github.com/Dlloydev/QuickPID/blob/master/examples/AutoTune_Filter_DIRECT/AutoTune_Filter_DIRECT.ino) and [AutoTune_Filter_REVERSE.ino](https://github.com/Dlloydev/QuickPID/blob/master/examples/AutoTune_Filter_REVERSE/AutoTune_Filter_REVERSE.ino) for details.

### Functions

#### QuickPID_Constructor

```c++
QuickPID::QuickPID(float* Input, float* Output, float* Setpoint,
                   float Kp, float Ki, float Kd, float POn, uint8_t ControllerDirection);
```

- `Input`, `Output`, and `Setpoint` are pointers to the variables holding these values.
- `Kp`, `Ki`, and `Kd` are the PID proportional, integral, and derivative gains.
- `POn` is the Proportional on Error weighting value with range 0.0-1.0 and default 1.0 (100% Proportional on Error). This controls the mix of Proportional on Error to Proportional on Measurement.

![image](https://user-images.githubusercontent.com/63488701/118383726-986b6e80-b5ce-11eb-94b8-fdbddd4c914e.png)

- `ControllerDirection` Either DIRECT or REVERSE determines which direction the output will move for a given error. 

```c++
QuickPID::QuickPID(float* Input, float* Output, float* Setpoint,
                   float Kp, float Ki, float Kd, uint8_t ControllerDirection);
```

This allows you to use Proportional on Error without explicitly saying so.

#### Compute

```c++
bool QuickPID::Compute();
```

This function contains the PID algorithm and it should be called once every loop(). Most of the time it will just return false without doing anything. However, at a  frequency specified by `SetSampleTime` it will calculate a new Output and return true.

#### [AutoTune](https://github.com/Dlloydev/QuickPID/wiki/AutoTune)

For use of AutoTune, refer to the examples [AutoTune_Filter_DIRECT.ino](https://github.com/Dlloydev/QuickPID/blob/master/examples/AutoTune_Filter_DIRECT/AutoTune_Filter_DIRECT.ino) and [AutoTune_Filter_REVERSE.ino](https://github.com/Dlloydev/QuickPID/blob/master/examples/AutoTune_Filter_REVERSE/AutoTune_Filter_REVERSE.ino)

#### SetTunings

```c++
void QuickPID::SetTunings(float Kp, float Ki, float Kd, float POn);
```

This function allows the controller's dynamic performance to be adjusted. It's called automatically from the constructor, but tunings can also be adjusted on the fly during normal operation. The parameters are as described in the constructor.

```c++
void QuickPID::SetTunings(float Kp, float Ki, float Kd);
```

Set Tunings using the last remembered POn setting.

#### SetSampleTime

```c++
void QuickPID::SetSampleTimeUs(uint32_t NewSampleTimeUs);
```

Sets the period, in microseconds, at which the calculation is performed. The default is 100ms.

#### SetOutputLimits

```c++
void QuickPID::SetOutputLimits(int Min, int Max);
```

The PID controller is designed to vary its output within a given range.  By default this range is 0-255, the Arduino PWM range.

#### SetMode

```c++
void QuickPID::SetMode(uint8_t Mode);
```

Allows the controller Mode to be set to `MANUAL` (0) or `AUTOMATIC` (non-zero). when the transition from manual to automatic occurs, the controller is automatically initialized.

#### Initialize

```c++
void QuickPID::Initialize();
```

Does all the things that need to happen to ensure a bump-less transfer from manual to automatic mode.

#### SetControllerDirection

```c++
void QuickPID::SetControllerDirection(uint8_t Direction)
```

The PID will either be connected to a DIRECT acting process (+Output leads to +Input) or a REVERSE acting process (+Output leads to -Input.) We need to know which one, because otherwise we may increase the output when we should be decreasing. This is called from the constructor.

#### PID Query Functions

```c++
    float GetKp();               // proportional gain
    float GetKi();               // integral gain
    float GetKd();               // derivative gain
    float GetPeTerm();           // proportional on error component of output 
    float GetPmTerm();           // proportional on measurement component of output
    float GetIterm();            // integral component of output
    float GetDterm();            // derivative component of output
    mode_t GetMode();            // MANUAL (0) or AUTOMATIC (1)
    direction_t GetDirection();  // DIRECT (0) or REVERSE (1)
```

These functions query the internal state of the PID.

#### Utility Functions

```c++
int QuickPID::analogReadFast(int ADCpin)
```

A faster configuration of `analogRead()`where a preset of 32 is used.  If the architecture definition isn't found, normal `analogRead()`is used to return a value.

#### [AnalogWrite (PWM and DAC) for ESP32](https://github.com/Dlloydev/ESP32-ESP32S2-AnalogWrite)

Use this link for reference. Note that if you're using QuickPID, there's no need to install the AnalogWrite library as this feature is already included.

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

