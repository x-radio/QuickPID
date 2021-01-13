# QuickPID

This API (version 2.03) follows the [ArduinoPID](https://github.com/br3ttb/Arduino-PID-Library) library, however there have been some significant updates as follows:

- This library named as **QuickPID** and can run alongside with Arduino **PID** if needed
- Quicker fixed point math in compute function for small tuning values, floating point math used for large tuning values
- Reorganized and more efficient PID algorithm
- micros() timing resolution
- Faster analog read function
- `GetError()`and `GetpOnE()`functions added for diagnostics and control benefits
- Runs a complete PID cycle (*read-compute-write*) faster than just an `analogRead()` command  in Arduino

### Performance

| Compute                              | Kp   | Ki   | Kd   | Step Time (µS) |
| :----------------------------------- | ---- | ---- | ---- | -------------- |
| QuickPID                             | 2.0  | 15.0 | 0.05 | 68             |
| Arduino PID                          | 2.0  | 15.0 | 0.05 | 104            |
| **analogRead, Compute, analogWrite** |      |      |      |                |
| QuickPID                             | 2.0  | 5.0  | 0.2  | 96             |
| Arduino PID                          | 2.0  | 5.0  | 0.2  | 224            |

#### Self Test Example (RC Filter):

This example allows you to set an output voltage, then view the result of your tuning parameters. The mode of the P-Term automatically toggles from Proportional on Error to [Proportional on Measurement.](http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/)

![pid_self_test_pom](https://user-images.githubusercontent.com/63488701/104389509-a66a8f00-5509-11eb-927b-1190231a1ee9.gif)

### Simplified PID Algorithm 

| Proportional Term Mode      | Algorithm                                     |
| --------------------------- | --------------------------------------------- |
| Proportional on Error       | `outputSum += (kpi * error) - (kd * dInput);` |
| Proportional on Measurement | `outputSum += (ki * error) - (kpd * dInput);` |

### Variables


| Variable     | Arduino PID | QuickPID |
| :----------- | :---------- | :------- |
| Setpoint     | double      | int16_t  |
| Input        | double      | int16_t  |
| Output       | double      | int16_t  |
| Kp           | double      | float    |
| Ki           | double      | float    |
| Kd           | double      | float    |
| ratio        | double      | float    |
| SampleTimeUs | double      | uint32_t |
| outputSum    | double      | int16_t  |
| error        | double      | int16_t  |
| dInput       | double      | int16_t  |

### Original README

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
