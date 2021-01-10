# QuickPID

This API (version 2.02) follows the [ArduinoPID](https://github.com/br3ttb/Arduino-PID-Library) library, however there have been some significant updates as follows:

- Library named as **QuickPID** and can run concurrently with Arduino **PID**
- Reorganized and more efficient PID algorithms
- micros() timing resolution
- Faster analog read function
- `GetError()`function added for diagnostics

### Performance

| Compute                              | Kp   | Ki   | Kd   | Step Time (µS) |
| :----------------------------------- | ---- | ---- | ---- | -------------- |
| QuickPID                             | 2.0  | 15.0 | 0.05 | 72             |
| Arduino PID                          | 2.0  | 15.0 | 0.05 | 104            |
| **analogRead, Compute, analogWrite** |      |      |      |                |
| QuickPID                             | 2.0  | 5.0  | 0.2  | 132            |
| Arduino PID                          | 2.0  | 5.0  | 0.2  | 224            |

#### Self Test Example (RC Filter): P_ON_M

![pid_self_test_pom](https://user-images.githubusercontent.com/63488701/104115407-2cee5900-52dd-11eb-9b24-ff06d39fd2d6.gif)

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
