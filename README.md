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

### Variables


| Variable     | Arduino PID | QuickPID | Data Type | Min        | Max        |
| :----------- | :---------- | :------- | :-------- | :--------- | :--------- |
| Setpoint     | double      | int16_t  | Binary    | 0          | 1023       |
| Input        | double      | int16_t  | Binary    | 0          | 1023       |
| Output       | double      | uint8_t  | Binary    | 0          | 255        |
| Kp           | double      | float    | float     | -3.402E+38 | 3.402E+38  |
| Ki           | double      | float    | float     | -3.402E+38 | 3.402E+38  |
| Kd           | double      | float    | float     | -3.402E+38 | 3.402E+38  |
| ratio        | double      | float    | float     | -3.402E+38 | 3.402E+38  |
| SampleTimeUs | double      | uint32_t | Binary    | 0          | 4294967295 |
| outputSum    | double      | int16_t  | Binary    | 0          | 255        |
| error        | double      | int32_t  | Binary    | -1023      | 1023       |
| dInput       | double      | int32_t  | Binary    | -1023      | 1023       |

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
