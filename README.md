# QuickPID

This API (version 2.01) follows the [ArduinoPID](https://github.com/br3ttb/Arduino-PID-Library) library, however there have been some significant updates as follows:

- Library named as **QuickPID** and can run concurrently with Arduino **PID**
- Reorganized and more efficient PID algorithms
- Fast fixed-point calculations for smaller coefficients, floating point calculations for larger coefficients
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


| Variable     | Arduino PID | QuickPID       | Data Type    | Resolution           | Bits Used | Min         | Max         |
| ------------ | ----------- | -------------- | ------------ | -------------------- | --------- | ----------- | ----------- |
| Setpoint     | double      | int16_t        | Binary       | 1                    | 10        | 0           | 1023        |
| Input        | double      | int16_t        | Binary       | 1                    | 10        | 0           | 1023        |
| Output       | double      | uint8_t        | Binary       | 1                    | 8         | 0           | 255         |
| Kp           | double      | int32_t, float | s23.8, float | 6-7 digits precision | 4 bytes   |             |             |
| Ki           | double      | int32_t, float | s23.8, float | 6-7 digits precision | 4 bytes   |             |             |
| Kd           | double      | int32_t, float | s23.8, float | 6-7 digits precision | 4 bytes   |             |             |
| ratio        | double      | float          | float        | 6-7 digits precision | 4 bytes   |             |             |
| SampleTimeUs | double      | uint32_t       | Binary       | 1                    | 32        | 0           | 4294967295  |
| outputSum    | double      | int16_t        | Binary       | 1                    | 8         | 0 (limit)   | 255 (limit) |
| error        | double      | int32_t        | Binary       | 1                    | 32        | -2147483648 | 2147483647  |
| dInput       | double      | int32_t        | Binary       | 1                    | 32        | -2147483648 | 2147483647  |

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
