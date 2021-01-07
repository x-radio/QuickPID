# QuickPID

The API follows the [ArduinoPID](https://github.com/br3ttb/Arduino-PID-Library) library, however there have been some significant updates as follows:

- Library named as **QuickPID** and can run concurrently with Arduino **PID**
- Fixed-point calculations are used in `Compute()` to improve performance
- `analogReadFast()` added to provide further performance improvement
- `GetError()`added for diagnostics

### Performance

PID performance varies depending on how many coefficients are used.  When a  coefficient is zero, less calculation is done. The controller was  benchmarked using an Arduino UNO. QuickPID was benchmarked using analogReadFast() code.

| P_ON_M  Compute                               | Kp   | Ki   | Kd   | Step Time (µS) |
| :-------------------------------------------- | ---- | ---- | ---- | -------------- |
| QuickPID                                      | 2.0  | 5.0  | 0.0  | 88             |
| Arduino PID                                   | 2.0  | 5.0  | 0.0  | 104            |
| **P_ON_M   analogRead, Compute, analogWrite** |      |      |      |                |
| QuickPID                                      | 2.0  | 5.0  | 0.2  | 164            |
| Arduino PID                                   | 2.0  | 5.0  | 0.2  | 224            |

### Execution Frequency

A future version will provide further performance improvements by pre-calculating (scaling) the terms and providing direct timer with ISR support.

### Variables


| Variable     | Arduino PID | QuickPID         | Data Type    | Resolution | Bits Used | Min   | Max        |
| ------------ | ----------- | ---------------- | ------------ | ---------- | --------- | ----- | ---------- |
| Setpoint     | double      | int16_t          | Binary       | 1          | 10        | 0     | 1023       |
| Input        | double      | int16_t          | Binary       | 1          | 10        | 0     | 1023       |
| Output       | double      | uint8_t          | Binary       | 1          | 8         | 0     | 255        |
| Kp           | double      | int32_t, int64_t | s23.8, s55.8 | 0.00390625 | 10.8      | <-1m  | >1m        |
| Ki           | double      | int32_t, int64_t | s23.8, s55.8 | 0.00390625 | 10.8      | <-1m  | >1m        |
| Kd           | double      | int32_t          | s23.8        | 0.00390625 | 5.8       | -32   | 31.984375  |
| ratio        | double      | int32_t          | s23.8        | 0.00390625 | 5.8       | -32   | 31.984375  |
| SampleTimeUs | double      | uint32_t         | Binary       | 1          | 32        | 0     | 4294967295 |
| outputSum    | double      | int16_t          | Binary       | 1          | 10        | 0     | 1023       |
| error        | double      | int32_t          | Binary       | 1          | s10       | -1023 | 1023       |
| dInput       | double      | int32_t          | Binary       | 1          | s10       | -1023 | 1023       |

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
