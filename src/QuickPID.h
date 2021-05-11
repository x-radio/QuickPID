#pragma once
#ifndef QuickPID_h
#define QuickPID_h

class QuickPID {

  public:

    // controller mode
    typedef enum { MANUAL = 0, AUTOMATIC = 1 } mode_t;

    // DIRECT: intput increases when the error is positive. REVERSE: intput decreases when the error is positive.
    typedef enum { DIRECT = 0, REVERSE = 1 } direction_t;

    // commonly used functions ************************************************************************************

    // Constructor. Links the PID to Input, Output, Setpoint and initial Tuning Parameters.
    QuickPID(float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, float POn, direction_t ControllerDirection);

    // Overload constructor with proportional mode. Links the PID to Input, Output, Setpoint and Tuning Parameters.
    QuickPID(float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, direction_t ControllerDirection);

    // Sets PID to either Manual (0) or Auto (non-0).
    void SetMode(mode_t Mode);

    // Performs the PID calculation. It should be called every time loop() cycles. ON/OFF and calculation frequency
    // can be set using SetMode and SetSampleTime respectively.
    bool Compute();

    // Automatically determines and sets the tuning parameters Kp, Ki and Kd. Use this in the setup loop.
    void AutoTune(int inputPin, int outputPin, int tuningRule, int Print, uint32_t timeout);

    // Sets and clamps the output to a specific range (0-255 by default).
    void SetOutputLimits(int Min, int Max);

    // available but not commonly used functions ******************************************************************

    // While most users will set the tunings once in the constructor, this function gives the user the option of
    // changing tunings during runtime for Adaptive control.
    void SetTunings(float Kp, float Ki, float Kd);

    // Overload for specifying proportional mode.
    void SetTunings(float Kp, float Ki, float Kd, float POn);

    // Sets the controller Direction or Action. DIRECT means the output will increase when the error is positive.
    // REVERSE means the output will decrease when the error is positive.
    void SetControllerDirection(direction_t ControllerDirection);

    // Sets the sample time in microseconds with which each PID calculation is performed. Default is 100000 µs.
    void SetSampleTimeUs(uint32_t NewSampleTimeUs);

    //Display functions ******************************************************************************************
    float GetKp();         // These functions query the pid for interal values. They were created mainly for
    float GetKi();         // the pid front-end, where it's important to know what is actually inside the PID.
    float GetKd();
    mode_t GetMode();
    direction_t GetDirection();
    int analogReadFast(int ADCpin);

  private:

    void Initialize();

    float dispKp;       // tuning parameters for display purposes.
    float dispKi;
    float dispKd;

    float pOn;          // proportional mode (0-1) default = 1 (100% Proportional on Error)
    float kp;           // (P)roportional Tuning Parameter
    float ki;           // (I)ntegral Tuning Parameter
    float kd;           // (D)erivative Tuning Parameter
    float kpi;          // proportional on error amount
    float kpd;          // proportional on measurement amount

    float *myInput;     // Pointers to the Input, Output, and Setpoint variables. This creates a
    float *myOutput;    // hard link between the variables and the PID, freeing the user from having
    float *mySetpoint;  // to constantly tell us what these values are. With pointers we'll just know.

    direction_t controllerDirection;
    uint32_t sampleTimeUs, lastTime;
    int outMin, outMax, error;
    int lastInput, outputSum;
    bool inAuto;

    inline int32_t FL_FX(float a) {
      return (a * 256.0); // float to fixed point
    }
    inline int32_t FX_MUL(int32_t a, int32_t b) {
      return ((a * b) >> 8); // fixed point multiply
    }
    inline int32_t CONSTRAIN(int32_t x, int32_t lower, int32_t upper) {
      return ((x) < (lower) ? (lower) : ((x) > (upper) ? (upper) : (x)));
    }

}; // class QuickPID

#if (defined(ESP32) || defined(ARDUINO_ARCH_ESP32))
#include "analogWrite.h"
#endif
#endif // QuickPID.h
