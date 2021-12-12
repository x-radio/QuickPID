#pragma once
#ifndef QuickPID_h
#define QuickPID_h

class QuickPID {

  public:

    enum Mode : uint8_t {MANUAL, AUTOMATIC, TIMER}; // controller modes
    enum Action : uint8_t {DIRECT, REVERSE};        // controller actions
    enum pMode : uint8_t {PE, PM, PEM};             // proportional modes
    enum dMode : uint8_t {DE, DM};                  // derivative modes
    enum awMode : uint8_t {CONDITION, CLAMP, OFF};  // integral anti-windup modes

    // commonly used functions ************************************************************************************

    // Constructor. Links the PID to Input, Output, Setpoint and initial Tuning Parameters.
    QuickPID(float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, uint8_t pMode, uint8_t dMode, uint8_t awMode, uint8_t Action);

    // Overload constructor with proportional ratio. Links the PID to Input, Output, Setpoint and Tuning Parameters.
    QuickPID(float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, uint8_t Action);

    // Sets PID mode to MANUAL (0), AUTOMATIC (1) or TIMER (2).
    void SetMode(uint8_t Mode);

    // Performs the PID calculation. It should be called every time loop() cycles ON/OFF and calculation frequency
    // can be set using SetMode and SetSampleTime respectively.
    bool Compute();

    // Sets and clamps the output to a specific range (0-255 by default).
    void SetOutputLimits(float Min, float Max);

    // available but not commonly used functions ******************************************************************

    // While most users will set the tunings once in the constructor, this function gives the user the option of
    // changing tunings during runtime for Adaptive control.
    void SetTunings(float Kp, float Ki, float Kd);

    // Overload for specifying proportional ratio.
    void SetTunings(float Kp, float Ki, float Kd, uint8_t pMode, uint8_t dMode, uint8_t awMode);

    // Sets the controller Direction or Action. DIRECT means the output will increase when the error is positive.
    // REVERSE means the output will decrease when the error is positive.
    void SetControllerDirection(uint8_t Action);

    // Sets the sample time in microseconds with which each PID calculation is performed. Default is 100000 µs.
    void SetSampleTimeUs(uint32_t NewSampleTimeUs);

    // PID Query functions ***********************************************************************************
    float GetKp();            // proportional gain
    float GetKi();            // integral gain
    float GetKd();            // derivative gain
    float GetPterm();         // proportional component of output
    float GetIterm();         // integral component of output
    float GetDterm();         // derivative component of output
    uint8_t GetMode();        // MANUAL (0), AUTOMATIC (1) or TIMER (2)
    uint8_t GetDirection();   // DIRECT (0), REVERSE (1)
    uint8_t GetPmode();       // PE (0), PM (1), PEM (2)
    uint8_t GetDmode();       // DE (0), DM (1)
    uint8_t GetAwMode();      // CONDITION (0, CLAMP (1), OFF (2)

  private:

    void Initialize();

    float dispKp;       // tuning parameters for display purposes.
    float dispKi;
    float dispKd;
    float pTerm;
    float iTerm;
    float dTerm;

    float kp;           // (P)roportional Tuning Parameter
    float ki;           // (I)ntegral Tuning Parameter
    float kd;           // (D)erivative Tuning Parameter

    float *myInput;     // Pointers to the Input, Output, and Setpoint variables. This creates a
    float *myOutput;    // hard link between the variables and the PID, freeing the user from having
    float *mySetpoint;  // to constantly tell us what these values are. With pointers we'll just know.

    uint8_t mode = MANUAL;
    uint8_t action = DIRECT;
    uint8_t pmode = PE;
    uint8_t dmode = DM;
    uint8_t awmode = CONDITION;

    uint32_t sampleTimeUs, lastTime;
    float outputSum, outMin, outMax, error, lastError, lastInput;

}; // class QuickPID

#if (defined(ESP32) || defined(ARDUINO_ARCH_ESP32))
#include "analogWrite.h"
#endif
#endif // QuickPID.h
