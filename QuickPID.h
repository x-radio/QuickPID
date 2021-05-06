#ifndef QuickPID_h
#define QuickPID_h

class QuickPID {

  public:

    //Constants and macros
#define AUTOMATIC 1
#define MANUAL  0
#define DIRECT  0
#define REVERSE 1

static const byte TRY_DIRECT = 0;
static const byte TRY_AUTOMATIC = 1;

#define FL_FX(a) (int32_t)(a*256.0)  // float to fixed point
#define FX_MUL(a,b) ((a*b)>>8)       // fixed point multiply
#define CONSTRAIN(x,lower,upper)    ((x)<(lower)?(lower):((x)>(upper)?(upper):(x)))

    // commonly used functions ************************************************************************************

    // Constructor. Links the PID to Input, Output, Setpoint and initial Tuning Parameters.
    QuickPID(float*, float*, float*, float, float, float, float, uint8_t);

    // Overload constructor with proportional mode. Links the PID to Input, Output, Setpoint and Tuning Parameters.
    QuickPID(float*, float*, float*, float, float, float, uint8_t);

    // Sets PID to either Manual (0) or Auto (non-0).
    void SetMode(uint8_t Mode);

    // Performs the PID calculation. It should be called every time loop() cycles. ON/OFF and calculation frequency
    // can be set using SetMode and SetSampleTime respectively.
    bool Compute();

    // Automatically determines and sets the tuning parameters Kp, Ki and Kd. Use this in the setup loop.
    void AutoTune(int, int, int, int, uint32_t);

    // Sets and clamps the output to a specific range (0-255 by default).
    void SetOutputLimits(int, int);

    // available but not commonly used functions ******************************************************************

    // While most users will set the tunings once in the constructor, this function gives the user the option of
    // changing tunings during runtime for Adaptive control.
    void SetTunings(float, float, float);

    // Overload for specifying proportional mode.
    void SetTunings(float, float, float, float);

    // Sets the Direction, or "Action" of control. DIRECT means the output will increase when error is positive.
    // REVERSE means the opposite. It's very unlikely that this will be needed once it is set in the constructor.
    void SetControllerDirection(uint8_t);

    // Sets the sample time in milliseconds with which each PID calculation is performed. Default is 100.
    void SetSampleTimeUs(uint32_t);

    //Display functions ******************************************************************************************
    float GetKp();         // These functions query the pid for interal values. They were created mainly for
    float GetKi();         // the pid front-end, where it's important to know what is actually inside the PID.
    float GetKd();
    float GetKu();         // Ultimate Gain
    float GetTu();         // Ultimate Period
    float GetTd();         // Dead Time
    uint8_t GetMode();
    uint8_t GetDirection();

    // Utility functions ******************************************************************************************
    int analogReadFast(int);
    float analogReadAvg(int);

  private:
    void Initialize();
    void CheckPeak(int);
    void StepUp(int, int, uint32_t);
    void StepDown(int, int, uint32_t);
    void Stabilize(int, int, uint32_t);

    float dispKp;      // tuning parameters for display purposes.
    float dispKi;
    float dispKd;
    float dispKu;
    float dispTu;
    float dispTd;

    float pOn;         // proportional mode (0-1) default = 1, 100% Proportional on Error
    float kp;          // (P)roportional Tuning Parameter
    float ki;          // (I)ntegral Tuning Parameter
    float kd;          // (D)erivative Tuning Parameter
    float kpi;         // proportional on error amount
    float kpd;         // proportional on measurement amount

    uint8_t controllerDirection;

    float *myInput;      // Pointers to the Input, Output, and Setpoint variables. This creates a
    float *myOutput;     // hard link between the variables and the PID, freeing the user from having
    float *mySetpoint;   // to constantly tell us what these values are. With pointers we'll just know.

    uint32_t sampleTimeUs, lastTime;
    int outMin, outMax, error;
    int lastInput, outputSum;
    bool inAuto;

    // AutoTune
    float peakHigh, peakLow;
    const word readPeriod = 1667;
    const byte outputStep = 1;
    const byte hysteresis = 1;
    const int atSetpoint = 341;  // 1/3 of 10-bit ADC matches 8-bit PWM value of 85 for symetrical waveform
    const int atOutput = 85;

};

#if (defined(ESP32) || defined(ARDUINO_ARCH_ESP32))
#include "analogWrite.h"
#endif

#endif // QuickPID.h
