#ifndef QuickPID_h
#define QuickPID_h

class QuickPID
{

  public:

    //Constants used in some of the functions below
#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE 1
#define P_ON_M  0
#define P_ON_E  1

    // s23.8 fixed point defines
#define FL__FX(a) (int64_t)(a*256.0) // float to fixed point
#define FL_FX(a) (int32_t)(a*256.0)  // float to fixed point
#define FX_FL(a) (float)(a/256.0)    // fixed point to float
#define INT_FX(a) (a<<8)             // integer to fixed point
#define FX_INT(a) (int32_t)(a>>8)    // fixed point to integer
#define FX_ADD(a,b) (a+b)            // fixed point add
#define FX_SUB(a,b) (a-b)            // fixed point subtract 
#define FX_MUL(a,b) ((a*b)>>8)       // fixed point multiply
#define FX_DIV(a,b) ((a/b)<<8)       // fixed point divide


    // commonly used functions ************************************************************************************

    // Constructor. Links the PID to Input, Output, Setpoint and initial Tuning Parameters.
    QuickPID(int16_t*, uint8_t*, int16_t*, float, float, float, bool, bool);

    // Overload constructor with proportional mode. Links the PID to Input, Output, Setpoint and Tuning Parameters.
    QuickPID(int16_t*, uint8_t*, int16_t*, float, float, float, bool);

    // Sets PID to either Manual (0) or Auto (non-0).
    void SetMode(bool Mode);

    // Performs the PID calculation. It should be called every time loop() cycles. ON/OFF and calculation frequency
    // can be set using SetMode and SetSampleTime respectively.
    bool Compute();

    // Clamps the output to a specific range. 0-255 by default, but it's likely the user will want to change this
    // depending on the application.
    void SetOutputLimits(uint8_t, uint8_t);

    // available but not commonly used functions ******************************************************************

    // While most users will set the tunings once in the constructor, this function gives the user the option of
    // changing tunings during runtime for Adaptive control.
    void SetTunings(float, float, float);

    // Overload for specifying proportional mode.
    void SetTunings(float, float, float, bool);

    // Sets the Direction, or "Action" of control. DIRECT means the output will increase when error is positive.
    // REVERSE means the opposite. It's very unlikely that this will be needed once it is set in the constructor.
    void SetControllerDirection(bool);

    // Sets the sample time in milliseconds with which each PID calculation is performed. Default is 100.
    void SetSampleTimeUs(uint32_t);

    //Display functions ******************************************************************************************
    float GetKp();         // These functions query the pid for interal values. They were created mainly for
    float GetKi();         // the pid front-end, where it's important to know what is actually inside the PID.
    float GetKd();
    int16_t GetError();
    bool GetMode();
    bool GetDirection();

    // Utility functions ******************************************************************************************
    int analogReadFast(int);

  private:
    void Initialize();

    float dispKp;          // We'll hold on to the tuning parameters in user-entered format for display purposes.
    float dispKi;
    float dispKd;

    float kp;              // (P)roportional Tuning Parameter
    float ki;              // (I)ntegral Tuning Parameter
    float kd;              // (D)erivative Tuning Parameter

    bool controllerDirection;
    bool pOn;

    int16_t *myInput;      // Pointers to the Input, Output, and Setpoint variables. This creates a
    uint8_t *myOutput;     // hard link between the variables and the PID, freeing the user from having
    int16_t *mySetpoint;   // to constantly tell us what these values are. With pointers we'll just know.

    uint32_t SampleTimeUs, lastTime;
    int16_t lastInput, outputSum;
    uint8_t outMin, outMax;
    bool inAuto, pOnE;
    int32_t error;
};

#endif
