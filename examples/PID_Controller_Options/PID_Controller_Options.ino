/**************************************************************************************
  For testing and checking parameter options. From QuickPID.h:

    enum class Mode : uint8_t {MANUAL, AUTOMATIC, TIMER}; // controller modes
    enum class Action : uint8_t {DIRECT, REVERSE};        // controller actions
    enum class pMode : uint8_t {PE, PM, PEM};             // proportional modes
    enum class dMode : uint8_t {DE, DM};                  // derivative modes
    enum class awMode : uint8_t {CONDITION, CLAMP, OFF};  // integral anti-windup modes

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
    uint8_t GetAwMode();      // CONDITION (0), CLAMP (1), OFF (2)

  Documentation (GitHub): https://github.com/Dlloydev/QuickPID
 **************************************************************************************/

#include "QuickPID.h"

const byte inputPin = 0;
const byte outputPin = 3;


//Define variables we'll be connecting to
float Setpoint = 0, Input = 0, Output = 0, Kp = 2, Ki = 5, Kd = 1;

QuickPID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, myPID.pMode::PE, myPID.dMode::DM, myPID.awMode::CONDITION, myPID.Action::DIRECT);
/*                                                                  PE               DE                CONDITION                DIRECT
                                                                    PM               DM                CLAMP                    REVERSE
                                                                    PEM                                OFF
*/

void setup()
{
  Serial.begin(115200);
  delay(2000);

  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTimeUs(100000);
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetMode(myPID.Control::AUTOMATIC);

  Input = analogRead(inputPin);
  myPID.Compute();

  analogWrite(outputPin, Output);

  Serial.println();
  Serial.print(F(" Setpoint: "));  Serial.println(Setpoint);
  Serial.print(F(" Input:    "));  Serial.println(Input);
  Serial.print(F(" Output:   "));  Serial.println(Output);
  Serial.print(F(" Pterm:    "));  Serial.println(myPID.GetPterm());
  Serial.print(F(" Iterm:    "));  Serial.println(myPID.GetIterm());
  Serial.print(F(" Dterm:    "));  Serial.println(myPID.GetDterm());
  Serial.print(F(" Control:  "));  Serial.println(myPID.GetMode());
  Serial.print(F(" Action:   "));  Serial.println(myPID.GetDirection());
  Serial.print(F(" Pmode:    "));  Serial.println(myPID.GetPmode());
  Serial.print(F(" Dmode:    "));  Serial.println(myPID.GetDmode());
  Serial.print(F(" AwMode:   "));  Serial.println(myPID.GetAwMode());

  analogWrite(outputPin, 0);
}

void loop()
{
}
