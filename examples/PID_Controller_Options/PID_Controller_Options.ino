/**************************************************************************************
  For testing and checking parameter options. From QuickPID.h:

    enum class Mode : uint8_t {manual, automatic, timer};           // controller modes
    enum class Action : uint8_t {direct, reverse};                  // controller actions
    enum class pMode : uint8_t {pOnError, pOnMeas, pOnErrorMeas};   // proportional modes
    enum class dMode : uint8_t {dOnError, dOnMeas};                 // derivative modes
    enum class iAwMode : uint8_t {iAwCondition, iAwClamp, iAwOff};  // integral anti-windup modes

    float GetKp();            // proportional gain
    float GetKi();            // integral gain
    float GetKd();            // derivative gain
    float GetPterm();         // proportional component of output
    float GetIterm();         // integral component of output
    float GetDterm();         // derivative component of output
    uint8_t GetMode();        // manual (0), automatic (1) or timer (2)
    uint8_t GetDirection();   // direct (0), reverse (1)
    uint8_t GetPmode();       // pOnError (0), pOnMeas (1), pOnErrorMeas (2)
    uint8_t GetDmode();       // dOnError (0), dOnMeas (1)
    uint8_t GetAwMode();      // iAwCondition (0), iAwClamp (1), iAwOff (2)

  Documentation (GitHub): https://github.com/Dlloydev/QuickPID
 **************************************************************************************/

#include "QuickPID.h"

const byte inputPin = 0;
const byte outputPin = 3;

//Define variables we'll be connecting to
float Setpoint = 0, Input = 0, Output = 0, Kp = 2, Ki = 5, Kd = 1;

QuickPID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, myPID.pMode::pOnError, myPID.dMode::dOnMeas, myPID.iAwMode::iAwCondition, myPID.Action::direct);
/*                                                                  pOnError               dOnError                iAwCondition                direct
                                                                    pOnMeas                dOnMeas                 iAwClamp                    reverse
                                                                    pOnErrorMeas                                   iAwOff
*/

void setup()
{
  Serial.begin(115200);
  delay(2000);

  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTimeUs(100000);
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetMode(myPID.Control::automatic);
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
