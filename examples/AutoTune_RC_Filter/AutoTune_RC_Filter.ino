/******************************************************************************
   AutoTune RC Filter Example
   Use Serial Monitor and Serial Plotter to view results.
   Reference: https://github.com/Dlloydev/QuickPID/wiki/AutoTune
   Circuit: https://github.com/Dlloydev/QuickPID/wiki/AutoTune_RC_Filter

   TUNING RULE             RECOMMENED FOR
   0 ZIEGLER_NICHOLS_PI    Good noise and disturbance rejection
   1 ZIEGLER_NICHOLS_PID   Good noise and disturbance rejection
   2 TYREUS_LUYBEN_PI      Time-constant (lag) dominant processes (conservative)
   3 TYREUS_LUYBEN_PID     Time-constant (lag) dominant processes (conservative)
   4 CIANCONE_MARLIN_PI    Delay (dead-time) dominant processes
   5 CIANCONE_MARLIN_PID   Delay (dead-time) dominant processes
   6 AMIGO_PID             More universal than ZN_PID (uses a dead time dependancy)
   7 PESSEN_INTEGRAL_PID   Similar to ZN_PID but with better dynamic response
   8 SOME_OVERSHOOT_PID    ZN_PID with lower proportional and integral gain
   9 NO_OVERSHOOT_PID      ZN_PID with much lower P,I,D gain settings
 ******************************************************************************/

#include "QuickPID.h"

const byte inputPin = 0;
const byte outputPin = 3;

int Print = 0;         // on(1) monitor, off(0) plotter
int tuningRule = 1;    // see above table
float POn = 1.0;       // mix of PonE to PonM (0.0-1.0)
byte aTune = 0;        // autoTune status, done = 10

float Input, Output, Setpoint;
float Kp = 0, Ki = 0, Kd = 0;

QuickPID myQuickPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, POn, QuickPID::DIRECT);

void setup() {
  Serial.begin(115200);
  Serial.println();
}

void loop() {
  Input = avg(myQuickPID.analogReadFast(inputPin));
  if (aTune < 10) autoTune();
  if (aTune == 9) { // apply new tunings
    myQuickPID.SetTunings(Kp, Ki, Kd);
    myQuickPID.SetMode(QuickPID::AUTOMATIC);
    Setpoint = 700;
  }
  analogWrite(outputPin, Output);
  if (aTune == 10) { // compute
    if (Print == 0) { // serial plotter
      Serial.print("Setpoint:");  Serial.print(Setpoint);  Serial.print(",");
      Serial.print("Input:");     Serial.print(Input);     Serial.print(",");
      Serial.print("Output:");    Serial.print(Output);    Serial.print(",");
      Serial.println(",");
    }
    Input = avg(myQuickPID.analogReadFast(inputPin));
    myQuickPID.Compute();;
    analogWrite(outputPin, Output);
  }
}

void autoTune() {
  const int Rule[10][3] =
  { //ckp,  cki, ckd x 1000
    { 450,  540,   0 },  // ZIEGLER_NICHOLS_PI
    { 600, 176,  75 },  // ZIEGLER_NICHOLS_PID
    { 313,  142,   0 },  // TYREUS_LUYBEN_PI
    { 454,  206,  72 },  // TYREUS_LUYBEN_PID
    { 303, 1212,   0 },  // CIANCONE_MARLIN_PI
    { 303, 1333,  37 },  // CIANCONE_MARLIN_PID
    {   0,    0,   0 },  // AMIGOF_PID
    { 700, 1750, 105 },  // PESSEN_INTEGRAL_PID
    { 333,  667, 111 },  // SOME_OVERSHOOT_PID
    { 200,  400,  67 },  // NO_OVERSHOOT_PID
  };
  const byte ckp = 0, cki = 1, ckd = 2; //c = column

  const byte outputStep = 1;
  const byte hysteresis = 1;
  const int atSetpoint = 341;  // 1/3 of 10-bit ADC matches 8-bit PWM value of 85 for symetrical waveform
  const int atOutput = 85;

  static uint32_t t0, t1, t2, t3;
  static float Ku, Tu, td, kp, ki, kd, rdAvg, peakHigh, peakLow;

  switch (aTune) {
    case 0:
      peakHigh = atSetpoint;
      peakLow = atSetpoint;
      if (Print == 1) Serial.print("Stabilizing (33%) →");
      for (int i = 0; i < 16; i++) avg(Input); // initialize
      Output = 0;
      aTune++;
      break;
    case 1: // start coarse adjust
      if (avg(Input) < (atSetpoint - hysteresis)) {
        Output = atOutput + 20;
        aTune++;
      }
      break;
    case 2: // start fine adjust
      if (avg(Input) > atSetpoint) {
        Output = atOutput - outputStep;
        aTune++;
      }
      break;
    case 3: // run AutoTune
      if (avg(Input) < atSetpoint) {
        if (Print == 1) Serial.print(" Running AutoTune");
        Output = atOutput + outputStep;
        aTune++;
      }
      break;
    case 4: // get t0
      if (avg(Input) > atSetpoint) {
        t0 = micros();
        if (Print == 1) Serial.print(" ↑");
        aTune++;
      }
      break;
    case 5: // get t1
      if (avg(Input) > atSetpoint + 0.2) {
        t1 = micros();
        aTune++;
      }
      break;
    case 6: // get t2
      rdAvg = avg(Input);
      if (rdAvg > peakHigh) peakHigh = rdAvg;
      if ((rdAvg < peakLow) && (peakHigh >= (atSetpoint + hysteresis))) peakLow = rdAvg;

      if (rdAvg > atSetpoint + hysteresis) {
        t2 = micros();
        if (Print == 1) Serial.print(" ↓");
        Output = atOutput - outputStep;
        aTune++;
      }
      break;
    case 7: // begin t3
      rdAvg = avg(Input);
      if (rdAvg > peakHigh) peakHigh = rdAvg;
      if ((rdAvg < peakLow) && (peakHigh >= (atSetpoint + hysteresis))) peakLow = rdAvg;
      if (rdAvg < atSetpoint - hysteresis) {
        if (Print == 1) Serial.print(" ↑");
        Output = atOutput + outputStep;
        aTune++;
      }
      break;
    case 8: // get t3
      rdAvg = avg(Input);
      if (rdAvg > peakHigh) peakHigh = rdAvg;
      if ((rdAvg < peakLow) && (peakHigh >= (atSetpoint + hysteresis))) peakLow = rdAvg;
      if (rdAvg > atSetpoint + hysteresis) {
        t3 = micros();
        if (Print == 1) Serial.println(" Done.");
        aTune++;
        td = (float)(t1 - t0) / 1000000.0; // dead time (seconds)
        Tu = (float)(t3 - t2) / 1000000.0; // ultimate period (seconds)
        Ku = (float)(4 * outputStep * 2) / (float)(3.14159 * sqrt (sq (peakHigh - peakLow) - sq (hysteresis))); // ultimate gain
        if (tuningRule == 6) { //AMIGO_PID
          (td < 10) ? td = 10 : td = td; // 10µs minimum
          kp = (0.2 + 0.45 * (Tu / td)) / Ku;
          float Ti = (((0.4 * td) + (0.8 * Tu)) / (td + (0.1 * Tu)) * td);
          float Td = (0.5 * td * Tu) / ((0.3 * td) + Tu);
          ki = kp / Ti;
          kd = Td * kp;
        } else { //other rules
          kp = Rule[tuningRule][ckp] / 1000.0 * Ku;
          ki = Rule[tuningRule][cki] / 1000.0 * Ku / Tu;
          kd = Rule[tuningRule][ckd] / 1000.0 * Ku * Tu;
        }
        Kp = kp;
        Ki = ki;
        Kd = kd;
        if (Print == 1) {
          // Controllability https://blog.opticontrols.com/wp-content/uploads/2011/06/td-versus-tau.png
          if ((Tu / td + 0.0001) > 0.75) Serial.println("This process is easy to control.");
          else if ((Tu / td + 0.0001) > 0.25) Serial.println("This process has average controllability.");
          else Serial.println("This process is difficult to control.");
          Serial.print("Tu: "); Serial.print(Tu);    // Ultimate Period (sec)
          Serial.print("  td: "); Serial.print(td);  // Dead Time (sec)
          Serial.print("  Ku: "); Serial.print(Ku);  // Ultimate Gain
          Serial.print("  Kp: "); Serial.print(Kp);
          Serial.print("  Ki: "); Serial.print(Ki);
          Serial.print("  Kd: "); Serial.println(Kd);
          Serial.println();
        }
      }
      break;
    case 9: // ready to set tunings
      aTune++;
      break;
  }
}

float avg(int inputVal) {
  static int arrDat[16];
  static int pos;
  static long sum;
  pos++;
  if (pos >= 16) pos = 0;
  sum = sum - arrDat[pos] + inputVal;
  arrDat[pos] = inputVal;
  return (float)sum / 16.0;
}
