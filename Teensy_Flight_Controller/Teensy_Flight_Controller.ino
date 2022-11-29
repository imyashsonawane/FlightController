#include <Wire.h>
float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
int RateCalibrationNumber;
uint32_t LoopTimer;
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[] = {0, 0, 0};
float PRateRoll = 1.3; float PRatePitch = PRateRoll; float PRateYaw = 0;
float IRateRoll = 0.04; float IRatePitch = IRateRoll; float IRateYaw = 0;
float DRateRoll = 0.03; float DRatePitch = DRateRoll; float DRateYaw = 0;
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

#define RCPinRoll 15
#define RCPinPitch 16
#define RCPinThrottle 17
#define RCPinYaw 20

volatile long StartTimeRoll = 0;
volatile long CurrentTimeRoll = 0;
volatile long PulsesRoll = 0;
int PulseWidthRoll = 0;

volatile long StartTimePitch = 0;
volatile long CurrentTimePitch = 0;
volatile long PulsesPitch = 0;
int PulseWidthPitch = 0;

volatile long StartTimeThrottle = 0;
volatile long CurrentTimeThrottle = 0;
volatile long PulsesThrottle = 0;
int PulseWidthThrottle = 0;

volatile long StartTimeYaw = 0;
volatile long CurrentTimeYaw = 0;
volatile long PulsesYaw = 0;
int PulseWidthYaw = 0;

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
}
void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
  if (Iterm > 400) Iterm = 400;
  else if (Iterm < -400) Iterm = -400;
  float Dterm = D * (Error - PrevError) / 0.004;
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400) PIDOutput = 400;
  else if (PIDOutput < -400) PIDOutput = -400;
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void reset_pid(void) {
  PrevErrorRateRoll = 0; PrevErrorRatePitch = 0; PrevErrorRateYaw = 0;
  PrevItermRateRoll = 0; PrevItermRatePitch = 0; PrevItermRateYaw = 0;
}

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
  analogWriteFrequency(1, 250);
  analogWriteFrequency(2, 250);
  analogWriteFrequency(3, 250);
  analogWriteFrequency(4, 250);
  analogWriteResolution(12);
  delay(4);
  Serial.begin(9600);
  pinMode(RCPinRoll, INPUT_PULLUP);
  pinMode(RCPinPitch, INPUT_PULLUP);
  pinMode(RCPinThrottle, INPUT_PULLUP);
  pinMode(RCPinYaw, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RCPinRoll), PulseTimerRoll, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RCPinPitch), PulseTimerPitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RCPinThrottle), PulseTimerThrottle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RCPinYaw), PulseTimerYaw, CHANGE);
  digitalWrite(13, LOW);
  LoopTimer = micros();
}
void loop() {
  gyro_signals();
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;
  if (PulsesRoll < 2000) {
    PulseWidthRoll = PulsesRoll;
  }
  if (PulsesPitch < 2000) {
    PulseWidthPitch = PulsesPitch;
  }
  if (PulsesThrottle < 2000) {
    PulseWidthThrottle = PulsesThrottle;
  }
  if (PulsesYaw < 2000) {
    PulseWidthYaw = PulsesYaw;
  }
  //printRadioData();
  printMotorCommands();
  DesiredRateRoll = (PulseWidthRoll - 1500) * 0.15;
  DesiredRatePitch = (PulseWidthPitch - 1500) * 0.15;
  InputThrottle = PulseWidthThrottle;
  DesiredRateYaw = (PulseWidthYaw - 1500) * 0.15;
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;
  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll = PIDReturn[0];
  PrevErrorRateRoll = PIDReturn[1];
  PrevItermRateRoll = PIDReturn[2];
  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch = PIDReturn[0];
  PrevErrorRatePitch = PIDReturn[1];
  PrevItermRatePitch = PIDReturn[2];
  pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
  InputYaw = PIDReturn[0];
  PrevErrorRateYaw = PIDReturn[1];
  PrevItermRateYaw = PIDReturn[2];
  if (InputThrottle > 1800) InputThrottle = 1800;
  MotorInput1 = 1.024 * (InputThrottle - InputRoll - InputPitch - InputYaw);
  MotorInput2 = 1.024 * (InputThrottle - InputRoll + InputPitch + InputYaw);
  MotorInput3 = 1.024 * (InputThrottle + InputRoll + InputPitch - InputYaw);
  MotorInput4 = 1.024 * (InputThrottle + InputRoll - InputPitch + InputYaw);
  if (MotorInput1 > 2000)MotorInput1 = 1999;
  if (MotorInput2 > 2000)MotorInput2 = 1999;
  if (MotorInput3 > 2000)MotorInput3 = 1999;
  if (MotorInput4 > 2000)MotorInput4 = 1999;
  int ThrottleIdle = 1000;
  if (MotorInput1 < ThrottleIdle) MotorInput1 =  ThrottleIdle;
  if (MotorInput2 < ThrottleIdle) MotorInput2 =  ThrottleIdle;
  if (MotorInput3 < ThrottleIdle) MotorInput3 =  ThrottleIdle;
  if (MotorInput4 < ThrottleIdle) MotorInput4 =  ThrottleIdle;
  int ThrottleCutOff = 1150;
  if (PulseWidthThrottle < 1050) {
    MotorInput1 = ThrottleCutOff;
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff;
    MotorInput4 = ThrottleCutOff;
    reset_pid();
  }
  analogWrite(1, MotorInput1);
  analogWrite(2, MotorInput2);
  analogWrite(3, MotorInput3);
  analogWrite(4, MotorInput4);

  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}

void PulseTimerRoll() {
  CurrentTimeRoll = micros();
  if (CurrentTimeRoll > StartTimeRoll) {
    PulsesRoll = CurrentTimeRoll - StartTimeRoll;
    StartTimeRoll = CurrentTimeRoll;
  }
}

void PulseTimerPitch() {
  CurrentTimePitch = micros();
  if (CurrentTimePitch > StartTimePitch) {
    PulsesPitch = CurrentTimePitch - StartTimePitch;
    StartTimePitch = CurrentTimePitch;
  }
}

void PulseTimerThrottle() {
  CurrentTimeThrottle = micros();
  if (CurrentTimeThrottle > StartTimeThrottle) {
    PulsesThrottle = CurrentTimeThrottle - StartTimeThrottle;
    StartTimeThrottle = CurrentTimeThrottle;
  }
}

void PulseTimerYaw() {
  CurrentTimeYaw = micros();
  if (CurrentTimeYaw > StartTimeYaw) {
    PulsesYaw = CurrentTimeYaw - StartTimeYaw;
    StartTimeYaw = CurrentTimeYaw;
  }
}

void printMotorCommands() {
  Serial.print(F("Motor1: "));
  Serial.print(MotorInput1);
  Serial.print(F("Motor2: "));
  Serial.print(MotorInput2);
  Serial.print(F("Motor3: "));
  Serial.print(MotorInput3);
  Serial.print(F("Motor4: "));
  Serial.println(MotorInput4);
}

void printRadioData() {
  Serial.print(F("Roll: "));
  Serial.print(DesiredRateRoll);
  Serial.print(F(" Pitch: "));
  Serial.print(PulseWidthPitch);
  Serial.print(F(" Throttle: "));
  Serial.print(PulseWidthThrottle);
  Serial.print(F(" Yaw: "));
  Serial.println(PulseWidthYaw);
}
