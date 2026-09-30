#ifndef TUNGLAM_OMNIMECANUM_4WD_H
#define TUNGLAM_OMNIMECANUM_4WD_H

#include <Arduino.h>
#include "TungLam_Control_MotorV5.h"

enum class TungLamPwmMode : uint8_t {
  Low976Hz = 0,
  High7k8Hz = 1
};

enum class TungLamChassis : uint8_t {
  MecanumX = 0,
  OmniX = 1
};

class TungLamDrive4WD {
 public:
  TungLamDrive4WD();

  // Hardware lifecycle
  void begin(TungLamPwmMode pwmMode = TungLamPwmMode::High7k8Hz);
  void update();

  // Chassis / wiring configuration
  void setChassis(TungLamChassis chassis);
  TungLamChassis chassis() const;
  void setMotorInverted(uint8_t wheel, bool inverted);
  void setDirectionDeadTimeUs(uint16_t deadTimeUs);

  // Low-level signed wheel output: -255..255
  void setWheels(int16_t m1, int16_t m2, int16_t m3, int16_t m4);

  // Holonomic command. vx: forward, vy: right, wz: clockwise.
  void drive(int16_t vx, int16_t vy, int16_t wz);
  void driveMecanum(int16_t vx, int16_t vy, int16_t wz);
  void driveOmniX(int16_t vx, int16_t vy, int16_t wz);

  // Convenience motion helpers
  void forward(uint8_t duty);
  void backward(uint8_t duty);
  void strafeRight(uint8_t duty);
  void strafeLeft(uint8_t duty);
  void rotateRight(uint8_t duty);
  void rotateLeft(uint8_t duty);

  // Stop / braking
  void stop();          // Coast: PWM disabled
  void coast();         // Alias of stop()
  void dynamicBrake();  // L298N bridge brake: EN high, both DIR inputs equal

  // Legacy-style active reverse braking, now non-blocking.
  // brakeDuty is fully user-controlled (0..255), exactly like the old ABS(duty).
  // Call update() repeatedly from loop() so the brake pulse can end on time.
  void ABS(uint8_t brakeDuty);
  void activeBrake(uint8_t brakeDuty); // Descriptive alias of ABS()
  bool isBraking() const;
  void cancelBrake();

  // Legacy timing API kept unchanged.
  void setTimABS(uint8_t t500,
                 uint8_t t1000,
                 uint8_t t1500,
                 uint8_t t2000,
                 uint8_t t3000,
                 uint8_t tAbove3000);

  // Newer descriptive alias; same timing table as setTimABS().
  void setBrakeTimings(uint8_t t500,
                       uint8_t t1000,
                       uint8_t t1500,
                       uint8_t t2000,
                       uint8_t t3000,
                       uint8_t tAbove3000);

 private:
  struct Wheels {
    int16_t m1;
    int16_t m2;
    int16_t m3;
    int16_t m4;
  };

  TungLamChassis chassis_;
  TungLamPwmMode pwmMode_;
  bool inverted_[4];
  uint16_t deadTimeUs_;

  Wheels commanded_;
  Wheels applied_;
  Wheels preBrake_;

  bool moving_;
  bool braking_;
  bool dynamicBraking_;
  uint32_t motionStartMs_;
  uint32_t brakeStartMs_;
  uint8_t brakeDurationMs_;

  uint8_t brakeT500_;
  uint8_t brakeT1000_;
  uint8_t brakeT1500_;
  uint8_t brakeT2000_;
  uint8_t brakeT3000_;
  uint8_t brakeTAbove3000_;

  void initPwm();
  void applyWheelsRaw(const Wheels& wheels);
  void writeDirectionPattern(const Wheels& wheels);
  void writePwm(const Wheels& wheels);
  void writeAllPwm(uint8_t duty);
  uint8_t selectBrakeDuration(uint32_t motionDurationMs) const;

  static int16_t clampWheel(int32_t value);
  static uint8_t magnitude(int16_t value);
  static int8_t signOf(int16_t value);
  static bool anyMoving(const Wheels& wheels);
  static bool directionChanged(const Wheels& a, const Wheels& b);
  static Wheels normalize(int32_t m1, int32_t m2, int32_t m3, int32_t m4);
};

#endif
