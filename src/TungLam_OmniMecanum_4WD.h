/*
  TungLam_OmniMecanum_4WD
  Nguyen Khac Tung Lam - Tung Lam Automation

  Arduino Mega 2560 / ATmega2560 four-wheel holonomic motor library.

  Supported drive layouts:
  - Mecanum-X 4WD
  - Omni X-drive 4WD
  - Raw independent four-wheel output

  Hardware mapping:
    PWM M1 -> D5  / PE3 / OC3A
    PWM M2 -> D6  / PH3 / OC4A
    PWM M3 -> D7  / PH4 / OC4B
    PWM M4 -> D8  / PH5 / OC4C

    DIR M1 -> D30 / D31
    DIR M2 -> D32 / D33
    DIR M3 -> D34 / D35
    DIR M4 -> D36 / D37

  Compatibility:
  - The original TungLam_Control_MotorV5 class and public API are preserved.
  - Existing projects may continue to include <TungLam_Control_MotorV5.h>.
  - Legacy ABS(duty) keeps user-controlled reverse-brake strength and setTimABS()
    timing, but is executed non-blocking by a Timer3 overflow one-shot.
*/

#ifndef TUNGLAM_OMNIMECANUM_4WD_H
#define TUNGLAM_OMNIMECANUM_4WD_H

#include <Arduino.h>

// ============================================================================
// Legacy V5 API
// ============================================================================
// This class intentionally keeps the original names/signatures so old robot
// projects can update the library without rewriting movement code.
class TungLam_Control_MotorV5 {
 public:
  TungLam_Control_MotorV5();

  // PWM/timer initialization.
  void Mode0();  // ~976.56 Hz on D5..D8
  void Mode1();  // ~7.8125 kHz on D5..D8
  void Init_Timer1(uint8_t duty11, uint8_t duty12);  // Legacy auxiliary PWM D11/D12
  void Init_Timer2(uint8_t duty9, uint8_t duty10);   // Legacy auxiliary PWM D9/D10

  // Basic movement API.
  void STOP();
  void moveForward(uint8_t duty);
  void moveBackward(uint8_t duty);
  void Forward_Right(uint8_t duty);
  void Backward_Right(uint8_t duty);
  void moveRight(uint8_t duty);
  void moveLeft(uint8_t duty);
  void moveLeftSide(uint8_t duty);
  void moveRightSide(uint8_t duty);
  void Forward_Left(uint8_t duty);
  void Backward_Left(uint8_t duty);

  /**
   * Active reverse braking, preserving the original V5 calling convention.
   *
   * @param duty Reverse-brake PWM selected directly by the user (0..255).
   *
   * The reverse pulse duration is selected from the six-stage table configured
   * by setTimABS(). Unlike the historical V5 implementation, this call returns
   * immediately. Timer3 overflow interrupt stops the pulse automatically, so
   * legacy sketches do not need an update() call.
   */
  void ABS(uint8_t duty);

  // Direct wheel direction helper retained for compatibility.
  void Dir(uint8_t BanhNumber, bool Set);

  // Independent per-wheel duty movement API retained exactly as V5.
  void Tien(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);
  void Lui(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);
  void Trai(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);
  void Phai(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);
  void T_Trai(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);
  void T_Phai(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);
  void L_Trai(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);
  void L_Phai(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);
  void N_Trai(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);
  void N_Phai(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);

  /**
   * Configure legacy active-reverse-brake duration table.
   *
   * @param t500       Brake time when previous motion duration is < 500 ms.
   * @param t1000      Brake time when previous motion duration is < 1000 ms.
   * @param t1500      Brake time when previous motion duration is < 1500 ms.
   * @param t2000      Brake time when previous motion duration is < 2000 ms.
   * @param t3000      Brake time when previous motion duration is < 3000 ms.
   * @param tAbove3000 Brake time when previous motion duration is >= 3000 ms.
   */
  void setTimABS(uint8_t t500,
                 uint8_t t1000,
                 uint8_t t1500,
                 uint8_t t2000,
                 uint8_t t3000,
                 uint8_t tAbove3000);

 private:
  void Reset_45(bool Off);
  void Reset_Timer(uint8_t timerNumber);
  void setSTOP(uint8_t pattern);
  void setPWM(uint8_t duty);
  void PWM(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);
  void Tim();
  void Timer();

  bool isMoving = false;
  unsigned long startTime = 0;
  uint8_t TIM = 0;
  uint8_t pre = 0;
  uint8_t tim500 = 45;
  uint8_t tim1000 = 65;
  uint8_t tim1500 = 70;
  uint8_t tim2000 = 75;
  uint8_t tim3000 = 80;
  uint8_t timAbove3000 = 85;
  uint8_t pwmMode = 1;  // 0: 976.56 Hz, 1: 7.8125 kHz
  bool Set = false;
};

// Transitional type alias kept at zero file-cost.
// The old 0.5.x transitional header itself is intentionally removed.
using TungLamMecanumL298N = TungLam_Control_MotorV5;

// ============================================================================
// Modern V6+ API
// ============================================================================

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

  // Hardware lifecycle.
  void begin(TungLamPwmMode pwmMode = TungLamPwmMode::High7k8Hz);
  void update();

  // Chassis / wiring configuration.
  void setChassis(TungLamChassis chassis);
  TungLamChassis chassis() const;
  void setMotorInverted(uint8_t wheel, bool inverted);
  void setDirectionDeadTimeUs(uint16_t deadTimeUs);

  // Low-level signed wheel output: -255..255.
  void setWheels(int16_t m1, int16_t m2, int16_t m3, int16_t m4);

  // Holonomic command convention:
  // vx > 0 forward, vy > 0 right, wz > 0 clockwise.
  void drive(int16_t vx, int16_t vy, int16_t wz);
  void driveMecanum(int16_t vx, int16_t vy, int16_t wz);
  void driveOmniX(int16_t vx, int16_t vy, int16_t wz);

  // Convenience motion helpers.
  void forward(uint8_t duty);
  void backward(uint8_t duty);
  void strafeRight(uint8_t duty);
  void strafeLeft(uint8_t duty);
  void rotateRight(uint8_t duty);
  void rotateLeft(uint8_t duty);

  // Stop / braking.
  void stop();          // Coast: PWM disabled.
  void coast();         // Alias of stop().
  void dynamicBrake();  // L298N bridge brake: EN active, DIR inputs equal.

  /**
   * Start user-controlled active reverse braking.
   *
   * @param brakeDuty Reverse-brake PWM selected by the user (0..255).
   *
   * Modern API braking is non-blocking and serviced by update().
   */
  void ABS(uint8_t brakeDuty);
  void activeBrake(uint8_t brakeDuty);
  bool isBraking() const;
  void cancelBrake();

  // Same six-stage timing concept as legacy setTimABS().
  void setTimABS(uint8_t t500,
                 uint8_t t1000,
                 uint8_t t1500,
                 uint8_t t2000,
                 uint8_t t3000,
                 uint8_t tAbove3000);

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

#endif  // TUNGLAM_OMNIMECANUM_4WD_H
