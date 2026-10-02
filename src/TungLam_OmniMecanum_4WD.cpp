/*==============================================================================
  IMPLEMENTATION - TUNGLAM OMNI / MECANUM 4WD
  ==============================================================================
  File này chứa implementation cho cả Modern API và Legacy V5.

  Phần cứng được thư viện sử dụng:
  - PORTC D30..D37: chân chiều của 4 motor.
  - Timer3 OC3A: PWM M1 trên D5.
  - Timer4 OC4A/OC4B/OC4C: PWM M2/M3/M4 trên D6/D7/D8.
  - Timer3 overflow ISR: bộ định thời one-shot cho ABS.

  Trình tự an toàn khi đổi trạng thái chiều:
      PWM = 0 -> dead-time -> cập nhật DIR -> PWM mới
==============================================================================*/

#if !defined(__AVR_ATmega2560__) && !defined(__AVR_ATmega1280__)
#error "TungLam_OmniMecanum_4WD requires Arduino Mega / ATmega2560 or ATmega1280 timer and PORT layout."
#endif

#include "TungLam_OmniMecanum_4WD.h"
#include <avr/interrupt.h>

// ============================================================================
// COMMON MOTOR HAL + TIMER3 ABS SCHEDULER
// ============================================================================
// Cả Modern và Legacy cùng đi qua một tầng HAL để tránh lệch hành vi phần cứng.

namespace {

// Vector 4 bánh có dấu ở tầng HAL.
struct HalWheels {
  int16_t m1;
  int16_t m2;
  int16_t m3;
  int16_t m4;
};

// State vật lý gần nhất đã áp xuống H-bridge; ISR có thể reset về 0.
volatile int16_t gHalApplied[4] = {0, 0, 0, 0};

// State dùng chung của bộ hẹn giờ ABS one-shot.
volatile bool gTimedBrakeActive = false;
volatile uint16_t gTimedBrakeOverflowsRemaining = 0;

// Động học normalized Mecanum theo body frame tay phải:
// +vx tiến, +vy trái, +wz quay CCW.
constexpr int32_t mecanumM1(int32_t vx, int32_t vy, int32_t wz) {
  return vx - vy - wz;
}
constexpr int32_t mecanumM2(int32_t vx, int32_t vy, int32_t wz) {
  return vx + vy - wz;
}
constexpr int32_t mecanumM3(int32_t vx, int32_t vy, int32_t wz) {
  return vx + vy + wz;
}
constexpr int32_t mecanumM4(int32_t vx, int32_t vy, int32_t wz) {
  return vx - vy + wz;
}

static_assert(
    mecanumM1(1, 0, 0) == 1 && mecanumM2(1, 0, 0) == 1 &&
    mecanumM3(1, 0, 0) == 1 && mecanumM4(1, 0, 0) == 1,
    "Mecanum +vx forward basis must remain ++++");

static_assert(
    mecanumM1(0, 1, 0) == -1 && mecanumM2(0, 1, 0) == 1 &&
    mecanumM3(0, 1, 0) == 1 && mecanumM4(0, 1, 0) == -1,
    "Mecanum +vy left-strafe basis must remain -++-");

static_assert(
    mecanumM1(0, -1, 0) == 1 && mecanumM2(0, -1, 0) == -1 &&
    mecanumM3(0, -1, 0) == -1 && mecanumM4(0, -1, 0) == 1,
    "Mecanum -vy right-strafe basis must remain +--+");

static_assert(
    mecanumM1(0, 0, 1) == -1 && mecanumM2(0, 0, 1) == -1 &&
    mecanumM3(0, 0, 1) == 1 && mecanumM4(0, 0, 1) == 1,
    "Mecanum +wz CCW basis must remain --++");

static_assert(
    mecanumM1(1, -1, 0) == 2 && mecanumM2(1, -1, 0) == 0 &&
    mecanumM3(1, -1, 0) == 0 && mecanumM4(1, -1, 0) == 2,
    "Mecanum forward-right diagonal must remain +00+");

static_assert(
    mecanumM1(1, 1, 0) == 0 && mecanumM2(1, 1, 0) == 2 &&
    mecanumM3(1, 1, 0) == 2 && mecanumM4(1, 1, 0) == 0,
    "Mecanum forward-left diagonal must remain 0++0");

// Động học normalized Omni-X canonical 45 độ.
constexpr int32_t omniM1(int32_t vx, int32_t vy, int32_t wz) {
  return vx - vy - wz;
}
constexpr int32_t omniM2(int32_t vx, int32_t vy, int32_t wz) {
  return vx + vy - wz;
}
constexpr int32_t omniM3(int32_t vx, int32_t vy, int32_t wz) {
  return -vx - vy - wz;
}
constexpr int32_t omniM4(int32_t vx, int32_t vy, int32_t wz) {
  return -vx + vy - wz;
}

static_assert(
    omniM1(1, 0, 0) == 1 && omniM2(1, 0, 0) == 1 &&
    omniM3(1, 0, 0) == -1 && omniM4(1, 0, 0) == -1,
    "Omni-X +vx forward basis must remain ++--");

static_assert(
    omniM1(0, 1, 0) == -1 && omniM2(0, 1, 0) == 1 &&
    omniM3(0, 1, 0) == -1 && omniM4(0, 1, 0) == 1,
    "Omni-X +vy left basis must remain -+-+");

static_assert(
    omniM1(0, 0, 1) == -1 && omniM2(0, 0, 1) == -1 &&
    omniM3(0, 0, 1) == -1 && omniM4(0, 0, 1) == -1,
    "Omni-X +wz CCW basis must remain ----");

constexpr float kTwoPi = 6.2831853071795864769f;
constexpr float kInvSqrt2 = 0.7071067811865475244f;
constexpr float kSqrt2 = 1.4142135623730950488f;

// Các hàm tiện ích low-level cho dấu, PWM và pattern chân chiều.
inline int8_t halSign(int16_t value) {
  if (value > 0) return 1;
  if (value < 0) return -1;
  return 0;
}

inline uint8_t halMagnitude(int16_t value) {
  if (value < 0) value = -value;
  return value > 255 ? 255 : (uint8_t)value;
}

inline void halWriteAllPwm(uint8_t duty) {
  OCR3A = duty;
  OCR4A = duty;
  OCR4B = duty;
  OCR4C = duty;
}

inline void halWritePwm(const HalWheels& wheels) {
  OCR3A = halMagnitude(wheels.m1);
  OCR4A = halMagnitude(wheels.m2);
  OCR4B = halMagnitude(wheels.m3);
  OCR4C = halMagnitude(wheels.m4);
}

inline void halWriteDirectionPattern(const HalWheels& wheels) {
  uint8_t pattern = 0;

  if (wheels.m1 > 0) pattern |= (1 << PC7);
  else if (wheels.m1 < 0) pattern |= (1 << PC6);

  if (wheels.m2 > 0) pattern |= (1 << PC5);
  else if (wheels.m2 < 0) pattern |= (1 << PC4);

  if (wheels.m3 > 0) pattern |= (1 << PC3);
  else if (wheels.m3 < 0) pattern |= (1 << PC2);

  if (wheels.m4 > 0) pattern |= (1 << PC0);
  else if (wheels.m4 < 0) pattern |= (1 << PC1);

  PORTC = pattern;
}

inline void halRememberApplied(const HalWheels& wheels) {
  gHalApplied[0] = wheels.m1;
  gHalApplied[1] = wheels.m2;
  gHalApplied[2] = wheels.m3;
  gHalApplied[3] = wheels.m4;
}

inline HalWheels halCurrentApplied() {
  return {
      gHalApplied[0],
      gHalApplied[1],
      gHalApplied[2],
      gHalApplied[3]
  };
}

inline void halStopHardwareImmediate() {
  halWriteAllPwm(0);
  PORTC = 0x00;
  gHalApplied[0] = 0;
  gHalApplied[1] = 0;
  gHalApplied[2] = 0;
  gHalApplied[3] = 0;
}

// Hủy xung ABS đang hoạt động; có thể dừng output ngay nếu cần.
void cancelTimedBrake(bool stopOutputs) {
  const uint8_t oldSreg = SREG;
  cli();

  TIMSK3 &= ~(1 << TOIE3);
  gTimedBrakeOverflowsRemaining = 0;
  gTimedBrakeActive = false;

  if (stopOutputs) {
    halStopHardwareImmediate();
  }

  SREG = oldSreg;
}

// Áp vector motor qua trình tự đảo chiều an toàn của H-bridge.
void halApply(const HalWheels& target, uint16_t deadTimeUs) {

  if (gTimedBrakeActive) {
    cancelTimedBrake(true);
  }

  const HalWheels previous = halCurrentApplied();
  const int16_t oldValue[4] = {
      previous.m1, previous.m2, previous.m3, previous.m4
  };
  const int16_t newValue[4] = {
      target.m1, target.m2, target.m3, target.m4
  };

  bool electricalStateChanged = false;
  for (uint8_t i = 0; i < 4; ++i) {
    if (halSign(oldValue[i]) != halSign(newValue[i])) {
      electricalStateChanged = true;
      break;
    }
  }

  if (electricalStateChanged) {
    halWriteAllPwm(0);
    if (deadTimeUs > 0) {
      delayMicroseconds(deadTimeUs);
    }
  }

  halWriteDirectionPattern(target);
  halWritePwm(target);
  halRememberApplied(target);
}

void halSetLegacyDirection(uint8_t wheel, bool setValue, uint16_t deadTimeUs) {
  if (wheel < 1 || wheel > 4) {
    return;
  }

  if (gTimedBrakeActive) {
    cancelTimedBrake(true);
  }

  const uint8_t duty[4] = {
      (uint8_t)OCR3A, (uint8_t)OCR4A, (uint8_t)OCR4B, (uint8_t)OCR4C
  };

  halWriteAllPwm(0);
  if (deadTimeUs > 0) {
    delayMicroseconds(deadTimeUs);
  }

  const uint8_t forwardBit[4] = {PC7, PC5, PC3, PC0};
  const uint8_t reverseBit[4] = {PC6, PC4, PC2, PC1};
  const uint8_t idx = wheel - 1;

  if (!setValue) {
    PORTC |= (1 << forwardBit[idx]);
    PORTC &= ~(1 << reverseBit[idx]);
  } else {
    PORTC |= (1 << reverseBit[idx]);
    PORTC &= ~(1 << forwardBit[idx]);
  }

  OCR3A = duty[0];
  OCR4A = duty[1];
  OCR4B = duty[2];
  OCR4C = duty[3];

  HalWheels remembered = halCurrentApplied();
  int16_t* value[4] = {
      &remembered.m1, &remembered.m2, &remembered.m3, &remembered.m4
  };
  *value[idx] = (!setValue ? 1 : -1) * (int16_t)duty[idx];
  halRememberApplied(remembered);
}

uint16_t timedBrakeOverflowCount(uint8_t brakeMs, bool highPwmMode) {
  if (brakeMs == 0) {
    return 0;
  }

  if (highPwmMode) {
    return (uint16_t)(((uint32_t)brakeMs * 125UL + 15UL) / 16UL);
  }

  return (uint16_t)(((uint32_t)brakeMs * 125UL + 127UL) / 128UL);
}

void startTimedBrakeOneShot(uint8_t brakeMs, bool highPwmMode) {
  const uint16_t overflows = timedBrakeOverflowCount(brakeMs, highPwmMode);

  if (overflows == 0) {
    halStopHardwareImmediate();
    return;
  }

  const uint8_t oldSreg = SREG;
  cli();

  gTimedBrakeOverflowsRemaining = overflows;
  gTimedBrakeActive = true;

  TIFR3 = (1 << TOV3);
  TIMSK3 |= (1 << TOIE3);

  SREG = oldSreg;
}

inline bool timedBrakeIsActive() {
  return gTimedBrakeActive;
}

inline void legacyApplyWheels(int16_t m1, int16_t m2, int16_t m3, int16_t m4) {
  halApply({m1, m2, m3, m4}, 100);
}

}

// ISR Timer3: đếm overflow và tự cắt xung ABS đúng deadline.
ISR(TIMER3_OVF_vect) {
  if (!gTimedBrakeActive) {
    TIMSK3 &= ~(1 << TOIE3);
    return;
  }

  if (gTimedBrakeOverflowsRemaining > 0) {
    --gTimedBrakeOverflowsRemaining;
  }

  if (gTimedBrakeOverflowsRemaining == 0) {
    halStopHardwareImmediate();
    TIMSK3 &= ~(1 << TOIE3);
    gTimedBrakeActive = false;
  }
}

// ============================================================================
// MODERN API IMPLEMENTATION
// ============================================================================

// Khởi tạo toàn bộ state Modern về giá trị an toàn.
TungLamDrive4WD::TungLamDrive4WD()
    : chassis_(TungLamChassis::MecanumX),
      pwmMode_(TungLamPwmMode::High7k8Hz),
      driveConfig_(),
      driveConfigValid_(false),
      cachedLeverM_(0.0f),
      cachedMotorRpm_(0.0f),
      cachedMaxWheelMps_(0.0f),
      cachedPwmPerMps_(0.0f),
      cachedMaxBodyMps_(0.0f),
      cachedMaxYawRadps_(0.0f),
      requestedBodyVelocity_{0.0f, 0.0f, 0.0f},
      rampedBodyVelocity_{0.0f, 0.0f, 0.0f},
      appliedBodyVelocity_{0.0f, 0.0f, 0.0f},
      lastVelocityScale_(1.0f),
      velocityLimited_(false),
      commandTimeoutMs_(0),
      lastCommandMs_(0),
      commandTimedOut_(false),
      velocityRampEnabled_(false),
      velocityRampPrimed_(false),
      linearAccelMps2_(0.0f),
      yawAccelRadps2_(0.0f),
      lastVelocityUpdateMs_(0),
      inverted_{false, false, false, false},
      deadTimeUs_(100),
      commanded_{0, 0, 0, 0},
      applied_{0, 0, 0, 0},
      preBrake_{0, 0, 0, 0},
      moving_(false),
      braking_(false),
      dynamicBraking_(false),
      motionStartMs_(0),
      brakeStartMs_(0),
      brakeDurationMs_(0),
      brakeT500_(45),
      brakeT1000_(65),
      brakeT1500_(70),
      brakeT2000_(75),
      brakeT3000_(80),
      brakeTAbove3000_(85) {}

// Khởi tạo GPIO, timer PWM và trạng thái dừng.
void TungLamDrive4WD::begin(TungLamPwmMode pwmMode) {
  cancelTimedBrake(true);

  pwmMode_ = pwmMode;

  DDRC = 0xFF;
  PORTC = 0x00;

  DDRE |= (1 << PE3);
  DDRH |= (1 << PH3) | (1 << PH4) | (1 << PH5);

  initPwm();

  lastCommandMs_ = millis();
  commandTimedOut_ = false;
  resetVelocityRampState();
  stop();
}

void TungLamDrive4WD::initPwm() {
  TCCR3A = 0;
  TCCR3B = 0;
  TIMSK3 = 0;

  TCCR4A = 0;
  TCCR4B = 0;
  TIMSK4 = 0;

  if (pwmMode_ == TungLamPwmMode::High7k8Hz) {

    TCCR3A = (1 << WGM30) | (1 << COM3A1);
    TCCR3B = (1 << WGM32) | (1 << CS31);

    TCCR4A = (1 << WGM40) | (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1);
    TCCR4B = (1 << WGM42) | (1 << CS41);
  } else {

    TCCR3A = (1 << WGM31) | (1 << COM3A1);
    TCCR3B = (1 << WGM32) | (1 << WGM33) | (1 << CS31) | (1 << CS30);
    ICR3 = 255;

    TCCR4A = (1 << WGM41) | (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1);
    TCCR4B = (1 << WGM42) | (1 << WGM43) | (1 << CS41) | (1 << CS40);
    ICR4 = 255;
  }

  halWriteAllPwm(0);
}

// Đồng bộ state phần mềm sau khi ISR đã kết thúc ABS ở tầng phần cứng.
void TungLamDrive4WD::update() {
  if (braking_ && !timedBrakeIsActive()) {
    braking_ = false;
    moving_ = false;
    motionStartMs_ = 0;
    brakeStartMs_ = 0;
    brakeDurationMs_ = 0;
    commanded_ = {0, 0, 0, 0};
    applied_ = {0, 0, 0, 0};
    appliedBodyVelocity_ = {0.0f, 0.0f, 0.0f};
    resetVelocityRampState();
  }

  if (commandTimeoutMs_ > 0 &&
      moving_ &&
      !braking_ &&
      !dynamicBraking_) {
    const uint32_t now = millis();
    if ((uint32_t)(now - lastCommandMs_) >= commandTimeoutMs_) {
      commanded_ = {0, 0, 0, 0};
      moving_ = false;
      motionStartMs_ = 0;
      applyWheelsRaw(commanded_);
      appliedBodyVelocity_ = {0.0f, 0.0f, 0.0f};
      resetVelocityRampState();
      commandTimedOut_ = true;
    }
  }
}

void TungLamDrive4WD::setChassis(TungLamChassis chassis) {
  chassis_ = chassis;
  rebuildChassisLimits();
  resetVelocityRampState();
}

TungLamChassis TungLamDrive4WD::chassis() const {
  return chassis_;
}

void TungLamDrive4WD::setMotorInverted(uint8_t wheel, bool inverted) {
  if (wheel < 1 || wheel > 4) {
    return;
  }
  inverted_[wheel - 1] = inverted;
}

void TungLamDrive4WD::setDirectionDeadTimeUs(uint16_t deadTimeUs) {
  deadTimeUs_ = deadTimeUs;
}

// Điểm trung tâm nhận lệnh bánh có dấu và cập nhật state chuyển động.
void TungLamDrive4WD::setWheels(int16_t m1, int16_t m2, int16_t m3, int16_t m4) {

  update();
  noteCommandReceived();

  Wheels next = {
      clampWheel(m1),
      clampWheel(m2),
      clampWheel(m3),
      clampWheel(m4)
  };

  if (braking_ || timedBrakeIsActive()) {
    cancelTimedBrake(true);
    braking_ = false;
    moving_ = false;
    motionStartMs_ = 0;
    brakeStartMs_ = 0;
    brakeDurationMs_ = 0;
    commanded_ = {0, 0, 0, 0};
    applied_ = {0, 0, 0, 0};
  }

  if (dynamicBraking_) {

    halWriteAllPwm(0);
    if (deadTimeUs_ > 0) {
      delayMicroseconds(deadTimeUs_);
    }
    PORTC = 0x00;
    halRememberApplied({0, 0, 0, 0});
    applied_ = {0, 0, 0, 0};
    dynamicBraking_ = false;
  }

  const bool wasMoving = anyMoving(commanded_);
  const bool willMove = anyMoving(next);
  const bool changedDirection = directionChanged(commanded_, next);

  if (willMove && (!wasMoving || changedDirection)) {
    motionStartMs_ = millis();
  }

  commanded_ = next;
  moving_ = willMove;
  applyWheelsRaw(next);

  if (!willMove) {
    motionStartMs_ = 0;
  }
}

// Chuyển lệnh body normalized tới mixer Mecanum hoặc Omni đang chọn.
void TungLamDrive4WD::drive(int16_t vx, int16_t vy, int16_t wz) {
  if (chassis_ == TungLamChassis::OmniX) {
    driveOmniX(vx, vy, wz);
  } else {
    driveMecanum(vx, vy, wz);
  }
}

// Lưu model vật lý nếu mọi tham số bắt buộc đều hợp lệ.
bool TungLamDrive4WD::setDriveConfig(const TungLamDriveConfig& config) {
  if (!driveConfigIsValid(config)) {
    return false;
  }

  driveConfig_ = config;
  driveConfigValid_ = true;
  rebuildDerivedModel();
  resetVelocityRampState();
  return true;
}

const TungLamDriveConfig& TungLamDrive4WD::driveConfig() const {
  return driveConfig_;
}

bool TungLamDrive4WD::hasDriveConfig() const {
  return driveConfigValid_;
}

// Ước lượng RPM từ RPM danh định, tỷ lệ điện áp và speedScale.
float TungLamDrive4WD::estimatedMotorRpmAtSupply() const {
  return driveConfigValid_ ? cachedMotorRpm_ : 0.0f;
}

float TungLamDrive4WD::maxWheelLinearSpeedMps() const {
  return driveConfigValid_ ? cachedMaxWheelMps_ : 0.0f;
}

float TungLamDrive4WD::maxBodyLinearSpeedMps() const {
  return driveConfigValid_ ? cachedMaxBodyMps_ : 0.0f;
}

float TungLamDrive4WD::maxYawRateRadps() const {
  return driveConfigValid_ ? cachedMaxYawRadps_ : 0.0f;
}

TungLamWheelVelocity TungLamDrive4WD::inverseKinematics(
    float vxMps,
    float vyMps,
    float wzRadps) const {
  if (!driveConfigValid_) {
    return {0.0f, 0.0f, 0.0f, 0.0f};
  }

  const float lever = cachedLeverM_;

  if (chassis_ == TungLamChassis::OmniX) {

    return {
        kInvSqrt2 * ( vxMps - vyMps - lever * wzRadps),
        kInvSqrt2 * ( vxMps + vyMps - lever * wzRadps),
        kInvSqrt2 * (-vxMps - vyMps - lever * wzRadps),
        kInvSqrt2 * (-vxMps + vyMps - lever * wzRadps)
    };
  }

  return {
      vxMps - vyMps - lever * wzRadps,
      vxMps + vyMps - lever * wzRadps,
      vxMps + vyMps + lever * wzRadps,
      vxMps - vyMps + lever * wzRadps
  };
}

// Động học thuận: vận tốc 4 bánh -> body velocity SI.
TungLamBodyVelocity TungLamDrive4WD::forwardKinematics(
    const TungLamWheelVelocity& wheels) const {
  if (!driveConfigValid_) {
    return {0.0f, 0.0f, 0.0f};
  }

  const float lever = cachedLeverM_;

  if (lever <= 0.0f) {
    return {0.0f, 0.0f, 0.0f};
  }

  if (chassis_ == TungLamChassis::OmniX) {
    return {
        kSqrt2 * ( wheels.m1Mps + wheels.m2Mps
                 - wheels.m3Mps - wheels.m4Mps) * 0.25f,
        kSqrt2 * (-wheels.m1Mps + wheels.m2Mps
                 - wheels.m3Mps + wheels.m4Mps) * 0.25f,
       -kSqrt2 * ( wheels.m1Mps + wheels.m2Mps
                 + wheels.m3Mps + wheels.m4Mps) / (4.0f * lever)
    };
  }

  return {
      ( wheels.m1Mps + wheels.m2Mps
      + wheels.m3Mps + wheels.m4Mps) * 0.25f,
      (-wheels.m1Mps + wheels.m2Mps
      + wheels.m3Mps - wheels.m4Mps) * 0.25f,
      (-wheels.m1Mps - wheels.m2Mps
      + wheels.m3Mps + wheels.m4Mps) / (4.0f * lever)
  };
}

// SI velocity -> IK -> giới hạn vector -> feed-forward PWM.
bool TungLamDrive4WD::driveVelocity(
    float vxMps,
    float vyMps,
    float wzRadps) {
  if (!driveConfigValid_) {
    stop();
    return false;
  }

  const uint32_t now = millis();
  requestedBodyVelocity_ = {vxMps, vyMps, wzRadps};

  const TungLamBodyVelocity shaped =
      applyVelocityRamp(requestedBodyVelocity_, now);
  rampedBodyVelocity_ = shaped;

  const TungLamWheelVelocity requested =
      inverseKinematics(shaped.vxMps, shaped.vyMps, shaped.wzRadps);

  float scale = 1.0f;
  const TungLamWheelVelocity limited =
      limitWheelVelocities(requested, &scale);

  lastVelocityScale_ = scale;
  velocityLimited_ = scale < 0.9999f;

  appliedBodyVelocity_ = {
      shaped.vxMps * scale,
      shaped.vyMps * scale,
      shaped.wzRadps * scale
  };

  setWheels(
      wheelVelocityToPwm(limited.m1Mps),
      wheelVelocityToPwm(limited.m2Mps),
      wheelVelocityToPwm(limited.m3Mps),
      wheelVelocityToPwm(limited.m4Mps));

  return true;
}

void TungLamDrive4WD::enableSmartSafety(
    uint16_t timeoutMs,
    float linearAccelMps2,
    float yawAccelRadps2) {
  setCommandTimeoutMs(timeoutMs);

  if (linearAccelMps2 > 0.0f && yawAccelRadps2 > 0.0f) {
    setVelocityRamp(linearAccelMps2, yawAccelRadps2);
  } else {
    disableVelocityRamp();
  }
}

void TungLamDrive4WD::disableSmartSafety() {
  setCommandTimeoutMs(0);
  disableVelocityRamp();
}

void TungLamDrive4WD::setCommandTimeoutMs(uint16_t timeoutMs) {
  commandTimeoutMs_ = timeoutMs;
  commandTimedOut_ = false;
  lastCommandMs_ = millis();
}

bool TungLamDrive4WD::commandTimedOut() const {
  return commandTimedOut_;
}

bool TungLamDrive4WD::setVelocityRamp(
    float linearAccelMps2,
    float yawAccelRadps2) {
  if (linearAccelMps2 <= 0.0f || yawAccelRadps2 <= 0.0f) {
    disableVelocityRamp();
    return false;
  }

  linearAccelMps2_ = linearAccelMps2;
  yawAccelRadps2_ = yawAccelRadps2;
  velocityRampEnabled_ = true;
  resetVelocityRampState();
  return true;
}

void TungLamDrive4WD::disableVelocityRamp() {
  velocityRampEnabled_ = false;
  linearAccelMps2_ = 0.0f;
  yawAccelRadps2_ = 0.0f;
  resetVelocityRampState();
}

bool TungLamDrive4WD::wasVelocityLimited() const {
  return velocityLimited_;
}

float TungLamDrive4WD::lastVelocityScale() const {
  return lastVelocityScale_;
}

TungLamBodyVelocity TungLamDrive4WD::requestedBodyVelocity() const {
  return requestedBodyVelocity_;
}

TungLamBodyVelocity TungLamDrive4WD::appliedBodyVelocity() const {
  return appliedBodyVelocity_;
}

void TungLamDrive4WD::driveMecanum(int16_t vx, int16_t vy, int16_t wz) {

  const Wheels out = normalize(
      mecanumM1(vx, vy, wz),
      mecanumM2(vx, vy, wz),
      mecanumM3(vx, vy, wz),
      mecanumM4(vx, vy, wz));

  setWheels(out.m1, out.m2, out.m3, out.m4);
}

// Mixer normalized Omni X-drive canonical.
void TungLamDrive4WD::driveOmniX(int16_t vx, int16_t vy, int16_t wz) {

  const Wheels out = normalize(
      omniM1(vx, vy, wz),
      omniM2(vx, vy, wz),
      omniM3(vx, vy, wz),
      omniM4(vx, vy, wz));

  setWheels(out.m1, out.m2, out.m3, out.m4);
}

void TungLamDrive4WD::forward(uint8_t duty) {
  drive((int16_t)duty, 0, 0);
}

void TungLamDrive4WD::backward(uint8_t duty) {
  drive(-(int16_t)duty, 0, 0);
}

void TungLamDrive4WD::strafeRight(uint8_t duty) {
  drive(0, -(int16_t)duty, 0);
}

void TungLamDrive4WD::strafeLeft(uint8_t duty) {
  drive(0, (int16_t)duty, 0);
}

void TungLamDrive4WD::rotateRight(uint8_t duty) {
  drive(0, 0, -(int16_t)duty);
}

void TungLamDrive4WD::rotateLeft(uint8_t duty) {
  drive(0, 0, (int16_t)duty);
}

void TungLamDrive4WD::stop() {
  noteCommandReceived();

  braking_ = false;
  dynamicBraking_ = false;
  moving_ = false;
  motionStartMs_ = 0;
  commanded_ = {0, 0, 0, 0};

  requestedBodyVelocity_ = {0.0f, 0.0f, 0.0f};
  rampedBodyVelocity_ = {0.0f, 0.0f, 0.0f};
  appliedBodyVelocity_ = {0.0f, 0.0f, 0.0f};
  lastVelocityScale_ = 1.0f;
  velocityLimited_ = false;
  resetVelocityRampState();

  applyWheelsRaw(commanded_);
}

void TungLamDrive4WD::coast() {
  stop();
}

// Hãm điện bằng trạng thái bridge của L298N, khác với ABS hãm ngược.
void TungLamDrive4WD::dynamicBrake() {
  noteCommandReceived();
  cancelTimedBrake(true);

  braking_ = false;
  dynamicBraking_ = true;
  moving_ = false;
  motionStartMs_ = 0;
  brakeStartMs_ = 0;
  brakeDurationMs_ = 0;
  commanded_ = {0, 0, 0, 0};

  requestedBodyVelocity_ = {0.0f, 0.0f, 0.0f};
  appliedBodyVelocity_ = {0.0f, 0.0f, 0.0f};
  lastVelocityScale_ = 1.0f;
  velocityLimited_ = false;
  resetVelocityRampState();

  PORTC = 0x00;
  halWriteAllPwm(255);
  halRememberApplied({0, 0, 0, 0});
  applied_ = {0, 0, 0, 0};
}

void TungLamDrive4WD::ABS(uint8_t brakeDuty) {

  update();
  noteCommandReceived();

  if (braking_ && timedBrakeIsActive()) {
    return;
  }

  if (timedBrakeIsActive()) {
    return;
  }

  if (dynamicBraking_) {
    stop();
    return;
  }

  if (!anyMoving(commanded_)) {
    stop();
    return;
  }

  preBrake_ = commanded_;

  const uint32_t motionDuration =
      moving_ ? (uint32_t)(millis() - motionStartMs_) : 0;
  brakeDurationMs_ = selectBrakeDuration(motionDuration);

  Wheels reverse = {0, 0, 0, 0};
  const int16_t previous[4] = {
      preBrake_.m1, preBrake_.m2, preBrake_.m3, preBrake_.m4
  };
  int16_t* target[4] = {
      &reverse.m1, &reverse.m2, &reverse.m3, &reverse.m4
  };

  for (uint8_t i = 0; i < 4; ++i) {
    if (previous[i] > 0) {
      *target[i] = -(int16_t)brakeDuty;
    } else if (previous[i] < 0) {
      *target[i] = (int16_t)brakeDuty;
    }
  }

  braking_ = true;
  moving_ = false;
  commanded_ = {0, 0, 0, 0};
  motionStartMs_ = 0;
  brakeStartMs_ = millis();
  appliedBodyVelocity_ = {0.0f, 0.0f, 0.0f};
  resetVelocityRampState();

  applyWheelsRaw(reverse);

  startTimedBrakeOneShot(
      brakeDurationMs_,
      pwmMode_ == TungLamPwmMode::High7k8Hz);

  update();
}

void TungLamDrive4WD::activeBrake(uint8_t brakeDuty) {
  ABS(brakeDuty);
}

bool TungLamDrive4WD::isBraking() const {
  return braking_ && timedBrakeIsActive();
}

void TungLamDrive4WD::cancelBrake() {
  noteCommandReceived();

  if (braking_ || dynamicBraking_ || timedBrakeIsActive()) {
    cancelTimedBrake(true);
    braking_ = false;
    dynamicBraking_ = false;
    moving_ = false;
    motionStartMs_ = 0;
    brakeStartMs_ = 0;
    brakeDurationMs_ = 0;
    commanded_ = {0, 0, 0, 0};
    applied_ = {0, 0, 0, 0};
    appliedBodyVelocity_ = {0.0f, 0.0f, 0.0f};
    resetVelocityRampState();
  }
}

void TungLamDrive4WD::setTimABS(uint8_t t500,
                                    uint8_t t1000,
                                    uint8_t t1500,
                                    uint8_t t2000,
                                    uint8_t t3000,
                                    uint8_t tAbove3000) {
  brakeT500_ = t500;
  brakeT1000_ = t1000;
  brakeT1500_ = t1500;
  brakeT2000_ = t2000;
  brakeT3000_ = t3000;
  brakeTAbove3000_ = tAbove3000;
}

void TungLamDrive4WD::setBrakeTimings(uint8_t t500,
                                      uint8_t t1000,
                                      uint8_t t1500,
                                      uint8_t t2000,
                                      uint8_t t3000,
                                      uint8_t tAbove3000) {
  setTimABS(t500, t1000, t1500, t2000, t3000, tAbove3000);
}

// Áp motor inversion rồi chuyển xuống common HAL.
void TungLamDrive4WD::applyWheelsRaw(const Wheels& wheels) {
  Wheels physical = wheels;

  int16_t* values[4] = {
      &physical.m1, &physical.m2, &physical.m3, &physical.m4
  };
  for (uint8_t i = 0; i < 4; ++i) {
    if (inverted_[i]) {
      *values[i] = -*values[i];
    }
  }

  halApply(
      {physical.m1, physical.m2, physical.m3, physical.m4},
      deadTimeUs_);

  applied_ = physical;
}

uint8_t TungLamDrive4WD::selectBrakeDuration(uint32_t duration) const {
  if (duration < 500UL) return brakeT500_;
  if (duration < 1000UL) return brakeT1000_;
  if (duration < 1500UL) return brakeT1500_;
  if (duration < 2000UL) return brakeT2000_;
  if (duration < 3000UL) return brakeT3000_;
  return brakeTAbove3000_;
}

int16_t TungLamDrive4WD::clampWheel(int32_t value) {
  if (value > 255) return 255;
  if (value < -255) return -255;
  return (int16_t)value;
}

int8_t TungLamDrive4WD::signOf(int16_t value) {
  if (value > 0) return 1;
  if (value < 0) return -1;
  return 0;
}

bool TungLamDrive4WD::anyMoving(const Wheels& wheels) {
  return wheels.m1 != 0 || wheels.m2 != 0 || wheels.m3 != 0 || wheels.m4 != 0;
}

bool TungLamDrive4WD::directionChanged(const Wheels& a, const Wheels& b) {
  const int16_t av[4] = {a.m1, a.m2, a.m3, a.m4};
  const int16_t bv[4] = {b.m1, b.m2, b.m3, b.m4};

  for (uint8_t i = 0; i < 4; ++i) {
    const int8_t sa = signOf(av[i]);
    const int8_t sb = signOf(bv[i]);
    if (sa != 0 && sb != 0 && sa != sb) {
      return true;
    }
  }
  return false;
}

// Kiểm tra model vật lý: các đại lượng bắt buộc phải dương.
bool TungLamDrive4WD::driveConfigIsValid(const TungLamDriveConfig& config) {
  return config.motorNominalVoltageV > 0.0f &&
         config.motorNoLoadRpm > 0.0f &&
         config.supplyVoltageV > 0.0f &&
         config.wheelRadiusM > 0.0f &&
         config.wheelbaseM > 0.0f &&
         config.trackWidthM > 0.0f &&
         config.speedScale > 0.0f;
}

// Quy đổi m/s của bánh sang PWM feed-forward có dấu.
void TungLamDrive4WD::rebuildDerivedModel() {
  if (!driveConfigValid_) {
    cachedLeverM_ = 0.0f;
    cachedMotorRpm_ = 0.0f;
    cachedMaxWheelMps_ = 0.0f;
    cachedPwmPerMps_ = 0.0f;
    cachedMaxBodyMps_ = 0.0f;
    cachedMaxYawRadps_ = 0.0f;
    return;
  }

  cachedLeverM_ =
      0.5f * (driveConfig_.wheelbaseM + driveConfig_.trackWidthM);

  cachedMotorRpm_ =
      driveConfig_.motorNoLoadRpm *
      (driveConfig_.supplyVoltageV / driveConfig_.motorNominalVoltageV) *
      driveConfig_.speedScale;

  cachedMaxWheelMps_ =
      (cachedMotorRpm_ / 60.0f) *
      kTwoPi *
      driveConfig_.wheelRadiusM;

  cachedPwmPerMps_ =
      cachedMaxWheelMps_ > 0.0f ? (255.0f / cachedMaxWheelMps_) : 0.0f;

  rebuildChassisLimits();
}

void TungLamDrive4WD::rebuildChassisLimits() {
  if (!driveConfigValid_ ||
      cachedMaxWheelMps_ <= 0.0f ||
      cachedLeverM_ <= 0.0f) {
    cachedMaxBodyMps_ = 0.0f;
    cachedMaxYawRadps_ = 0.0f;
    return;
  }

  if (chassis_ == TungLamChassis::OmniX) {
    cachedMaxBodyMps_ = cachedMaxWheelMps_ * kSqrt2;
    cachedMaxYawRadps_ =
        cachedMaxWheelMps_ / (kInvSqrt2 * cachedLeverM_);
  } else {
    cachedMaxBodyMps_ = cachedMaxWheelMps_;
    cachedMaxYawRadps_ = cachedMaxWheelMps_ / cachedLeverM_;
  }
}

void TungLamDrive4WD::noteCommandReceived() {
  lastCommandMs_ = millis();
  commandTimedOut_ = false;
}

void TungLamDrive4WD::resetVelocityRampState() {
  velocityRampPrimed_ = false;
  lastVelocityUpdateMs_ = 0;
  rampedBodyVelocity_ = {0.0f, 0.0f, 0.0f};
}

TungLamBodyVelocity TungLamDrive4WD::applyVelocityRamp(
    const TungLamBodyVelocity& target,
    uint32_t nowMs) {
  if (!velocityRampEnabled_) {
    return target;
  }

  if (!velocityRampPrimed_) {
    velocityRampPrimed_ = true;
    lastVelocityUpdateMs_ = nowMs;
    rampedBodyVelocity_ = {0.0f, 0.0f, 0.0f};
    return rampedBodyVelocity_;
  }

  uint32_t dtMs = (uint32_t)(nowMs - lastVelocityUpdateMs_);
  lastVelocityUpdateMs_ = nowMs;

  if (dtMs > 100UL) {
    dtMs = 100UL;
  }

  const float dt = (float)dtMs * 0.001f;
  const float linearDelta = linearAccelMps2_ * dt;
  const float yawDelta = yawAccelRadps2_ * dt;

  rampedBodyVelocity_.vxMps =
      approachFloat(rampedBodyVelocity_.vxMps, target.vxMps, linearDelta);
  rampedBodyVelocity_.vyMps =
      approachFloat(rampedBodyVelocity_.vyMps, target.vyMps, linearDelta);
  rampedBodyVelocity_.wzRadps =
      approachFloat(rampedBodyVelocity_.wzRadps, target.wzRadps, yawDelta);

  return rampedBodyVelocity_;
}

float TungLamDrive4WD::approachFloat(
    float current,
    float target,
    float maxDelta) {
  if (maxDelta <= 0.0f) {
    return target;
  }

  const float delta = target - current;
  if (delta > maxDelta) {
    return current + maxDelta;
  }
  if (delta < -maxDelta) {
    return current - maxDelta;
  }
  return target;
}

int16_t TungLamDrive4WD::wheelVelocityToPwm(float wheelMps) const {
  if (cachedPwmPerMps_ <= 0.0f || wheelMps == 0.0f) {
    return 0;
  }

  float pwm = wheelMps * cachedPwmPerMps_;
  if (pwm > 255.0f) pwm = 255.0f;
  if (pwm < -255.0f) pwm = -255.0f;

  return (int16_t)(pwm >= 0.0f ? pwm + 0.5f : pwm - 0.5f);
}

TungLamWheelVelocity TungLamDrive4WD::limitWheelVelocities(
    const TungLamWheelVelocity& wheels,
    float* scaleOut) const {
  if (scaleOut != nullptr) {
    *scaleOut = 1.0f;
  }

  const float maxAvailable = cachedMaxWheelMps_;
  if (maxAvailable <= 0.0f) {
    if (scaleOut != nullptr) {
      *scaleOut = 0.0f;
    }
    return {0.0f, 0.0f, 0.0f, 0.0f};
  }

  float maxRequested = wheels.m1Mps < 0.0f ? -wheels.m1Mps : wheels.m1Mps;
  const float a2 = wheels.m2Mps < 0.0f ? -wheels.m2Mps : wheels.m2Mps;
  const float a3 = wheels.m3Mps < 0.0f ? -wheels.m3Mps : wheels.m3Mps;
  const float a4 = wheels.m4Mps < 0.0f ? -wheels.m4Mps : wheels.m4Mps;

  if (a2 > maxRequested) maxRequested = a2;
  if (a3 > maxRequested) maxRequested = a3;
  if (a4 > maxRequested) maxRequested = a4;

  if (maxRequested <= maxAvailable || maxRequested <= 0.0f) {
    return wheels;
  }

  const float scale = maxAvailable / maxRequested;
  if (scaleOut != nullptr) {
    *scaleOut = scale;
  }

  return {
      wheels.m1Mps * scale,
      wheels.m2Mps * scale,
      wheels.m3Mps * scale,
      wheels.m4Mps * scale
  };
}

TungLamDrive4WD::Wheels TungLamDrive4WD::normalize(int32_t m1,
                                                   int32_t m2,
                                                   int32_t m3,
                                                   int32_t m4) {
  int32_t maxMagnitude = m1 < 0 ? -m1 : m1;
  const int32_t a2 = m2 < 0 ? -m2 : m2;
  const int32_t a3 = m3 < 0 ? -m3 : m3;
  const int32_t a4 = m4 < 0 ? -m4 : m4;

  if (a2 > maxMagnitude) maxMagnitude = a2;
  if (a3 > maxMagnitude) maxMagnitude = a3;
  if (a4 > maxMagnitude) maxMagnitude = a4;

  if (maxMagnitude > 255) {
    m1 = (m1 * 255L) / maxMagnitude;
    m2 = (m2 * 255L) / maxMagnitude;
    m3 = (m3 * 255L) / maxMagnitude;
    m4 = (m4 * 255L) / maxMagnitude;
  }

  return {
      clampWheel(m1),
      clampWheel(m2),
      clampWheel(m3),
      clampWheel(m4)
  };
}

// ============================================================================
// LEGACY V5 IMPLEMENTATION
// ============================================================================
// Giữ nguyên tên hàm và vector chuyển động V5 để project cũ tiếp tục hoạt động.

TungLam_Control_MotorV5::TungLam_Control_MotorV5() {

}

void TungLam_Control_MotorV5::Dir(uint8_t BanhNumber, bool Set)
{

  halSetLegacyDirection(BanhNumber, Set, 100);
}

void TungLam_Control_MotorV5::Reset_Timer(uint8_t timerNumber)
{
    switch (timerNumber)
    {
    case 1:

        TCCR1A = 0;
        TCCR1B = 0;
        TIMSK1 = 0;
        break;

    case 2:

        TCCR2A = 0;
        TCCR2B = 0;
        TIMSK2 = 0;
        break;

    case 3:

        TCCR3A = 0;
        TCCR3B = 0;
        TIMSK3 = 0;
        break;

    case 4:

        TCCR4A = 0;
        TCCR4B = 0;
        TIMSK4 = 0;
        break;

    default:

        break;
    }
}

// Mode1: PWM drive khoảng 7,8125 kHz.
void TungLam_Control_MotorV5::Mode1()
 {
    cancelTimedBrake(true);
    pwmMode = 1;

    DDRC |= 0xFF;

    DDRH |= (1 << PH3) | (1 << PH4) | (1 << PH5);
    DDRE |= (1 << PE3);

      Reset_Timer(3);
      Reset_Timer(4);

    TCCR3A = (1 << WGM30) | (1 << COM3A1);
    TCCR3B = (1 << WGM32) | (1 << CS31);

    TCCR4A = (1 << WGM40) | (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1);
    TCCR4B = (1 << WGM42) | (1 << CS41);

    OCR3A = 0;
    OCR4A = 0;
    OCR4B = 0;
    OCR4C = 0;
}

// Mode0: PWM drive khoảng 976,56 Hz.
void TungLam_Control_MotorV5::Mode0()
  {
    cancelTimedBrake(true);
    pwmMode = 0;

    DDRC |= 0xFF;

    DDRH |= (1 << PH3) | (1 << PH4) | (1 << PH5);
    DDRE |= (1 << PE3);

      Reset_Timer(3);
      Reset_Timer(4);

      TCCR3A |= (1 << WGM31);
      TCCR3B |= (1 << WGM32) | (1 << WGM33);

      TCCR3A |= (1 << COM3A1);

      TCCR3B |= (1 << CS31) | (1 << CS30);

      ICR3 = 255;

      TCCR4A |= (1 << WGM41);
      TCCR4B |= (1 << WGM42) | (1 << WGM43);

      TCCR4A |= (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1);

      TCCR4B |= (1 << CS41) | (1 << CS40);

      ICR4 = 255;

      OCR3A = 0;
      OCR4A = 0;
      OCR4B = 0;
      OCR4C = 0;
}

void TungLam_Control_MotorV5:: Init_Timer1(uint8_t duty11, uint8_t duty12)
{

      Reset_Timer(1);

      DDRB |= (1 << PB6);
      DDRB |= (1 << PB5);
      TCCR1A |= (1 << WGM11);
      TCCR1B |= (1 << WGM12) | (1 << WGM13);
      TCCR1A |= (1 << COM1B1);
      TCCR1A |= (1 << COM1A1);
      TCCR1B |= (1 << CS10) | (1 << CS11);
      ICR1 = 255;

      OCR1B = duty12;
      OCR1A = duty11;
}

void TungLam_Control_MotorV5:: Init_Timer2(uint8_t duty9, uint8_t duty10)
{

      Reset_Timer(2);

      TCCR2A |= (1 << WGM21) | (1 << WGM20);
      TCCR2B |= (1 << CS22);

      DDRH |= (1 << PH6);
      TCCR2A |= (1 << COM2B1);
      OCR2B = duty9;

      DDRB |= (1 << PB4);
      TCCR2A |= (1 << COM2A1);
      OCR2A = duty10;
}

void TungLam_Control_MotorV5::STOP() {
    cancelTimedBrake(false);
    legacyApplyWheels(0, 0, 0, 0);
    pre = 0;
    isMoving = false;
    startTime = 0;
}

void TungLam_Control_MotorV5::moveForward(uint8_t duty) {
    Tim();
    legacyApplyWheels(duty, duty, duty, duty);
    pre = 1;
}

void TungLam_Control_MotorV5::moveBackward(uint8_t duty) {
    Tim();
    legacyApplyWheels(-(int16_t)duty, -(int16_t)duty, -(int16_t)duty, -(int16_t)duty);
    pre = 2;
}

void TungLam_Control_MotorV5::Forward_Right(uint8_t duty) {
    Tim();
    legacyApplyWheels(duty, 0, duty, 0);
    pre = 3;
}

void TungLam_Control_MotorV5::Backward_Right(uint8_t duty) {
    Tim();
    legacyApplyWheels(0, -(int16_t)duty, 0, -(int16_t)duty);
    pre = 4;
}

void TungLam_Control_MotorV5::moveRight(uint8_t duty) {
    Tim();
    legacyApplyWheels(duty, duty, -(int16_t)duty, -(int16_t)duty);
    pre = 5;
}

void TungLam_Control_MotorV5::moveLeft(uint8_t duty) {
    Tim();
    legacyApplyWheels(-(int16_t)duty, -(int16_t)duty, duty, duty);
    pre = 6;
}

void TungLam_Control_MotorV5::moveLeftSide(uint8_t duty) {
    Tim();
    legacyApplyWheels(-(int16_t)duty, duty, -(int16_t)duty, duty);
    pre = 7;
}

void TungLam_Control_MotorV5::moveRightSide(uint8_t duty) {
    Tim();
    legacyApplyWheels(duty, -(int16_t)duty, duty, -(int16_t)duty);
    pre = 8;
}

void TungLam_Control_MotorV5::Forward_Left(uint8_t duty) {
    Tim();
    legacyApplyWheels(0, duty, 0, duty);
    pre = 9;
}

void TungLam_Control_MotorV5::Backward_Left(uint8_t duty) {
    Tim();
    legacyApplyWheels(-(int16_t)duty, 0, -(int16_t)duty, 0);
    pre = 10;
}

// ABS V5: giữ semantics duty/timing cũ nhưng tự cắt bằng Timer3 ISR.
void TungLam_Control_MotorV5::ABS(uint8_t duty) {
    if (timedBrakeIsActive()) {
        return;
    }

    const uint8_t previousMotion = pre;
    if (previousMotion == 0) {
        STOP();
        return;
    }

    Timer();

    switch (previousMotion) {
      case 1:
        legacyApplyWheels(-(int16_t)duty, -(int16_t)duty, -(int16_t)duty, -(int16_t)duty);
        break;
      case 2:
        legacyApplyWheels(duty, duty, duty, duty);
        break;
      case 3:
        legacyApplyWheels(-(int16_t)duty, 0, -(int16_t)duty, 0);
        break;
      case 4:
        legacyApplyWheels(0, duty, 0, duty);
        break;
      case 5:
        legacyApplyWheels(-(int16_t)duty, -(int16_t)duty, duty, duty);
        break;
      case 6:
        legacyApplyWheels(duty, duty, -(int16_t)duty, -(int16_t)duty);
        break;
      case 7:
        legacyApplyWheels(duty, -(int16_t)duty, duty, -(int16_t)duty);
        break;
      case 8:
        legacyApplyWheels(-(int16_t)duty, duty, -(int16_t)duty, duty);
        break;
      case 9:
        legacyApplyWheels(0, -(int16_t)duty, 0, -(int16_t)duty);
        break;
      case 10:
        legacyApplyWheels(duty, 0, duty, 0);
        break;
      default:
        STOP();
        return;
    }

    pre = 0;
    isMoving = false;
    startTime = 0;

    startTimedBrakeOneShot(TIM, pwmMode == 1);
}

void TungLam_Control_MotorV5::Tim() {
    if (!isMoving) {
        startTime = millis();
        isMoving = true;
    }
}

// Lưu bảng 6 mốc thời gian ABS tương thích V5.
void TungLam_Control_MotorV5::setTimABS(uint8_t t500, uint8_t t1000, uint8_t t1500, uint8_t t2000, uint8_t t3000, uint8_t tAbove3000) {
    tim500  = t500 ;
    tim1000 = t1000;
    tim1500 = t1500;
    tim2000 = t2000;
    tim3000 = t3000;
    timAbove3000 = tAbove3000;
}

void TungLam_Control_MotorV5::Timer() {
    unsigned long duration = millis() - startTime;
    if (duration < 500) TIM = tim500;
    else if (duration < 1000) TIM = tim1000;
    else if (duration < 1500) TIM = tim1500;
    else if (duration < 2000) TIM = tim2000;
    else if (duration < 3000) TIM = tim3000;
    else TIM = timAbove3000;
    isMoving = false;
    startTime = 0;
}

void TungLam_Control_MotorV5::Tien(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    legacyApplyWheels(duty1, duty2, duty3, duty4);
    pre = 1;
}

void TungLam_Control_MotorV5::Lui(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    legacyApplyWheels(-(int16_t)duty1, -(int16_t)duty2, -(int16_t)duty3, -(int16_t)duty4);
    pre = 2;
}

void TungLam_Control_MotorV5::T_Phai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    (void)duty2;
    (void)duty4;
    legacyApplyWheels(duty1, 0, duty3, 0);
    pre = 3;
}

void TungLam_Control_MotorV5::L_Phai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    (void)duty1;
    (void)duty3;
    legacyApplyWheels(0, -(int16_t)duty2, 0, -(int16_t)duty4);
    pre = 4;
}

void TungLam_Control_MotorV5::Phai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    legacyApplyWheels(duty1, duty2, -(int16_t)duty3, -(int16_t)duty4);
    pre = 5;
}

void TungLam_Control_MotorV5::Trai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    legacyApplyWheels(-(int16_t)duty1, -(int16_t)duty2, duty3, duty4);
    pre = 6;
}

void TungLam_Control_MotorV5::N_Trai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    legacyApplyWheels(-(int16_t)duty1, duty2, -(int16_t)duty3, duty4);
    pre = 7;
}

void TungLam_Control_MotorV5::N_Phai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    legacyApplyWheels(duty1, -(int16_t)duty2, duty3, -(int16_t)duty4);
    pre = 8;
}

void TungLam_Control_MotorV5::T_Trai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    (void)duty1;
    (void)duty3;
    legacyApplyWheels(0, duty2, 0, duty4);
    pre = 9;
}

void TungLam_Control_MotorV5::L_Trai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    (void)duty2;
    (void)duty4;
    legacyApplyWheels(-(int16_t)duty1, 0, -(int16_t)duty3, 0);
    pre = 10;
}
