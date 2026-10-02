/**
 * @file TungLam_OmniMecanum_4WD.cpp
 * @brief Hardware implementation for TungLam_OmniMecanum_4WD.
 *
 * @details
 * This translation unit contains both:
 * - the modern TungLamDrive4WD implementation; and
 * - the source-compatible TungLam_Control_MotorV5 implementation.
 *
 * Hardware ownership on Arduino Mega 2560:
 * - PORTC PC0..PC7: direction lines D30..D37;
 * - Timer3 OC3A: M1 PWM on D5;
 * - Timer4 OC4A/OC4B/OC4C: M2/M3/M4 PWM on D6/D7/D8;
 * - Timer3 overflow interrupt: temporary one-shot scheduler for legacy ABS.
 *
 * The code intentionally uses direct AVR registers to minimize overhead and
 * preserve deterministic PWM behavior.
 */

// Direct-register pin/timer mapping below is specific to the Mega-class AVR.
#if !defined(__AVR_ATmega2560__) && !defined(__AVR_ATmega1280__)
#error "TungLam_OmniMecanum_4WD requires Arduino Mega / ATmega2560 or ATmega1280 timer and PORT layout."
#endif

#include "TungLam_OmniMecanum_4WD.h"
#include <avr/interrupt.h>

// ============================================================================
// INTERNAL COMMON MOTOR HAL + TIMER3 ONE-SHOT BRAKE SCHEDULER
// ============================================================================
//
// Both the modern API and the legacy V5 compatibility API ultimately drive the
// same physical hardware. Keeping one low-level path guarantees the same safe
// electrical transition sequence for every command:
//
//   current PWM -> 0 -> dead-time -> DIR update -> requested PWM
//
// Timer3 overflow is also used as a short one-shot scheduler for BOTH legacy
// and modern active-reverse braking. Therefore the brake pulse is terminated by
// hardware timing even when loop() is delayed or blocked.

namespace {

struct HalWheels {
  int16_t m1;
  int16_t m2;
  int16_t m3;
  int16_t m4;
};

// Physical signed state most recently applied to the H-bridges.
// Volatile because the Timer3 ISR clears it when a timed brake expires.
volatile int16_t gHalApplied[4] = {0, 0, 0, 0};

// Shared one-shot state. The motor hardware is singleton on one Mega board, so
// only one timed active-brake pulse can exist at a time.
volatile bool gTimedBrakeActive = false;
volatile uint16_t gTimedBrakeOverflowsRemaining = 0;

// Modern body-frame convention follows the standard right-handed mobile-robot
// frame used by ROS-style systems:
//   +X / +vx = forward
//   +Y / +vy = left
//   +Z / +wz = counter-clockwise yaw when viewed from above.
//
// The wheel signs below preserve the proven V5 physical movement vectors while
// assigning modern vx/vy/wz the standard body-axis signs.
//
// CHANNEL order used by every vector:
//   [M1, M2, M3, M4]
// = [front-left, rear-left, rear-right, front-right]
// = [D5, D6, D7, D8]
//
// Clockwise physical order viewed from above is:
//   M1 -> M4 -> M3 -> M2
constexpr int32_t mecanumM1(int32_t vx, int32_t vy, int32_t wz) {
  return vx - vy - wz;
}
constexpr int32_t mecanumM2(int32_t vx, int32_t vy, int32_t wz) {
  return vx + vy - wz;
}
constexpr int32_t mecanumM3(int32_t vx, int32_t vy, int32_t wz) {
  return vx - vy + wz;
}
constexpr int32_t mecanumM4(int32_t vx, int32_t vy, int32_t wz) {
  return vx + vy + wz;
}

static_assert(
    mecanumM1(1, 0, 0) == 1 && mecanumM2(1, 0, 0) == 1 &&
    mecanumM3(1, 0, 0) == 1 && mecanumM4(1, 0, 0) == 1,
    "Mecanum +vx forward basis must remain ++++");

static_assert(
    mecanumM1(0, 1, 0) == -1 && mecanumM2(0, 1, 0) == 1 &&
    mecanumM3(0, 1, 0) == -1 && mecanumM4(0, 1, 0) == 1,
    "Mecanum +vy left-strafe basis must remain -+-+");

static_assert(
    mecanumM1(0, -1, 0) == 1 && mecanumM2(0, -1, 0) == -1 &&
    mecanumM3(0, -1, 0) == 1 && mecanumM4(0, -1, 0) == -1,
    "Mecanum -vy right-strafe basis must remain +-+-");

static_assert(
    mecanumM1(0, 0, 1) == -1 && mecanumM2(0, 0, 1) == -1 &&
    mecanumM3(0, 0, 1) == 1 && mecanumM4(0, 0, 1) == 1,
    "Mecanum +wz CCW basis must remain --++");

static_assert(
    mecanumM1(1, -1, 0) == 2 && mecanumM2(1, -1, 0) == 0 &&
    mecanumM3(1, -1, 0) == 2 && mecanumM4(1, -1, 0) == 0,
    "Mecanum forward-right diagonal must remain +0+0");

static_assert(
    mecanumM1(1, 1, 0) == 0 && mecanumM2(1, 1, 0) == 2 &&
    mecanumM3(1, 1, 0) == 0 && mecanumM4(1, 1, 0) == 2,
    "Mecanum forward-left diagonal must remain 0+0+");

// Canonical normalized Omni-X sign helpers. Metric Omni-X uses the same signs
// plus the 45-degree projection factor and the physical rotation lever arm.
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

  // Forward bit / reverse bit per logical wheel:
  // M1: PC7 / PC6  -> D30 / D31
  // M2: PC5 / PC4  -> D32 / D33
  // M3: PC3 / PC2  -> D34 / D35
  // M4: PC0 / PC1  -> D37 / D36
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

/**
 * Cancel any active Timer3 brake one-shot.
 *
 * stopOutputs=true is used when a new command must immediately take ownership
 * from a brake pulse.
 */
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

/**
 * Apply one signed physical wheel vector through the common safe H-bridge path.
 */
void halApply(const HalWheels& target, uint16_t deadTimeUs) {
  // A new motor command always preempts an active reverse-brake pulse.
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

/**
 * Safely change one legacy DIR pair while preserving current PWM magnitudes.
 *
 * This supports direct user calls to legacy Dir() without allowing a live-PWM
 * H-bridge direction flip.
 */
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

  // Historical V5 convention: Set==false is logical wheel-forward.
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

/**
 * Convert a requested brake time to Timer3 PWM overflows.
 *
 * highPwmMode=true : 16 MHz / (8 * 256)  = 7812.5 overflow/s = 125/16 per ms
 * highPwmMode=false: 16 MHz / (64 * 256) = 976.5625 overflow/s = 125/128 per ms
 */
uint16_t timedBrakeOverflowCount(uint8_t brakeMs, bool highPwmMode) {
  if (brakeMs == 0) {
    return 0;
  }

  if (highPwmMode) {
    return (uint16_t)(((uint32_t)brakeMs * 125UL + 15UL) / 16UL);
  }

  return (uint16_t)(((uint32_t)brakeMs * 125UL + 127UL) / 128UL);
}

/**
 * Arm Timer3 overflow as a hardware-timed one-shot brake cutoff.
 */
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

  // Clear a stale Timer3 overflow flag before enabling the one-shot IRQ.
  TIFR3 = (1 << TOV3);
  TIMSK3 |= (1 << TOIE3);

  SREG = oldSreg;
}

inline bool timedBrakeIsActive() {
  return gTimedBrakeActive;
}

// Legacy movement commands use the same safe low-level path as modern commands.
inline void legacyApplyWheels(int16_t m1, int16_t m2, int16_t m3, int16_t m4) {
  halApply({m1, m2, m3, m4}, 100);
}

}  // namespace

/**
 * Timer3 overflow ISR shared by legacy and modern active-reverse braking.
 *
 * The ISR does only bounded direct-register work. It never calls millis(),
 * delay(), Serial or dynamic allocation.
 */
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
// MODERN TUNGLAMDRIVE4WD IMPLEMENTATION
// ============================================================================

/**
 * @brief Initialize all software state to safe defaults.
 *
 * No AVR register is touched here; hardware configuration is deferred to
 * begin(). This keeps global/static construction safe before Arduino runtime
 * initialization is complete.
 */
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

/**
 * @brief Configure direction GPIO, PWM GPIO and Timer3/Timer4.
 *
 * Direction outputs are cleared before PWM is initialized so the H-bridges
 * start from a deterministic coast/stop state.
 */
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
    // Fast PWM 8-bit, prescaler 8 -> 16 MHz / (8 * 256) = 7.8125 kHz.
    TCCR3A = (1 << WGM30) | (1 << COM3A1);
    TCCR3B = (1 << WGM32) | (1 << CS31);

    TCCR4A = (1 << WGM40) | (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1);
    TCCR4B = (1 << WGM42) | (1 << CS41);
  } else {
    // Fast PWM with TOP=ICRn, prescaler 64 -> 976.5625 Hz.
    TCCR3A = (1 << WGM31) | (1 << COM3A1);
    TCCR3B = (1 << WGM32) | (1 << WGM33) | (1 << CS31) | (1 << CS30);
    ICR3 = 255;

    TCCR4A = (1 << WGM41) | (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1);
    TCCR4B = (1 << WGM42) | (1 << WGM43) | (1 << CS41) | (1 << CS40);
    ICR4 = 255;
  }

  halWriteAllPwm(0);  // Never leave stale compare values active after timer setup.
}

/**
 * @brief Service the modern non-blocking reverse-brake timeout.
 *
 * Uses unsigned subtraction so millis() wrap-around remains safe.
 */
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

/**
 * @brief Configure physical motor polarity correction.
 *
 * Inversion is applied only in the hardware-output layer, leaving kinematics
 * and application commands in a consistent logical coordinate system.
 */
void TungLamDrive4WD::setMotorInverted(uint8_t wheel, bool inverted) {
  if (wheel < 1 || wheel > 4) {
    return;
  }
  inverted_[wheel - 1] = inverted;  // API is 1-based; storage is 0-based.
}

/** @brief Configure the PWM-off dead-time used before direction changes. */
void TungLamDrive4WD::setDirectionDeadTimeUs(uint16_t deadTimeUs) {
  deadTimeUs_ = deadTimeUs;  // Used only when the electrical sign state changes.
}

/**
 * @brief Apply a new raw signed four-wheel command.
 *
 * This function is the central state-transition point for the modern API:
 * it clamps demands, cancels active braking when a new command arrives,
 * exits dynamic braking safely, updates motion timing and writes hardware.
 */
void TungLamDrive4WD::setWheels(int16_t m1, int16_t m2, int16_t m3, int16_t m4) {
  // Synchronize a brake that may already have expired in the Timer3 ISR.
  update();
  noteCommandReceived();

  Wheels next = {
      clampWheel(m1),
      clampWheel(m2),
      clampWheel(m3),
      clampWheel(m4)
  };

  // A fresh movement command has priority over an active timed brake pulse.
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
    // Leave bridge-brake state safely: disable EN/PWM before restoring DIR.
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

/** @brief Dispatch a Cartesian command to the selected chassis mixer. */
void TungLamDrive4WD::drive(int16_t vx, int16_t vy, int16_t wz) {
  if (chassis_ == TungLamChassis::OmniX) {
    driveOmniX(vx, vy, wz);
  } else {
    driveMecanum(vx, vy, wz);
  }
}

/**
 * @brief Store a validated physical model for SI-unit open-loop kinematics.
 */
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

/** @brief Report whether SI-unit velocity control has a valid physical model. */
bool TungLamDrive4WD::hasDriveConfig() const {
  return driveConfigValid_;
}

/**
 * @brief Estimate no-load output RPM after voltage and empirical speed scaling.
 */
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
    // 45-degree X-drive wheel rolling axes:
    // M1 FL: +x -y, M2 RL: +x +y,
    // M3 RR: -x -y, M4 FR: -x +y.
    return {
        kInvSqrt2 * ( vxMps - vyMps - lever * wzRadps),
        kInvSqrt2 * ( vxMps + vyMps - lever * wzRadps),
        kInvSqrt2 * (-vxMps - vyMps - lever * wzRadps),
        kInvSqrt2 * (-vxMps + vyMps - lever * wzRadps)
    };
  }

  // Mecanum-X with M1 FL, M2 RL, M3 RR, M4 FR.
  return {
      vxMps - vyMps - lever * wzRadps,
      vxMps + vyMps - lever * wzRadps,
      vxMps - vyMps + lever * wzRadps,
      vxMps + vyMps + lever * wzRadps
  };
}

/**
 * @brief Forward kinematics: wheel-perimeter velocity -> body velocity.
 */
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
      - wheels.m3Mps + wheels.m4Mps) * 0.25f,
      (-wheels.m1Mps - wheels.m2Mps
      + wheels.m3Mps + wheels.m4Mps) / (4.0f * lever)
  };
}

/**
 * @brief SI body-velocity command -> inverse kinematics -> open-loop PWM.
 */
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
  // Standard right-handed body-frame basis:
  //   +vx forward = + + + +
  //   +vy left    = - + - +
  //   +wz CCW     = - - + +
  //
  // These physical wheel vectors are still exactly the proven V5 movements;
  // only the modern Cartesian sign convention is standardized.
  const Wheels out = normalize(
      mecanumM1(vx, vy, wz),
      mecanumM2(vx, vy, wz),
      mecanumM3(vx, vy, wz),
      mecanumM4(vx, vy, wz));

  setWheels(out.m1, out.m2, out.m3, out.m4);
}

/**
 * @brief Convert Cartesian chassis demand into canonical four-wheel Omni-X demand.
 *
 * The matrix assumes the documented X-drive wheel orientation. Real hardware
 * must be commissioned for wheel order and polarity before high-speed use.
 */
void TungLamDrive4WD::driveOmniX(int16_t vx, int16_t vy, int16_t wz) {
  // Canonical 45-degree Omni X-drive using the same right-handed body frame:
  //   +vx = forward, +vy = left, +wz = CCW.
  //
  // Positive wheel directions are chosen tangent-clockwise around the chassis,
  // giving ++-- for +vx, -+-+ for +vy, and ---- for +wz.
  const Wheels out = normalize(
      omniM1(vx, vy, wz),
      omniM2(vx, vy, wz),
      omniM3(vx, vy, wz),
      omniM4(vx, vy, wz));

  setWheels(out.m1, out.m2, out.m3, out.m4);
}

/** @brief Generate a pure +vx command. */
void TungLamDrive4WD::forward(uint8_t duty) {
  drive((int16_t)duty, 0, 0);
}

/** @brief Generate a pure -vx command. */
void TungLamDrive4WD::backward(uint8_t duty) {
  drive(-(int16_t)duty, 0, 0);
}

/** @brief Generate a pure -vy command (right translation). */
void TungLamDrive4WD::strafeRight(uint8_t duty) {
  drive(0, -(int16_t)duty, 0);
}

/** @brief Generate a pure +vy command (left translation). */
void TungLamDrive4WD::strafeLeft(uint8_t duty) {
  drive(0, (int16_t)duty, 0);
}

/** @brief Generate a pure -wz command (clockwise/right rotation). */
void TungLamDrive4WD::rotateRight(uint8_t duty) {
  drive(0, 0, -(int16_t)duty);
}

/** @brief Generate a pure +wz command (counter-clockwise/left rotation). */
void TungLamDrive4WD::rotateLeft(uint8_t duty) {
  drive(0, 0, (int16_t)duty);
}

/**
 * @brief Coast-stop and reset all modern motion/brake state.
 *
 * The zero logical wheel vector is passed through the same safe output path as
 * a normal command, ensuring PWM is removed before any direction-state change.
 */
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

/**
 * @brief Apply L298N bridge/dynamic braking.
 *
 * PORTC=0 makes both inputs of each H-bridge equal. With EN/PWM driven high,
 * this electrically damps the motor. It is not the strong reverse-torque ABS.
 */
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
  // If our previous pulse already expired in hardware, synchronize state first.
  update();
  noteCommandReceived();

  if (braking_ && timedBrakeIsActive()) {
    return;
  }

  // Do not steal the single physical motor peripheral from an unrelated timed
  // brake owner. Normal use should instantiate only one controller API.
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

  // Capture the exact wheel signs that were active before braking.
  preBrake_ = commanded_;

  const uint32_t motionDuration =
      moving_ ? (uint32_t)(millis() - motionStartMs_) : 0;
  brakeDurationMs_ = selectBrakeDuration(motionDuration);

  // The caller-provided duty remains the actual reverse-brake strength.
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

  // The common HAL inserts PWM-off dead-time before applying reverse torque.
  applyWheelsRaw(reverse);

  // Hardware-timed cutoff: loop()/update() latency cannot extend the pulse.
  startTimedBrakeOneShot(
      brakeDurationMs_,
      pwmMode_ == TungLamPwmMode::High7k8Hz);

  // A zero-duration table entry stops immediately; synchronize that state now.
  update();
}

/** @brief Descriptive alias for ABS(brakeDuty). */
void TungLamDrive4WD::activeBrake(uint8_t brakeDuty) {
  ABS(brakeDuty);
}

/** @brief Report whether a modern active reverse-brake pulse is in progress. */
bool TungLamDrive4WD::isBraking() const {
  return braking_ && timedBrakeIsActive();
}

/** @brief Cancel active/dynamic braking and return the bridge to coast-stop. */
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

/** @brief Descriptive wrapper around the compatibility-named setTimABS(). */
void TungLamDrive4WD::setBrakeTimings(uint8_t t500,
                                      uint8_t t1000,
                                      uint8_t t1500,
                                      uint8_t t2000,
                                      uint8_t t3000,
                                      uint8_t tAbove3000) {
  setTimABS(t500, t1000, t1500, t2000, t3000, tAbove3000);
}

/**
 * @brief Convert logical wheel commands to safe physical H-bridge outputs.
 *
 * Sequence:
 * 1. copy logical commands;
 * 2. apply per-motor inversion;
 * 3. detect any electrical direction-state change;
 * 4. force PWM to zero and wait dead-time when needed;
 * 5. update direction bits;
 * 6. restore requested PWM magnitude.
 */
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

  // Both modern and legacy APIs converge on this same safe physical HAL.
  halApply(
      {physical.m1, physical.m2, physical.m3, physical.m4},
      deadTimeUs_);

  applied_ = physical;
}

/** @brief Select one reverse-brake duration from the six legacy-compatible ranges. */
uint8_t TungLamDrive4WD::selectBrakeDuration(uint32_t duration) const {
  if (duration < 500UL) return brakeT500_;
  if (duration < 1000UL) return brakeT1000_;
  if (duration < 1500UL) return brakeT1500_;
  if (duration < 2000UL) return brakeT2000_;
  if (duration < 3000UL) return brakeT3000_;
  return brakeTAbove3000_;
}

/** @brief Saturate one signed wheel demand to the legal -255..255 range. */
int16_t TungLamDrive4WD::clampWheel(int32_t value) {
  if (value > 255) return 255;
  if (value < -255) return -255;
  return (int16_t)value;
}

/** @brief Convert a signed value to the three-state sign representation -1/0/+1. */
int8_t TungLamDrive4WD::signOf(int16_t value) {
  if (value > 0) return 1;
  if (value < 0) return -1;
  return 0;
}

/** @brief Return true when any wheel in the vector has a non-zero demand. */
bool TungLamDrive4WD::anyMoving(const Wheels& wheels) {
  return wheels.m1 != 0 || wheels.m2 != 0 || wheels.m3 != 0 || wheels.m4 != 0;
}

/**
 * @brief Detect a true non-zero sign reversal between two logical wheel vectors.
 *
 * Transitions through zero are not counted here; hardware-level sign changes
 * are handled separately by applyWheelsRaw().
 */
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

/** @brief Validate all parameters required by the SI/open-loop model. */
bool TungLamDrive4WD::driveConfigIsValid(const TungLamDriveConfig& config) {
  return config.motorNominalVoltageV > 0.0f &&
         config.motorNoLoadRpm > 0.0f &&
         config.supplyVoltageV > 0.0f &&
         config.wheelRadiusM > 0.0f &&
         config.wheelbaseM > 0.0f &&
         config.trackWidthM > 0.0f &&
         config.speedScale > 0.0f;
}

/** @brief Convert one signed wheel-perimeter velocity to PWM feed-forward. */
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
// LEGACY TUNGLAM_CONTROL_MOTORV5 IMPLEMENTATION
// ============================================================================
//
// This code intentionally keeps the historical V5 movement mapping and public
// method signatures. The implementation is colocated with the modern core only
// to keep the Arduino library source tree compact.
//
// Both legacy and modern active-reverse braking use the same Timer3 overflow
// one-shot cutoff. The ISR is enabled only while a brake pulse is active and
// disables itself at completion or command preemption.
//

/**
 * @brief Construct the legacy compatibility controller.
 */
TungLam_Control_MotorV5::TungLam_Control_MotorV5() {
  // No hardware action here; Mode0()/Mode1() performs explicit initialization.
}
/**
 * @brief Write one legacy wheel-direction pair on PORTC.
 *
 * Before changing direction, any asynchronous legacy ABS pulse is cancelled so
 * an explicit movement command always takes immediate ownership of the motors.
 */
void TungLam_Control_MotorV5::Dir(uint8_t BanhNumber, bool Set)
{
  // Public legacy Dir() remains available, but direction changes are now safe:
  // PWM off -> 100 us dead-time -> DIR update -> restore PWM.
  halSetLegacyDirection(BanhNumber, Set, 100);
}

/**
 * @brief Clear the control and interrupt registers of Timer1..Timer4.
 *
 * This preserves the original direct-register initialization model.
 */
void TungLam_Control_MotorV5::Reset_Timer(uint8_t timerNumber)
{
    switch (timerNumber)
    {
    case 1:
        // Reset Timer 1
        TCCR1A = 0;
        TCCR1B = 0;
        TIMSK1 = 0;
        break;

    case 2:
        // Reset Timer 2
        TCCR2A = 0;
        TCCR2B = 0;
        TIMSK2 = 0;
        break;

    case 3:
        // Reset Timer 3
        TCCR3A = 0;
        TCCR3B = 0;
        TIMSK3 = 0;
        break;

    case 4:
        // Reset Timer 4
        TCCR4A = 0;
        TCCR4B = 0;
        TIMSK4 = 0;
        break;

    default:
        // Kh├┤ng thß╗▒c hiß╗çn g├¼ nß║┐u tham sß╗æ kh├┤ng hß╗úp lß╗ç
        break;
    }
}


/**
 * @brief Configure legacy high-frequency PWM mode (~7.8125 kHz).
 *
 * Timer3 drives M1 on D5 and Timer4 drives M2..M4 on D6..D8.
 * Existing pending ABS state is cancelled before timer reconfiguration.
 */
void TungLam_Control_MotorV5::Mode1()
 {
    cancelTimedBrake(true);
    pwmMode = 1;
    // C├ái ─æß║╖t c├íc ch├ón ─æiß╗üu khiß╗ân chiß╗üu (Dir) tß╗½ PC0 ─æß║┐n PC7 l├á OUTPUT
    DDRC |= 0xFF;

    // Khai b├ío ch├ón 5, 6, 7, 8 l├á OUTPUT PWM
    DDRH |= (1 << PH3) | (1 << PH4) | (1 << PH5); // PORT H: PH3 = Ch├ón 6, PH4 = Ch├ón 7, PH5 = Ch├ón 8
    DDRE |= (1 << PE3);                           // PORT E: PE3 = Ch├ón 5

    // ─Éß║╖t lß║íi c├íc thanh ghi Timer 3, Timer 4
      Reset_Timer(3);
      Reset_Timer(4);

    // Cß║Ñu h├¼nh Timer 3 cho ch├ón 5 (OC3A)
    TCCR3A = (1 << WGM30) | (1 << COM3A1);        // Fast PWM, Clear on Compare Match
    TCCR3B = (1 << WGM32) | (1 << CS31);          // Prescaler 8

    // Cß║Ñu h├¼nh Timer 4 cho ch├ón 6, 7, 8 (OC4A, OC4B, OC4C)
    TCCR4A = (1 << WGM40) | (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1); // Fast PWM, Clear on Compare Match
    TCCR4B = (1 << WGM42) | (1 << CS41);                                  // Prescaler 8

    // ─Éß║╖t gi├í trß╗ï ban ─æß║ºu cho c├íc ch├ón PWM
    OCR3A = 0; // Ch├ón 5
    OCR4A = 0; // Ch├ón 6
    OCR4B = 0; // Ch├ón 7
    OCR4C = 0; // Ch├ón 8
}
/**
 * @brief Configure legacy low-frequency PWM mode (~976.56 Hz).
 *
 * TOP is fixed at 255 via ICR3/ICR4 and the timer prescaler is 64.
 */
void TungLam_Control_MotorV5::Mode0()
  {
    cancelTimedBrake(true);
    pwmMode = 0;
    // C├ái ─æß║╖t c├íc ch├ón ─æiß╗üu khiß╗ân chiß╗üu (Dir) tß╗½ PC0 ─æß║┐n PC7 l├á OUTPUT
    DDRC |= 0xFF;

    // Khai b├ío ch├ón 5, 6, 7, 8 l├á OUTPUT PWM
    DDRH |= (1 << PH3) | (1 << PH4) | (1 << PH5); // PORT H: PH3 = Ch├ón 6, PH4 = Ch├ón 7, PH5 = Ch├ón 8
    DDRE |= (1 << PE3);                           // PORT E: PE3 = Ch├ón 5

    // ─Éß║╖t lß║íi c├íc thanh ghi Timer 3, Timer 4
      Reset_Timer(3);
      Reset_Timer(4);

    // Cß║Ñu h├¼nh Timer 3 cho ch├ón 5 (OC3A)      // Fast PWM mode with ICR3 as TOP.
      TCCR3A |= (1 << WGM31);
      TCCR3B |= (1 << WGM32) | (1 << WGM33);

      // Clear OC3A/OC3B on Compare Match
      TCCR3A |= (1 << COM3A1);

      // Prescaler = 64
      TCCR3B |= (1 << CS31) | (1 << CS30);

      // ─Éß║╖t TOP v├á gi├í trß╗ï PWM ban ─æß║ºu
      ICR3 = 255;  // TOP

    // Cß║Ñu h├¼nh Timer 4 cho ch├ón 6, 7, 8 (OC4A, OC4B, OC4C)      // Fast PWM mode with ICR4 as TOP.
      TCCR4A |= (1 << WGM41);
      TCCR4B |= (1 << WGM42) | (1 << WGM43);

      // Clear OC4A/OC4B/OC4C on Compare Match
      TCCR4A |= (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1);

      // Prescaler = 64
      TCCR4B |= (1 << CS41) | (1 << CS40);

      // ─Éß║╖t TOP v├á gi├í trß╗ï PWM ban ─æß║ºu
      ICR4 = 255;  // TOP
    // ─Éß║╖t gi├í trß╗ï ban ─æß║ºu cho c├íc ch├ón PWM
      OCR3A = 0; // Ch├ón 5
      OCR4A = 0; // Ch├ón 6
      OCR4B = 0; // Ch├ón 7
      OCR4C = 0; // Ch├ón 8
}
/**
 * @brief Configure auxiliary Timer1 PWM channels D11/D12.
 *
 * This function is preserved for old robot mechanisms outside the four-wheel
 * drive system. It resets Timer1 before applying the historical configuration.
 */
void TungLam_Control_MotorV5:: Init_Timer1(uint8_t duty11, uint8_t duty12)
{
    // ─Éß╗Öng c╞í ch├ón 11 12
      Reset_Timer(1);

      DDRB |= (1 << PB6);
      DDRB |= (1 << PB5);
      TCCR1A |= (1 << WGM11);
      TCCR1B |= (1 << WGM12) | (1 << WGM13);
      TCCR1A |= (1 << COM1B1);   // Ch├ón sß╗æ 12
      TCCR1A |= (1 << COM1A1);  // Chan sß╗æ 11
      TCCR1B |= (1 << CS10) | (1 << CS11);
      ICR1 = 255;

      OCR1B = duty12;  // PWM ch├ón 12
      OCR1A = duty11; // PWM ch├ón 11
}
/**
 * @brief Configure auxiliary Timer2 PWM channels D9/D10.
 *
 * Preserved for historical Planet/auxiliary motor mechanisms.
 */
void TungLam_Control_MotorV5:: Init_Timer2(uint8_t duty9, uint8_t duty10)
{
    // Reset cß║Ñu h├¼nh Timer2
      Reset_Timer(2);

    // Cß║Ñu h├¼nh tß║ºn sß╗æ Timer2
      TCCR2A |= (1 << WGM21) | (1 << WGM20);                                  // Chß║┐ ─æß╗Ö Fast PWM (TOP = 0xFF)
      TCCR2B |= (1 << CS22);                                                 // Chß╗ìn bß╗Ö chia xung 64

    // ─Éß╗Öng c╞í Planet ch├ón 9 PH6
      DDRH |= (1 << PH6);
      TCCR2A |= (1 << COM2B1);
      OCR2B = duty9;

    // ─Éß╗Öng c╞í Planet ch├ón 10 PB4
      DDRB |= (1 << PB4);
      TCCR2A |= (1 << COM2A1);
      OCR2A = duty10;
}
/**
 * @brief Stop all four drive motors and clear legacy movement state.
 */
void TungLam_Control_MotorV5::STOP() {
    cancelTimedBrake(false);
    legacyApplyWheels(0, 0, 0, 0);
    pre = 0;
    isMoving = false;
    startTime = 0;
}
/** @brief Legacy common-duty forward command; records movement code 1. */
void TungLam_Control_MotorV5::moveForward(uint8_t duty) {
    Tim();
    legacyApplyWheels(duty, duty, duty, duty);
    pre = 1;
}

/** @brief Legacy common-duty backward command; records movement code 2. */
void TungLam_Control_MotorV5::moveBackward(uint8_t duty) {
    Tim();
    legacyApplyWheels(-(int16_t)duty, -(int16_t)duty, -(int16_t)duty, -(int16_t)duty);
    pre = 2;
}

/** @brief Legacy forward-right diagonal command; active wheels M1 and M3. */
void TungLam_Control_MotorV5::Forward_Right(uint8_t duty) {
    Tim();
    legacyApplyWheels(duty, 0, duty, 0);
    pre = 3;
}
/** @brief Legacy backward-right diagonal command; active wheels M2 and M4. */
void TungLam_Control_MotorV5::Backward_Right(uint8_t duty) {
    Tim();
    legacyApplyWheels(0, -(int16_t)duty, 0, -(int16_t)duty);
    pre = 4;
}
/** @brief Legacy clockwise/right rotation command. */
void TungLam_Control_MotorV5::moveRight(uint8_t duty) {
    Tim();
    legacyApplyWheels(duty, duty, -(int16_t)duty, -(int16_t)duty);
    pre = 5;
}

/** @brief Legacy counter-clockwise/left rotation command. */
void TungLam_Control_MotorV5::moveLeft(uint8_t duty) {
    Tim();
    legacyApplyWheels(-(int16_t)duty, -(int16_t)duty, duty, duty);
    pre = 6;
}

/** @brief Legacy left-strafe command. */
void TungLam_Control_MotorV5::moveLeftSide(uint8_t duty) {
    Tim();
    legacyApplyWheels(-(int16_t)duty, duty, -(int16_t)duty, duty);
    pre = 7;
}

/** @brief Legacy right-strafe command. */
void TungLam_Control_MotorV5::moveRightSide(uint8_t duty) {
    Tim();
    legacyApplyWheels(duty, -(int16_t)duty, duty, -(int16_t)duty);
    pre = 8;
}

/** @brief Legacy forward-left diagonal command; active wheels M2 and M4. */
void TungLam_Control_MotorV5::Forward_Left(uint8_t duty) {
    Tim();
    legacyApplyWheels(0, duty, 0, duty);
    pre = 9;
}
/** @brief Legacy backward-left diagonal command; active wheels M1 and M3. */
void TungLam_Control_MotorV5::Backward_Left(uint8_t duty) {
    Tim();
    legacyApplyWheels(-(int16_t)duty, 0, -(int16_t)duty, 0);
    pre = 10;
}
/**
 * @brief Execute the historical V5 reverse-brake mapping without blocking.
 *
 * The user-supplied duty is applied directly as reverse torque. The old
 * movement code (pre=1..10) selects the exact opposite movement pattern.
 */
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

    // Preserve the exact historical V5 opposite-motion mapping, but execute it
    // through the common safe HAL and let Timer3 end the pulse deterministically.
    switch (previousMotion) {
      case 1:  // Forward -> Backward
        legacyApplyWheels(-(int16_t)duty, -(int16_t)duty, -(int16_t)duty, -(int16_t)duty);
        break;
      case 2:  // Backward -> Forward
        legacyApplyWheels(duty, duty, duty, duty);
        break;
      case 3:  // Forward-right -> Backward-left
        legacyApplyWheels(-(int16_t)duty, 0, -(int16_t)duty, 0);
        break;
      case 4:  // Backward-right -> Forward-left
        legacyApplyWheels(0, duty, 0, duty);
        break;
      case 5:  // Rotate right -> Rotate left
        legacyApplyWheels(-(int16_t)duty, -(int16_t)duty, duty, duty);
        break;
      case 6:  // Rotate left -> Rotate right
        legacyApplyWheels(duty, duty, -(int16_t)duty, -(int16_t)duty);
        break;
      case 7:  // Strafe left -> Strafe right
        legacyApplyWheels(duty, -(int16_t)duty, duty, -(int16_t)duty);
        break;
      case 8:  // Strafe right -> Strafe left
        legacyApplyWheels(-(int16_t)duty, duty, -(int16_t)duty, duty);
        break;
      case 9:  // Forward-left -> Backward-right
        legacyApplyWheels(0, -(int16_t)duty, 0, -(int16_t)duty);
        break;
      case 10: // Backward-left -> Forward-right
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

/**
 * @brief Start timing the current movement phase once.
 *
 * Repeated commands in the same movement phase do not continuously reset the
 * timestamp; this preserves the legacy ABS duration-selection behavior.
 */
void TungLam_Control_MotorV5::Tim() {
    if (!isMoving) {
        startTime = millis();  // Ghi lß║íi thß╗¥i ─æiß╗âm bß║»t ─æß║ºu di chuyß╗ân
        isMoving = true;       // Bß║¡t cß╗¥ di chuyß╗ân
    }
}

/** @brief Store the six user-configurable legacy ABS pulse durations. */
void TungLam_Control_MotorV5::setTimABS(uint8_t t500, uint8_t t1000, uint8_t t1500, uint8_t t2000, uint8_t t3000, uint8_t tAbove3000) {
    tim500  = t500 ;
    tim1000 = t1000;
    tim1500 = t1500;
    tim2000 = t2000;
    tim3000 = t3000;
    timAbove3000 = tAbove3000;
}

/**
 * @brief Convert elapsed movement time to one configured ABS pulse duration.
 */
void TungLam_Control_MotorV5::Timer() {
    unsigned long duration = millis() - startTime;  // T├¡nh thß╗¥i gian ─æ├ú di chuyß╗ân
    if (duration < 500) TIM = tim500;
    else if (duration < 1000) TIM = tim1000;
    else if (duration < 1500) TIM = tim1500;
    else if (duration < 2000) TIM = tim2000;
    else if (duration < 3000) TIM = tim3000;
    else TIM = timAbove3000;
    isMoving = false; // Tß║»t cß╗¥
    startTime = 0;    // Clear timestamp for the next movement phase
}

/** @brief Per-wheel-duty forward command; movement code 1. */
void TungLam_Control_MotorV5::Tien(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    legacyApplyWheels(duty1, duty2, duty3, duty4);
    pre = 1;
}

/** @brief Per-wheel-duty backward command; movement code 2. */
void TungLam_Control_MotorV5::Lui(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    legacyApplyWheels(-(int16_t)duty1, -(int16_t)duty2, -(int16_t)duty3, -(int16_t)duty4);
    pre = 2;
}

/** @brief Per-wheel-duty forward-right diagonal; uses M1 and M3. */
void TungLam_Control_MotorV5::T_Phai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    (void)duty2;
    (void)duty4;
    legacyApplyWheels(duty1, 0, duty3, 0);
    pre = 3;
}
/** @brief Per-wheel-duty backward-right diagonal; uses M2 and M4. */
void TungLam_Control_MotorV5::L_Phai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    (void)duty1;
    (void)duty3;
    legacyApplyWheels(0, -(int16_t)duty2, 0, -(int16_t)duty4);
    pre = 4;
}
/** @brief Per-wheel-duty clockwise/right rotation; movement code 5. */
void TungLam_Control_MotorV5::Phai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    legacyApplyWheels(duty1, duty2, -(int16_t)duty3, -(int16_t)duty4);
    pre = 5;
}

/** @brief Per-wheel-duty counter-clockwise/left rotation; movement code 6. */
void TungLam_Control_MotorV5::Trai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    legacyApplyWheels(-(int16_t)duty1, -(int16_t)duty2, duty3, duty4);
    pre = 6;
}

/** @brief Per-wheel-duty left strafe; movement code 7. */
void TungLam_Control_MotorV5::N_Trai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    legacyApplyWheels(-(int16_t)duty1, duty2, -(int16_t)duty3, duty4);
    pre = 7;
}

/** @brief Per-wheel-duty right strafe; movement code 8. */
void TungLam_Control_MotorV5::N_Phai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    legacyApplyWheels(duty1, -(int16_t)duty2, duty3, -(int16_t)duty4);
    pre = 8;
}

/** @brief Per-wheel-duty forward-left diagonal; uses M2 and M4. */
void TungLam_Control_MotorV5::T_Trai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    (void)duty1;
    (void)duty3;
    legacyApplyWheels(0, duty2, 0, duty4);
    pre = 9;
}
/** @brief Per-wheel-duty backward-left diagonal; uses M1 and M3. */
void TungLam_Control_MotorV5::L_Trai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    (void)duty2;
    (void)duty4;
    legacyApplyWheels(-(int16_t)duty1, 0, -(int16_t)duty3, 0);
    pre = 10;
}
