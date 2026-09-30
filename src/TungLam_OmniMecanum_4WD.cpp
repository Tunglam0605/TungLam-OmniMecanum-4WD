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

// Pure Mecanum basis helpers are constexpr so legacy parity is enforced at
// compile time, not only documented in comments.
constexpr int32_t mecanumV5M1(int32_t vx, int32_t vy, int32_t wz) {
  return vx + vy + wz;
}
constexpr int32_t mecanumV5M2(int32_t vx, int32_t vy, int32_t wz) {
  return vx - vy + wz;
}
constexpr int32_t mecanumV5M3(int32_t vx, int32_t vy, int32_t wz) {
  return vx + vy - wz;
}
constexpr int32_t mecanumV5M4(int32_t vx, int32_t vy, int32_t wz) {
  return vx - vy - wz;
}

static_assert(
    mecanumV5M1(1, 0, 0) == 1 && mecanumV5M2(1, 0, 0) == 1 &&
    mecanumV5M3(1, 0, 0) == 1 && mecanumV5M4(1, 0, 0) == 1,
    "Mecanum forward basis must remain V5-compatible: ++++");

static_assert(
    mecanumV5M1(0, 1, 0) == 1 && mecanumV5M2(0, 1, 0) == -1 &&
    mecanumV5M3(0, 1, 0) == 1 && mecanumV5M4(0, 1, 0) == -1,
    "Mecanum right-strafe basis must remain V5-compatible: +-+-");

static_assert(
    mecanumV5M1(0, 0, 1) == 1 && mecanumV5M2(0, 0, 1) == 1 &&
    mecanumV5M3(0, 0, 1) == -1 && mecanumV5M4(0, 0, 1) == -1,
    "Mecanum clockwise basis must remain V5-compatible: ++--");

static_assert(
    mecanumV5M1(1, 1, 0) == 2 && mecanumV5M2(1, 1, 0) == 0 &&
    mecanumV5M3(1, 1, 0) == 2 && mecanumV5M4(1, 1, 0) == 0,
    "Mecanum forward-right diagonal must remain V5-compatible: +0+0");

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
  // Ensure no stale one-shot state survives a reinitialization.
  cancelTimedBrake(true);

  pwmMode_ = pwmMode;

  // DIR pins D30..D37 = PC7..PC0.
  DDRC = 0xFF;
  PORTC = 0x00;

  // PWM pins:
  // D5  = PE3 / OC3A
  // D6  = PH3 / OC4A
  // D7  = PH4 / OC4B
  // D8  = PH5 / OC4C
  DDRE |= (1 << PE3);
  DDRH |= (1 << PH3) | (1 << PH4) | (1 << PH5);

  initPwm();
  stop();
}

/**
 * @brief Program Timer3 and Timer4 for the selected motor PWM frequency.
 *
 * Timer interrupt masks are cleared first because the library owns these
 * timers. PWM compare registers are forced to zero after configuration.
 */
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

  writeAllPwm(0);  // Never leave stale compare values active after timer setup.
}

/**
 * @brief Service the modern non-blocking reverse-brake timeout.
 *
 * Uses unsigned subtraction so millis() wrap-around remains safe.
 */
void TungLamDrive4WD::update() {
  // Hardware cutoff no longer depends on update(): Timer3 ISR stops the bridge.
  // update() now only synchronizes the high-level software state after expiry.
  if (braking_ && !timedBrakeIsActive()) {
    braking_ = false;
    moving_ = false;
    motionStartMs_ = 0;
    brakeStartMs_ = 0;
    brakeDurationMs_ = 0;
    commanded_ = {0, 0, 0, 0};
    applied_ = {0, 0, 0, 0};
  }
}

/** @brief Select which built-in holonomic mixer drive() will use. */
void TungLamDrive4WD::setChassis(TungLamChassis chassis) {
  chassis_ = chassis;
}

/** @brief Return the currently selected chassis mixer. */
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
 * @brief Convert Cartesian chassis demand into canonical Mecanum-X wheel demand.
 *
 * Wheel order: M1 front-left, M2 rear-left, M3 front-right, M4 rear-right.
 */
void TungLamDrive4WD::driveMecanum(int16_t vx, int16_t vy, int16_t wz) {
  // TungLam/V5-compatible Mecanum basis:
  //   forward      = + + + +
  //   strafe right = + - + -
  //   rotate right = + + - -
  //
  // This preserves the proven legacy robot movement convention while allowing
  // arbitrary vx/vy/wz mixing in the modern API.
  const Wheels out = normalize(
      mecanumV5M1(vx, vy, wz),
      mecanumV5M2(vx, vy, wz),
      mecanumV5M3(vx, vy, wz),
      mecanumV5M4(vx, vy, wz));

  setWheels(out.m1, out.m2, out.m3, out.m4);
}

/**
 * @brief Convert Cartesian chassis demand into canonical four-wheel Omni-X demand.
 *
 * The matrix assumes the documented X-drive wheel orientation. Real hardware
 * must be commissioned for wheel order and polarity before high-speed use.
 */
void TungLamDrive4WD::driveOmniX(int16_t vx, int16_t vy, int16_t wz) {
  // Canonical four-wheel Omni X-drive.
  // Wheel rolling directions are tangent to the X-drive layout.
  // Positive: vx forward, vy right, wz clockwise.
  const Wheels out = normalize(
      (int32_t)vx + vy + wz,
      (int32_t)vx - vy + wz,
      (int32_t)-vx - vy + wz,
      (int32_t)-vx + vy + wz);

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

/** @brief Generate a pure +vy command. */
void TungLamDrive4WD::strafeRight(uint8_t duty) {
  drive(0, (int16_t)duty, 0);
}

/** @brief Generate a pure -vy command. */
void TungLamDrive4WD::strafeLeft(uint8_t duty) {
  drive(0, -(int16_t)duty, 0);
}

/** @brief Generate a pure +wz clockwise command. */
void TungLamDrive4WD::rotateRight(uint8_t duty) {
  drive(0, 0, (int16_t)duty);
}

/** @brief Generate a pure -wz counter-clockwise command. */
void TungLamDrive4WD::rotateLeft(uint8_t duty) {
  drive(0, 0, -(int16_t)duty);
}

/**
 * @brief Coast-stop and reset all modern motion/brake state.
 *
 * The zero logical wheel vector is passed through the same safe output path as
 * a normal command, ensuring PWM is removed before any direction-state change.
 */
void TungLamDrive4WD::stop() {
  braking_ = false;
  dynamicBraking_ = false;
  moving_ = false;
  motionStartMs_ = 0;
  commanded_ = {0, 0, 0, 0};
  applyWheelsRaw(commanded_);
}

/** @brief Explicit coast-stop alias. */
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
  cancelTimedBrake(true);

  braking_ = false;
  dynamicBraking_ = true;
  moving_ = false;
  motionStartMs_ = 0;
  brakeStartMs_ = 0;
  brakeDurationMs_ = 0;
  commanded_ = {0, 0, 0, 0};

  // L298N dynamic braking: EN active and both bridge inputs equal.
  PORTC = 0x00;
  halWriteAllPwm(255);
  halRememberApplied({0, 0, 0, 0});
  applied_ = {0, 0, 0, 0};
}

/**
 * @brief Start the modern strong active reverse-brake pulse.
 *
 * Each moving wheel is commanded with the opposite sign at the exact
 * user-selected brakeDuty. The pulse duration comes from the six-stage table.
 * Timer3 overflow ISR is responsible for ending this modern brake pulse.
 */
void TungLamDrive4WD::ABS(uint8_t brakeDuty) {
  // If our previous pulse already expired in hardware, synchronize state first.
  update();

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
  }
}

/**
 * @brief Store the six active-brake duration thresholds.
 *
 * Values are milliseconds and deliberately remain uint8_t for V5-compatible
 * timing semantics and compact AVR RAM usage.
 */
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

/**
 * @brief Encode signed wheel directions into the complete PORTC bit pattern.
 *
 * A zero wheel command leaves both direction bits low for that wheel.
 */
void TungLamDrive4WD::writeDirectionPattern(const Wheels& wheels) {
  halWriteDirectionPattern({wheels.m1, wheels.m2, wheels.m3, wheels.m4});
}

/** @brief Write per-wheel absolute PWM magnitudes to Timer3/Timer4 OCR registers. */
void TungLamDrive4WD::writePwm(const Wheels& wheels) {
  halWritePwm({wheels.m1, wheels.m2, wheels.m3, wheels.m4});
}

/** @brief Write one common PWM duty to all four drive-motor compare registers. */
void TungLamDrive4WD::writeAllPwm(uint8_t duty) {
  halWriteAllPwm(duty);
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

/** @brief Convert a signed wheel demand to a PWM-safe 0..255 magnitude. */
uint8_t TungLamDrive4WD::magnitude(int16_t value) {
  if (value < 0) {
    value = -value;
  }
  return value > 255 ? 255 : (uint8_t)value;
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

/**
 * @brief Proportionally normalize a four-wheel vector into the PWM range.
 *
 * Scaling all four values by the same factor preserves the requested motion
 * vector better than clipping individual wheels independently.
 */
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
 * @brief Historical diagonal-direction cleanup helper retained for compatibility.
 *
 * The helper clears selected PORTC direction bits only; it does not drive PWM.
 */
void TungLam_Control_MotorV5:: Reset_45 (bool Off)
{
  // Historical helper now goes through the common safe HAL.
  HalWheels current = halCurrentApplied();

  if (Off == true) {
    // Disable M2 and M4.
    current.m2 = 0;
    current.m4 = 0;
  } else {
    // Disable M1 and M3.
    current.m1 = 0;
    current.m3 = 0;
  }

  halApply(current, 100);
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
        // Không thực hiện gì nếu tham số không hợp lệ
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
    // Cài đặt các chân điều khiển chiều (Dir) từ PC0 đến PC7 là OUTPUT
    DDRC |= 0xFF;

    // Khai báo chân 5, 6, 7, 8 là OUTPUT PWM
    DDRH |= (1 << PH3) | (1 << PH4) | (1 << PH5); // PORT H: PH3 = Chân 6, PH4 = Chân 7, PH5 = Chân 8
    DDRE |= (1 << PE3);                           // PORT E: PE3 = Chân 5

    // Đặt lại các thanh ghi Timer 3, Timer 4
      Reset_Timer(3);
      Reset_Timer(4);

    // Cấu hình Timer 3 cho chân 5 (OC3A)
    TCCR3A = (1 << WGM30) | (1 << COM3A1);        // Fast PWM, Clear on Compare Match
    TCCR3B = (1 << WGM32) | (1 << CS31);          // Prescaler 8

    // Cấu hình Timer 4 cho chân 6, 7, 8 (OC4A, OC4B, OC4C)
    TCCR4A = (1 << WGM40) | (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1); // Fast PWM, Clear on Compare Match
    TCCR4B = (1 << WGM42) | (1 << CS41);                                  // Prescaler 8

    // Đặt giá trị ban đầu cho các chân PWM
    OCR3A = 0; // Chân 5
    OCR4A = 0; // Chân 6
    OCR4B = 0; // Chân 7
    OCR4C = 0; // Chân 8
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
    // Cài đặt các chân điều khiển chiều (Dir) từ PC0 đến PC7 là OUTPUT
    DDRC |= 0xFF;

    // Khai báo chân 5, 6, 7, 8 là OUTPUT PWM
    DDRH |= (1 << PH3) | (1 << PH4) | (1 << PH5); // PORT H: PH3 = Chân 6, PH4 = Chân 7, PH5 = Chân 8
    DDRE |= (1 << PE3);                           // PORT E: PE3 = Chân 5

    // Đặt lại các thanh ghi Timer 3, Timer 4
      Reset_Timer(3);
      Reset_Timer(4);

    // Cấu hình Timer 3 cho chân 5 (OC3A)      // Fast PWM mode with ICR3 as TOP.
      TCCR3A |= (1 << WGM31);
      TCCR3B |= (1 << WGM32) | (1 << WGM33);

      // Clear OC3A/OC3B on Compare Match
      TCCR3A |= (1 << COM3A1);

      // Prescaler = 64
      TCCR3B |= (1 << CS31) | (1 << CS30);

      // Đặt TOP và giá trị PWM ban đầu
      ICR3 = 255;  // TOP

    // Cấu hình Timer 4 cho chân 6, 7, 8 (OC4A, OC4B, OC4C)      // Fast PWM mode with ICR4 as TOP.
      TCCR4A |= (1 << WGM41);
      TCCR4B |= (1 << WGM42) | (1 << WGM43);

      // Clear OC4A/OC4B/OC4C on Compare Match
      TCCR4A |= (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1);

      // Prescaler = 64
      TCCR4B |= (1 << CS41) | (1 << CS40);

      // Đặt TOP và giá trị PWM ban đầu
      ICR4 = 255;  // TOP
    // Đặt giá trị ban đầu cho các chân PWM
      OCR3A = 0; // Chân 5
      OCR4A = 0; // Chân 6
      OCR4B = 0; // Chân 7
      OCR4C = 0; // Chân 8
}
/**
 * @brief Configure auxiliary Timer1 PWM channels D11/D12.
 *
 * This function is preserved for old robot mechanisms outside the four-wheel
 * drive system. It resets Timer1 before applying the historical configuration.
 */
void TungLam_Control_MotorV5:: Init_Timer1(uint8_t duty11, uint8_t duty12)
{
    // Động cơ chân 11 12
      Reset_Timer(1);

      DDRB |= (1 << PB6);
      DDRB |= (1 << PB5);
      TCCR1A |= (1 << WGM11);
      TCCR1B |= (1 << WGM12) | (1 << WGM13);
      TCCR1A |= (1 << COM1B1);   // Chân số 12
      TCCR1A |= (1 << COM1A1);  // Chan số 11
      TCCR1B |= (1 << CS10) | (1 << CS11);
      ICR1 = 255;

      OCR1B = duty12;  // PWM chân 12
      OCR1A = duty11; // PWM chân 11
}
/**
 * @brief Configure auxiliary Timer2 PWM channels D9/D10.
 *
 * Preserved for historical Planet/auxiliary motor mechanisms.
 */
void TungLam_Control_MotorV5:: Init_Timer2(uint8_t duty9, uint8_t duty10)
{
    // Reset cấu hình Timer2
      Reset_Timer(2);

    // Cấu hình tần số Timer2
      TCCR2A |= (1 << WGM21) | (1 << WGM20);                                  // Chế độ Fast PWM (TOP = 0xFF)
      TCCR2B |= (1 << CS22);                                                 // Chọn bộ chia xung 64

    // Động cơ Planet chân 9 PH6
      DDRH |= (1 << PH6);
      TCCR2A |= (1 << COM2B1);
      OCR2B = duty9;

    // Động cơ Planet chân 10 PB4
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
        startTime = millis();  // Ghi lại thời điểm bắt đầu di chuyển
        isMoving = true;       // Bật cờ di chuyển
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
    unsigned long duration = millis() - startTime;  // Tính thời gian đã di chuyển
    if (duration < 500) TIM = tim500;
    else if (duration < 1000) TIM = tim1000;
    else if (duration < 1500) TIM = tim1500;
    else if (duration < 2000) TIM = tim2000;
    else if (duration < 3000) TIM = tim3000;
    else TIM = timAbove3000;
    isMoving = false; // Tắt cờ
    startTime = 0;    // Clear timestamp for the next movement phase
}

/** @brief Write a raw legacy direction pattern to PORTC. */
void TungLam_Control_MotorV5::setSTOP(uint8_t pattern) {
    // Cài đặt chiều động cơ từ pattern
    PORTC = pattern; // Giả sử PORTC được sử dụng để điều khiển các chân
}

/** @brief Write one common PWM duty to all four legacy drive outputs. */
void TungLam_Control_MotorV5::setPWM(uint8_t duty) {
    // Cài đặt giá trị PWM cho các chân
    OCR3A = duty;  // Chân 5
    OCR4A = duty;  // Chân 6
    OCR4B = duty;  // Chân 7
    OCR4C = duty;  // Chân 8
}

/** @brief Write independent legacy PWM duties to M1..M4. */
void TungLam_Control_MotorV5::PWM(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    // Cài đặt giá trị PWM cho các chân
    OCR3A = duty1;  // Chân 5
    OCR4A = duty2;  // Chân 6
    OCR4B = duty3;  // Chân 7
    OCR4C = duty4;  // Chân 8
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
