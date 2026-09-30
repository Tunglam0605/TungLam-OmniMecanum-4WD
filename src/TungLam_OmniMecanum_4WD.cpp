#if !defined(__AVR_ATmega2560__) && !defined(__AVR_ATmega1280__)
#error "TungLam_OmniMecanum_4WD requires Arduino Mega / ATmega2560 or ATmega1280 timer and PORT layout."
#endif

#include "TungLam_OmniMecanum_4WD.h"
#include <avr/interrupt.h>

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

void TungLamDrive4WD::begin(TungLamPwmMode pwmMode) {
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

  writeAllPwm(0);
}

void TungLamDrive4WD::update() {
  if (!braking_) {
    return;
  }

  if ((uint32_t)(millis() - brakeStartMs_) >= brakeDurationMs_) {
    braking_ = false;
    moving_ = false;
    motionStartMs_ = 0;
    commanded_ = {0, 0, 0, 0};
    applyWheelsRaw(commanded_);
  }
}

void TungLamDrive4WD::setChassis(TungLamChassis chassis) {
  chassis_ = chassis;
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

void TungLamDrive4WD::setWheels(int16_t m1, int16_t m2, int16_t m3, int16_t m4) {
  Wheels next = {
      clampWheel(m1),
      clampWheel(m2),
      clampWheel(m3),
      clampWheel(m4)
  };

  if (braking_) {
    braking_ = false;
  }

  if (dynamicBraking_) {
    // Leave bridge-brake state safely: disable EN/PWM before restoring DIR.
    writeAllPwm(0);
    if (deadTimeUs_ > 0) {
      delayMicroseconds(deadTimeUs_);
    }
    PORTC = 0x00;
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

void TungLamDrive4WD::drive(int16_t vx, int16_t vy, int16_t wz) {
  if (chassis_ == TungLamChassis::OmniX) {
    driveOmniX(vx, vy, wz);
  } else {
    driveMecanum(vx, vy, wz);
  }
}

void TungLamDrive4WD::driveMecanum(int16_t vx, int16_t vy, int16_t wz) {
  // Canonical Mecanum-X wheel order:
  // M1 = front-left, M2 = rear-left, M3 = front-right, M4 = rear-right.
  // Positive: vx forward, vy right, wz clockwise.
  const Wheels out = normalize(
      (int32_t)vx + vy + wz,
      (int32_t)vx - vy + wz,
      (int32_t)vx - vy - wz,
      (int32_t)vx + vy - wz);

  setWheels(out.m1, out.m2, out.m3, out.m4);
}

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

void TungLamDrive4WD::forward(uint8_t duty) {
  drive((int16_t)duty, 0, 0);
}

void TungLamDrive4WD::backward(uint8_t duty) {
  drive(-(int16_t)duty, 0, 0);
}

void TungLamDrive4WD::strafeRight(uint8_t duty) {
  drive(0, (int16_t)duty, 0);
}

void TungLamDrive4WD::strafeLeft(uint8_t duty) {
  drive(0, -(int16_t)duty, 0);
}

void TungLamDrive4WD::rotateRight(uint8_t duty) {
  drive(0, 0, (int16_t)duty);
}

void TungLamDrive4WD::rotateLeft(uint8_t duty) {
  drive(0, 0, -(int16_t)duty);
}

void TungLamDrive4WD::stop() {
  braking_ = false;
  dynamicBraking_ = false;
  moving_ = false;
  motionStartMs_ = 0;
  commanded_ = {0, 0, 0, 0};
  applyWheelsRaw(commanded_);
}

void TungLamDrive4WD::coast() {
  stop();
}

void TungLamDrive4WD::dynamicBrake() {
  braking_ = false;
  dynamicBraking_ = true;
  moving_ = false;
  motionStartMs_ = 0;
  commanded_ = {0, 0, 0, 0};

  // L298N dynamic braking: EN active and both bridge inputs equal.
  PORTC = 0x00;
  writeAllPwm(255);
  applied_ = {0, 0, 0, 0};
}

void TungLamDrive4WD::ABS(uint8_t brakeDuty) {
  if (braking_) {
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

  // Preserve the exact wheel directions that were active before braking.
  preBrake_ = commanded_;

  // Legacy V5 timing behavior:
  // <500, <1000, <1500, <2000, <3000, >=3000 ms
  // selects one configurable reverse-pulse duration.
  const uint32_t motionDuration =
      moving_ ? (uint32_t)(millis() - motionStartMs_) : 0;
  brakeDurationMs_ = selectBrakeDuration(motionDuration);

  // Important compatibility behavior:
  // The user-provided duty is the actual reverse-brake strength.
  // It is NOT automatically reduced or capped based on the previous PWM.
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
  brakeStartMs_ = millis();
  applyWheelsRaw(reverse);
}

void TungLamDrive4WD::activeBrake(uint8_t brakeDuty) {
  ABS(brakeDuty);
}

bool TungLamDrive4WD::isBraking() const {
  return braking_;
}

void TungLamDrive4WD::cancelBrake() {
  if (braking_ || dynamicBraking_) {
    stop();
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

  const int16_t oldValue[4] = {
      applied_.m1, applied_.m2, applied_.m3, applied_.m4
  };
  const int16_t newValue[4] = {
      physical.m1, physical.m2, physical.m3, physical.m4
  };

  bool directionStateChanged = false;
  for (uint8_t i = 0; i < 4; ++i) {
    if (signOf(oldValue[i]) != signOf(newValue[i])) {
      directionStateChanged = true;
      break;
    }
  }

  // Never change an L298N input state while the previous PWM command is active.
  if (directionStateChanged) {
    writeAllPwm(0);
    if (deadTimeUs_ > 0) {
      delayMicroseconds(deadTimeUs_);
    }
  }

  writeDirectionPattern(physical);
  writePwm(physical);
  applied_ = physical;
}

void TungLamDrive4WD::writeDirectionPattern(const Wheels& wheels) {
  uint8_t pattern = 0;

  // Forward bit / reverse bit per wheel:
  // M1: PC7 / PC6
  // M2: PC5 / PC4
  // M3: PC3 / PC2
  // M4: PC0 / PC1
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

void TungLamDrive4WD::writePwm(const Wheels& wheels) {
  OCR3A = magnitude(wheels.m1);
  OCR4A = magnitude(wheels.m2);
  OCR4B = magnitude(wheels.m3);
  OCR4C = magnitude(wheels.m4);
}

void TungLamDrive4WD::writeAllPwm(uint8_t duty) {
  OCR3A = duty;
  OCR4A = duty;
  OCR4B = duty;
  OCR4C = duty;
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

uint8_t TungLamDrive4WD::magnitude(int16_t value) {
  if (value < 0) {
    value = -value;
  }
  return value > 255 ? 255 : (uint8_t)value;
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
// Legacy TungLam_Control_MotorV5 implementation
// ============================================================================
// Kept in the same translation unit as the modern core to keep src/ compact.
// Public behavior/signatures remain compatible with existing V5 sketches.

namespace {
volatile bool gLegacyAbsActive = false;
volatile uint16_t gLegacyAbsOverflowsRemaining = 0;

inline void legacyAbsStopHardwareFromISR() {
    OCR3A = 0;
    OCR4A = 0;
    OCR4B = 0;
    OCR4C = 0;
    PORTC = 0x00;

    TIMSK3 &= ~(1 << TOIE3);
    gLegacyAbsOverflowsRemaining = 0;
    gLegacyAbsActive = false;
}

void cancelLegacyAbs(bool stopOutputs) {
    const uint8_t oldSreg = SREG;
    cli();

    const bool wasActive = gLegacyAbsActive;
    TIMSK3 &= ~(1 << TOIE3);
    gLegacyAbsOverflowsRemaining = 0;
    gLegacyAbsActive = false;

    SREG = oldSreg;

    if (stopOutputs && wasActive) {
        OCR3A = 0;
        OCR4A = 0;
        OCR4B = 0;
        OCR4C = 0;
        PORTC = 0x00;
    }
}

uint16_t legacyAbsOverflowCount(uint8_t brakeMs, uint8_t pwmMode) {
    if (brakeMs == 0) {
        return 0;
    }

    // Timer3 is already the PWM timebase owned by this library.
    // Mode1: 16 MHz / (8 * 256)  = 7812.5 overflows/s = 125/16 per ms.
    // Mode0: 16 MHz / (64 * 256) = 976.5625 overflows/s = 125/128 per ms.
    if (pwmMode == 1) {
        return (uint16_t)(((uint32_t)brakeMs * 125UL + 15UL) / 16UL);
    }

    return (uint16_t)(((uint32_t)brakeMs * 125UL + 127UL) / 128UL);
}

void startLegacyAbsOneShot(uint8_t brakeMs, uint8_t pwmMode) {
    const uint16_t overflows = legacyAbsOverflowCount(brakeMs, pwmMode);

    if (overflows == 0) {
        OCR3A = 0;
        OCR4A = 0;
        OCR4B = 0;
        OCR4C = 0;
        PORTC = 0x00;
        return;
    }

    const uint8_t oldSreg = SREG;
    cli();

    gLegacyAbsOverflowsRemaining = overflows;
    gLegacyAbsActive = true;

    // Clear any stale Timer3 overflow flag, then arm only the overflow IRQ.
    TIFR3 = (1 << TOV3);
    TIMSK3 |= (1 << TOIE3);

    SREG = oldSreg;
}
}

ISR(TIMER3_OVF_vect) {
    if (!gLegacyAbsActive) {
        TIMSK3 &= ~(1 << TOIE3);
        return;
    }

    if (gLegacyAbsOverflowsRemaining > 0) {
        --gLegacyAbsOverflowsRemaining;
    }

    if (gLegacyAbsOverflowsRemaining == 0) {
        legacyAbsStopHardwareFromISR();
    }
}

TungLam_Control_MotorV5::TungLam_Control_MotorV5() {
  // Constructor để khởi tạo các giá trị mặc định nếu cần
}
void TungLam_Control_MotorV5:: Reset_45 (bool Off)
{
  // Legacy helper kept for compatibility. All four DIR pairs are on PORTC.
  if (Off == true) {
    PORTC &= ~((1 << PC0) | (1 << PC1) | (1 << PC4) | (1 << PC5));
  } else {
    PORTC &= ~((1 << PC2) | (1 << PC3) | (1 << PC6) | (1 << PC7));
  }
}
void TungLam_Control_MotorV5::Dir(uint8_t BanhNumber, bool Set)
{
  // Any new explicit motor command takes control immediately from a pending ABS pulse.
  cancelLegacyAbs(true);

  switch (BanhNumber)
  {
  case 1:
    if (Set == false)
    {
      PORTC |=  (1 << PC7);
      PORTC &= ~(1 << PC6);
    }
    if (Set == true)
    {
      PORTC |=  (1 << PC6);
      PORTC &= ~(1 << PC7);
    }
    break;  // Add break here

  case 2:
    if (Set == false)
    {
      PORTC |=  (1 << PC5);
      PORTC &= ~(1 << PC4);
    }
    if (Set == true)
    {
      PORTC |=  (1 << PC4);
      PORTC &= ~(1 << PC5);
    }
    break;  // Add break here

  case 3:
    if (Set == false)
    {
      PORTC |=  (1 << PC3);
      PORTC &= ~(1 << PC2);
    }
    if (Set == true)
    {
      PORTC |=  (1 << PC2);
      PORTC &= ~(1 << PC3);
    }
    break;  // Add break here

  case 4:
    if (Set == false)
    {
      PORTC |=  (1 << PC0);
      PORTC &= ~(1 << PC1);
    }
    if (Set == true)
    {
      PORTC |=  (1 << PC1);
      PORTC &= ~(1 << PC0);
    }
    break;  // Add break here
  }
}

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


void TungLam_Control_MotorV5::Mode1() // Cài đặt tần số cao cho 4 chân 5,6,7,8 Mode riêng để điều khiển xe 4 bánh đa hướng với cài đặt các chân chiều và chân PWM
 {
    cancelLegacyAbs(true);
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
void TungLam_Control_MotorV5::Mode0() // Cài đặt tần số thấp cho 4 chân 5,6,7,8 Mode riêng để điều khiển xe 4 bánh đa hướng với cài đặt các chân chiều và chân PWM
  {
    cancelLegacyAbs(true);
    pwmMode = 0;
    // Cài đặt các chân điều khiển chiều (Dir) từ PC0 đến PC7 là OUTPUT
    DDRC |= 0xFF;

    // Khai báo chân 5, 6, 7, 8 là OUTPUT PWM
    DDRH |= (1 << PH3) | (1 << PH4) | (1 << PH5); // PORT H: PH3 = Chân 6, PH4 = Chân 7, PH5 = Chân 8
    DDRE |= (1 << PE3);                           // PORT E: PE3 = Chân 5

    // Đặt lại các thanh ghi Timer 3, Timer 4
      Reset_Timer(3);
      Reset_Timer(4);

    // Cấu hình Timer 3 cho chân 5 (OC3A)
      // Fast PWM với TOP = ICR3
      TCCR3A |= (1 << WGM31);
      TCCR3B |= (1 << WGM32) | (1 << WGM33);

      // Clear OC3A/OC3B on Compare Match
      TCCR3A |= (1 << COM3A1);

      // Prescaler = 64
      TCCR3B |= (1 << CS31) | (1 << CS30);

      // Đặt TOP và giá trị PWM ban đầu
      ICR3 = 255;  // TOP

    // Cấu hình Timer 4 cho chân 6, 7, 8 (OC4A, OC4B, OC4C)
      // Fast PWM với TOP = ICR4
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
void TungLam_Control_MotorV5:: Init_Timer1(uint8_t duty11, uint8_t duty12) // Cài đặt Timer1 tần số thấp cho chân 11 12 chuyên dùng cho Planet có PID hộp
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
void TungLam_Control_MotorV5:: Init_Timer2(uint8_t duty9, uint8_t duty10) // Cài đặt Timer 2 tần số thấp cho chân 9 10 chuyên dùng cho Planet có PID hộp
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
void TungLam_Control_MotorV5::STOP() {
    cancelLegacyAbs(false);
    setPWM(0);
    setSTOP(0);
    pre = 0;
    isMoving = false;
    startTime = 0;
}
void TungLam_Control_MotorV5::moveForward(uint8_t duty) {
    Tim();
    Dir(1, Set);
    Dir(2, Set);
    Dir(3, Set);
    Dir(4, Set);
    setPWM(duty);
    pre = 1; // Tiến
}

void TungLam_Control_MotorV5::moveBackward(uint8_t duty) {
    Tim();
    Dir(1,!Set);
    Dir(2,!Set);
    Dir(3,!Set);
    Dir(4,!Set);
    setPWM(duty);
    pre = 2; // Lùi
}

void TungLam_Control_MotorV5::Forward_Right(uint8_t duty) {
    Tim();
    Dir(1, Set);
    Dir(3, Set);
    PWM(duty, 0, duty, 0);
    pre = 3;
}
void TungLam_Control_MotorV5::Backward_Right(uint8_t duty) {
    Tim();
    Dir(2, !Set);
    Dir(4, !Set);
    PWM(0, duty, 0, duty);
    pre = 4;
}
void TungLam_Control_MotorV5::moveRight(uint8_t duty) {
    Tim();
    Dir(1, Set);
    Dir(2, Set);
    Dir(3, !Set);
    Dir(4, !Set);
  // Quay phải
    setPWM(duty);
    pre = 5; // Quay phải
}

void TungLam_Control_MotorV5::moveLeft(uint8_t duty) {
    Tim();
    Dir(1, !Set);
    Dir(2, !Set);
    Dir(3, Set);
    Dir(4, Set);
  // Quay trái
    setPWM(duty);
    pre = 6; // Quay trái
}

void TungLam_Control_MotorV5::moveLeftSide(uint8_t duty) {
    Tim();
    Dir(1, !Set);
    Dir(2, Set);
    Dir(3, !Set);
    Dir(4, Set);
  // Ngang trái
    setPWM(duty);
    pre = 7;
}

void TungLam_Control_MotorV5::moveRightSide(uint8_t duty) {
    Tim();
    Dir(1, Set);
    Dir(2, !Set);
    Dir(3, Set);
    Dir(4, !Set);
  // Ngang phải
    setPWM(duty);
    pre = 8; // Ngang trái
}

void TungLam_Control_MotorV5::Forward_Left(uint8_t duty) {
    Tim();
    Dir(2, Set);
    Dir(4, Set);
    PWM(0, duty, 0, duty);
    pre = 9;
}
void TungLam_Control_MotorV5::Backward_Left(uint8_t duty) {
    Tim();
    Dir(1, !Set);
    Dir(3, !Set);
    PWM(duty, 0, duty, 0);
    pre = 10;
}
void TungLam_Control_MotorV5::ABS(uint8_t duty) {
    if (gLegacyAbsActive) {
        return;
    }

    const uint8_t previousMotion = pre;
    if (previousMotion == 0) {
        STOP();
        return;
    }

    Timer();

    // Remove drive power before switching H-bridge direction.
    setPWM(0);
    delayMicroseconds(100);

    // Apply exactly the same opposite-motion mapping as the legacy V5 ABS,
    // but do it directly so we do not restart Tim() or overwrite pre.
    switch (previousMotion) {
      case 1:  // Forward -> Backward
        Dir(1, !Set); Dir(2, !Set); Dir(3, !Set); Dir(4, !Set);
        setPWM(duty);
        break;

      case 2:  // Backward -> Forward
        Dir(1, Set); Dir(2, Set); Dir(3, Set); Dir(4, Set);
        setPWM(duty);
        break;

      case 3:  // Forward_Right -> Backward_Left (M1, M3)
        Dir(1, !Set); Dir(3, !Set);
        PWM(duty, 0, duty, 0);
        break;

      case 4:  // Backward_Right -> Forward_Left (M2, M4)
        Dir(2, Set); Dir(4, Set);
        PWM(0, duty, 0, duty);
        break;

      case 5:  // Rotate right -> Rotate left
        Dir(1, !Set); Dir(2, !Set); Dir(3, Set); Dir(4, Set);
        setPWM(duty);
        break;

      case 6:  // Rotate left -> Rotate right
        Dir(1, Set); Dir(2, Set); Dir(3, !Set); Dir(4, !Set);
        setPWM(duty);
        break;

      case 7:  // Strafe left -> Strafe right
        Dir(1, Set); Dir(2, !Set); Dir(3, Set); Dir(4, !Set);
        setPWM(duty);
        break;

      case 8:  // Strafe right -> Strafe left
        Dir(1, !Set); Dir(2, Set); Dir(3, !Set); Dir(4, Set);
        setPWM(duty);
        break;

      case 9:  // Forward_Left -> Backward_Right (M2, M4)
        Dir(2, !Set); Dir(4, !Set);
        PWM(0, duty, 0, duty);
        break;

      case 10: // Backward_Left -> Forward_Right (M1, M3)
        Dir(1, Set); Dir(3, Set);
        PWM(duty, 0, duty, 0);
        break;

      default:
        STOP();
        return;
    }

    // The old public state is reset immediately; the hardware pulse continues
    // in the background and is terminated by TIMER3_OVF_vect.
    pre = 0;
    isMoving = false;
    startTime = 0;

    startLegacyAbsOneShot(TIM, pwmMode);
}

void TungLam_Control_MotorV5::Tim() {
    if (!isMoving) {
        startTime = millis();  // Ghi lại thời điểm bắt đầu di chuyển
        isMoving = true;       // Bật cờ di chuyển
    }
}

void TungLam_Control_MotorV5::setTimABS(uint8_t t500, uint8_t t1000, uint8_t t1500, uint8_t t2000, uint8_t t3000, uint8_t tAbove3000) {
    tim500  = t500 ;
    tim1000 = t1000;
    tim1500 = t1500;
    tim2000 = t2000;
    tim3000 = t3000;
    timAbove3000 = tAbove3000;
}

void TungLam_Control_MotorV5::Timer() {
    unsigned long duration = millis() - startTime;  // Tính thời gian đã di chuyển
    if (duration < 500) TIM = tim500;
    else if (duration < 1000) TIM = tim1000;
    else if (duration < 1500) TIM = tim1500;
    else if (duration < 2000) TIM = tim2000;
    else if (duration < 3000) TIM = tim3000;
    else TIM = timAbove3000;
    isMoving = false; // Tắt cờ
    startTime = 0;    // Reset startTime để chuẩn bị cho lần di chuyển tiếp theo
}

void TungLam_Control_MotorV5::setSTOP(uint8_t pattern) {
    // Cài đặt chiều động cơ từ pattern
    PORTC = pattern; // Giả sử PORTC được sử dụng để điều khiển các chân
}

void TungLam_Control_MotorV5::setPWM(uint8_t duty) {
    // Cài đặt giá trị PWM cho các chân
    OCR3A = duty;  // Chân 5
    OCR4A = duty;  // Chân 6
    OCR4B = duty;  // Chân 7
    OCR4C = duty;  // Chân 8
}

void TungLam_Control_MotorV5::PWM(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    // Cài đặt giá trị PWM cho các chân
    OCR3A = duty1;  // Chân 5
    OCR4A = duty2;  // Chân 6
    OCR4B = duty3;  // Chân 7
    OCR4C = duty4;  // Chân 8
}

void TungLam_Control_MotorV5::Tien(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    Dir(1, Set);
    Dir(2, Set);
    Dir(3, Set);
    Dir(4, Set);
    PWM(duty1,duty2,duty3,duty4);
    pre = 1;
}

void TungLam_Control_MotorV5::Lui(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    Dir(1,!Set);
    Dir(2,!Set);
    Dir(3,!Set);
    Dir(4,!Set);
    PWM(duty1,duty2,duty3,duty4);
    pre = 2;
}

void TungLam_Control_MotorV5::T_Phai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    (void)duty2;
    (void)duty4;
    Dir(1, Set);
    Dir(3, Set);
    PWM(duty1, 0, duty3, 0);
    pre = 3;
}
void TungLam_Control_MotorV5::L_Phai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    (void)duty1;
    (void)duty3;
    Dir(2, !Set);
    Dir(4, !Set);
    PWM(0, duty2, 0, duty4);
    pre = 4;
}
void TungLam_Control_MotorV5::Phai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    Dir(1, Set);
    Dir(2, Set);
    Dir(3, !Set);
    Dir(4, !Set);
  // Quay phải
    PWM(duty1,duty2,duty3,duty4);
    pre = 5;
}

void TungLam_Control_MotorV5::Trai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    Dir(1, !Set);
    Dir(2, !Set);
    Dir(3, Set);
    Dir(4, Set);
  // Quay trái
    PWM(duty1,duty2,duty3,duty4);
    pre = 6;
}

void TungLam_Control_MotorV5::N_Trai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    Dir(1, !Set);
    Dir(2, Set);
    Dir(3, !Set);
    Dir(4, Set);
  // Ngang trái
    PWM(duty1,duty2,duty3,duty4);
    pre = 7;
}

void TungLam_Control_MotorV5::N_Phai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    Dir(1, Set);
    Dir(2, !Set);
    Dir(3, Set);
    Dir(4, !Set);
  // Ngang phải
    PWM(duty1,duty2,duty3,duty4);
    pre = 8;
}

void TungLam_Control_MotorV5::T_Trai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    (void)duty1;
    (void)duty3;
    Dir(2, Set);
    Dir(4, Set);
    PWM(0, duty2, 0, duty4);
    pre = 9;
}
void TungLam_Control_MotorV5::L_Trai(uint8_t duty1,uint8_t duty2,uint8_t duty3,uint8_t duty4) {
    Tim();
    (void)duty2;
    (void)duty4;
    Dir(1, !Set);
    Dir(3, !Set);
    PWM(duty1, 0, duty3, 0);
    pre = 10;
}
