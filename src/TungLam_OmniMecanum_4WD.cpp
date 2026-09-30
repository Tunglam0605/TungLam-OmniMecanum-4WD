#if !defined(__AVR_ATmega2560__) && !defined(__AVR_ATmega1280__)
#error "TungLam_OmniMecanum_4WD requires Arduino Mega / ATmega2560 or ATmega1280 timer and PORT layout."
#endif

#include "TungLam_OmniMecanum_4WD.h"

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
      brakeTAbove3000_(85),
      autoBrakeMinDuty_(70),
      autoBrakeMaxDuty_(160),
      autoBrakePercent_(60) {}

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

void TungLamDrive4WD::activeBrake(uint8_t brakeDuty) {
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

  preBrake_ = commanded_;

  const uint32_t duration = moving_ ? (uint32_t)(millis() - motionStartMs_) : 0;
  brakeDurationMs_ = selectBrakeDuration(duration);

  uint8_t selectedDuty = brakeDuty;
  if (selectedDuty == 0) {
    selectedDuty = selectAutoBrakeDuty();
  }

  Wheels reverse = {0, 0, 0, 0};
  const int16_t previous[4] = {
      preBrake_.m1, preBrake_.m2, preBrake_.m3, preBrake_.m4
  };
  int16_t* target[4] = {
      &reverse.m1, &reverse.m2, &reverse.m3, &reverse.m4
  };

  for (uint8_t i = 0; i < 4; ++i) {
    const uint8_t previousDuty = magnitude(previous[i]);
    const uint8_t wheelBrakeDuty = previousDuty < selectedDuty ? previousDuty : selectedDuty;
    if (previous[i] > 0) {
      *target[i] = -(int16_t)wheelBrakeDuty;
    } else if (previous[i] < 0) {
      *target[i] = (int16_t)wheelBrakeDuty;
    }
  }

  braking_ = true;
  brakeStartMs_ = millis();
  applyWheelsRaw(reverse);
}

void TungLamDrive4WD::ABS(uint8_t brakeDuty) {
  activeBrake(brakeDuty);
}

bool TungLamDrive4WD::isBraking() const {
  return braking_;
}

void TungLamDrive4WD::cancelBrake() {
  if (braking_ || dynamicBraking_) {
    stop();
  }
}

void TungLamDrive4WD::setBrakeTimings(uint8_t t500,
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

void TungLamDrive4WD::setAutoBrakeDuty(uint8_t minDuty, uint8_t maxDuty, uint8_t percent) {
  if (minDuty > maxDuty) {
    const uint8_t temp = minDuty;
    minDuty = maxDuty;
    maxDuty = temp;
  }
  autoBrakeMinDuty_ = minDuty;
  autoBrakeMaxDuty_ = maxDuty;
  autoBrakePercent_ = percent > 100 ? 100 : percent;
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

uint8_t TungLamDrive4WD::selectAutoBrakeDuty() const {
  uint8_t maxPrevious = magnitude(preBrake_.m1);
  const uint8_t d2 = magnitude(preBrake_.m2);
  const uint8_t d3 = magnitude(preBrake_.m3);
  const uint8_t d4 = magnitude(preBrake_.m4);

  if (d2 > maxPrevious) maxPrevious = d2;
  if (d3 > maxPrevious) maxPrevious = d3;
  if (d4 > maxPrevious) maxPrevious = d4;

  uint16_t scaled = ((uint16_t)maxPrevious * autoBrakePercent_) / 100U;
  if (scaled < autoBrakeMinDuty_) scaled = autoBrakeMinDuty_;
  if (scaled > autoBrakeMaxDuty_) scaled = autoBrakeMaxDuty_;
  if (scaled > 255U) scaled = 255U;
  return (uint8_t)scaled;
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
