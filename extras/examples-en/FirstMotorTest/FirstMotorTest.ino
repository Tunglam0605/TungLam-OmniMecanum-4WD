/**
 * @file FirstMotorTest.ino
 * @brief First commissioning test for M1..M4 placement and polarity.
 *
 * Lift the robot before running.
 * M1=front-left D5, M2=rear-left D6, M3=rear-right D7, M4=front-right D8.
 */

#include <TungLam_OmniMecanum_4WD.h>

TungLamDrive4WD robot;

constexpr int16_t TEST_PWM = 80;
constexpr unsigned long RUN_MS = 800;
constexpr unsigned long PAUSE_MS = 400;

void driveOnly(uint8_t wheel, int16_t pwm);
void testWheel(uint8_t wheel);

void setup() {
  Serial.begin(115200);

  robot.begin(TungLamPwmMode::High7k8Hz);
  robot.stop();

  Serial.println(F("Lift robot. Motor test starts in 3 seconds."));
  delay(3000);

  for (uint8_t wheel = 1; wheel <= 4; ++wheel) {
    testWheel(wheel);
  }

  robot.stop();
  Serial.println(F("Motor test complete."));
}

void loop() {
}

void testWheel(uint8_t wheel) {
  Serial.print(F("M"));
  Serial.print(wheel);
  Serial.println(F(" forward"));
  driveOnly(wheel, TEST_PWM);
  delay(RUN_MS);
  robot.stop();
  delay(PAUSE_MS);

  Serial.print(F("M"));
  Serial.print(wheel);
  Serial.println(F(" reverse"));
  driveOnly(wheel, -TEST_PWM);
  delay(RUN_MS);
  robot.stop();
  delay(PAUSE_MS);
}

void driveOnly(uint8_t wheel, int16_t pwm) {
  switch (wheel) {
    case 1: robot.setWheels(pwm, 0, 0, 0); break;
    case 2: robot.setWheels(0, pwm, 0, 0); break;
    case 3: robot.setWheels(0, 0, pwm, 0); break;
    case 4: robot.setWheels(0, 0, 0, pwm); break;
    default: robot.stop(); break;
  }
}
