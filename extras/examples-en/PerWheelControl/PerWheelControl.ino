/**
 * @file PerWheelControl.ino
 * @brief Demonstrates the lowest public signed-wheel control layer.
 *
 * setWheels(m1, m2, m3, m4) accepts:
 * - positive value -> logical forward
 * - negative value -> logical reverse
 * - zero           -> stop that wheel
 * - magnitude      -> PWM 0..255
 *
 * For first-time hardware commissioning, FirstMotorTest is the more complete example.
 */

// Import the modern signed-wheel API.
#include <TungLam_OmniMecanum_4WD.h>

// Create the controller that owns the four drive motors.
TungLamDrive4WD robot;

void setup() {
  // Initialize motor GPIO plus the default high-frequency PWM mode.
  robot.begin();

  // Drive only M1 forward at PWM 80; M2/M3/M4 stay at zero.
  robot.setWheels(80, 0, 0, 0);

  // Observe M1 for 800 ms.
  delay(800);

  // Stop all four motors before selecting the next wheel.
  robot.stop();

  // Pause so individual tests are visually distinct.
  delay(500);

  // Drive only M2 forward at PWM 80.
  robot.setWheels(0, 80, 0, 0);

  // Observe M2 for 800 ms.
  delay(800);

  // Stop all four motors.
  robot.stop();

  // Pause before M3.
  delay(500);

  // Drive only M3 forward at PWM 80.
  robot.setWheels(0, 0, 80, 0);

  // Observe M3 for 800 ms.
  delay(800);

  // Stop all four motors.
  robot.stop();

  // Pause before M4.
  delay(500);

  // Drive only M4 forward at PWM 80.
  robot.setWheels(0, 0, 0, 80);

  // Observe M4 for 800 ms.
  delay(800);

  // Leave the drive system stopped after commissioning.
  robot.stop();
}

void loop() {
  // Preserve the normal modern service call pattern from the original example.
  // With no ABS active it has no motor-output effect, but it keeps software state synchronized.
  robot.update();
}
