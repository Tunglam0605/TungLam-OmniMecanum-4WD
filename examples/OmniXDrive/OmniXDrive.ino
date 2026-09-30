/**
 * @file OmniXDrive.ino
 * @brief Minimal modern four-wheel Omni X-drive example.
 *
 * Coordinate convention:
 * - +vx = forward
 * - +vy = right
 * - +wz = clockwise/right rotation
 *
 * Omni mechanical layouts vary, so verify M1..M4 using FirstMotorTest before
 * relying on combined X-drive motion.
 */

// Import the modern 4WD API.
#include <TungLam_OmniMecanum_4WD.h>

// Create one modern controller for the four-wheel drive hardware.
TungLamDrive4WD robot;

void setup() {
  // Initialize the motor pins and the recommended ~7.8125 kHz PWM mode.
  robot.begin(TungLamPwmMode::High7k8Hz);

  // Select the Omni-X mixer used by drive(vx, vy, wz).
  robot.setChassis(TungLamChassis::OmniX);
}

void loop() {
  // Optional high-level state synchronization after a hardware-timed brake pulse.
  robot.update();

  // Command combined forward and right translation with no rotation:
  // vx=140 -> forward component
  // vy=80  -> right component
  // wz=0   -> no rotational component
  robot.drive(140, 80, 0);

  // The mixer normalizes all four wheel outputs proportionally if required.
}
