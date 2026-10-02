/**
 * @file MecanumDrive.ino
 * @brief Minimal modern Mecanum-X vector-drive example.
 *
 * Coordinate convention:
 * - +vx = forward
 * - +vy = strafe left
 * - +wz = rotate counter-clockwise/left
 *
 * The mixer is aligned with the original TungLam V5 movement basis.
 */

// Import the modern 4WD API.
#include <TungLam_OmniMecanum_4WD.h>

// Create one modern controller for the four-motor drive base.
TungLamDrive4WD robot;

void setup() {
  // Configure D5..D8 PWM and D30..D37 direction outputs.
  // High7k8Hz selects the recommended ~7.8125 kHz Timer3/Timer4 PWM mode.
  robot.begin(TungLamPwmMode::High7k8Hz);

  // Select the Mecanum-X kinematic mixer for drive(vx, vy, wz).
  robot.setChassis(TungLamChassis::MecanumX);

  // If one physical motor is mounted/wired opposite to the logical convention,
  // invert only that wheel instead of editing the Mecanum equations.
  // Example: robot.setMotorInverted(3, true);
}

void loop() {
  // Optional software-state synchronization.
  // Physical ABS cutoff does not depend on this call, but keeping update() is harmless.
  robot.update();

  // Command pure forward motion in normalized command units:
  // +vx = forward, +vy = left, +wz = counter-clockwise.
  // Here vx=150, while vy=0 and wz=0 disable lateral/yaw motion.
  robot.drive(150, 0, 0);

  // drive() uses normalized -255..255 style command units.
  // For physical m/s and rad/s commands, configure TungLamDriveConfig and use
  // driveVelocity(vxMps, vyMps, wzRadps).
}
