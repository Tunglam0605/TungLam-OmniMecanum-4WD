/**
 * @file StudentQuickStart.ino
 * @brief Recommended beginner example: configure the robot once and command SI velocity.
 *
 * Remember only:
 * 1. robot.begin()
 * 2. robot.setChassis(...)
 * 3. robot.setDriveConfig(...)
 * 4. robot.enableSmartSafety(...)
 * 5. robot.update() in loop()
 * 6. robot.driveVelocity(vx, vy, wz)
 *
 * Inverse kinematics, wheel-speed limiting, RPM-to-PWM feed-forward, command
 * watchdog and velocity smoothing are handled internally by the library.
 */

#include <TungLam_OmniMecanum_4WD.h>

TungLamDrive4WD robot;

TungLamDriveConfig model(
    12.0f,   // motor nominal voltage [V]
    300.0f,  // gearbox output RPM
    12.0f,   // motor supply voltage [V]
    0.050f,  // wheel radius [m]
    0.320f,  // wheelbase [m]
    0.280f,  // track width [m]
    0.85f    // empirical calibration scale
);

void setup() {
  robot.begin();

  robot.setChassis(TungLamChassis::MecanumX);

  robot.setDriveConfig(model);

  // 500 ms command watchdog, 1.0 m/s^2 linear slew, 2.0 rad/s^2 yaw slew.
  robot.enableSmartSafety(500, 1.0f, 2.0f);
}

void loop() {
  robot.update();

  // +vx = forward, +vy = left, +wz = CCW.
  robot.driveVelocity(0.30f, 0.00f, 0.00f);

  // The library handles wheel kinematics, speed saturation and PWM conversion.
}
