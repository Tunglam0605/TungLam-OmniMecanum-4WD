#include <TungLam_OmniMecanum_4WD.h>

TungLamDrive4WD robot;

void setup() {
  robot.begin(TungLamPwmMode::High7k8Hz);
  robot.setChassis(TungLamChassis::MecanumX);

  // Enable only if a physical motor is mounted opposite to the logical convention.
  // robot.setMotorInverted(3, true);
}

void loop() {
  robot.update();

  // Cartesian command:
  // vx > 0 forward, vy > 0 right, wz > 0 clockwise.
  robot.drive(150, 0, 0);
}
