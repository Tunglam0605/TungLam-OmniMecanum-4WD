#include <TungLam_OmniMecanum_4WD.h>

TungLamDrive4WD robot;

void setup() {
  robot.begin(TungLamPwmMode::High7k8Hz);
  robot.setChassis(TungLamChassis::OmniX);
}

void loop() {
  robot.update();

  // Combined forward + right translation.
  robot.drive(140, 80, 0);
}
