#include <TungLam_OmniMecanum_4WD.h>

TungLamDrive4WD robot;

void setup() {
  robot.begin();

  // Useful for commissioning: verify one motor at a time.
  robot.setWheels(80, 0, 0, 0);
  delay(800);
  robot.stop();

  delay(500);

  robot.setWheels(0, 80, 0, 0);
  delay(800);
  robot.stop();

  delay(500);

  robot.setWheels(0, 0, 80, 0);
  delay(800);
  robot.stop();

  delay(500);

  robot.setWheels(0, 0, 0, 80);
  delay(800);
  robot.stop();
}

void loop() {
  robot.update();
}
