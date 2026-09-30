#include <TungLam_OmniMecanum_4WD.h>

// The new umbrella header still exposes the complete legacy class/API.
TungLam_Control_MotorV5 robot;

void setup() {
  robot.Mode1();
  robot.setTimABS(45, 65, 70, 75, 80, 85);
}

void loop() {
  robot.Tien(150, 150, 150, 150);
  delay(1000);

  robot.ABS(210);  // Same function and meaning as the old library.

  delay(1000);

  robot.N_Phai(140, 140, 140, 140);
  delay(800);
  robot.ABS(190);

  delay(1000);
}
