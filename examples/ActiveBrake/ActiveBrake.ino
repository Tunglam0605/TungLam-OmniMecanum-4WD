#include <TungLam_Control_MotorV5.h>

TungLam_Control_MotorV5 robot;

void setup() {
  robot.Mode1();

  // Legacy V5 reverse-brake duration table:
  // <500, <1000, <1500, <2000, <3000, >=3000 ms of prior motion.
  robot.setTimABS(45, 65, 70, 75, 80, 85);

  robot.STOP();
}

void loop() {
  robot.moveForward(150);
  delay(1500);

  // Same V5 API and user-selected brake strength.
  // In the new library this returns immediately; Timer3 stops the pulse.
  robot.ABS(100);

  // No robot.update() is needed for the legacy class.
  delay(2000);
}
