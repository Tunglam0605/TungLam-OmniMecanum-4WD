#include <TungLam_Control_MotorV5.h>

TungLam_Control_MotorV5 robot;

void setup() {
  // Legacy V5 API is intentionally supported by the new library.
  robot.Mode1();
  robot.STOP();
}

void loop() {
  robot.moveForward(140);
  delay(1200);

  robot.STOP();
  delay(500);

  robot.moveRightSide(140);
  delay(1200);

  robot.STOP();
  delay(500);

  robot.moveLeft(120);
  delay(800);

  robot.STOP();
  delay(1000);
}
