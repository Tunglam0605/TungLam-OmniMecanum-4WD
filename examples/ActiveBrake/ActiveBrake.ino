#include <TungLam_Control_MotorV5.h>

TungLam_Control_MotorV5 robot;

void setup() {
  robot.Mode1();

  // Reverse-brake duration table in milliseconds:
  // <500, <1000, <1500, <2000, <3000, >3000 ms of motion.
  robot.setTimABS(45, 65, 70, 75, 80, 85);

  robot.STOP();
}

void loop() {
  robot.moveForward(150);
  delay(1500);

  // Legacy V5 active reverse braking.
  robot.ABS(100);

  robot.STOP();
  delay(2000);
}
