#include <TungLam_Control_MotorV5.h>

TungLam_Control_MotorV5 robot;

void setup() {
  // This is intentionally the same public API as the original V5 library.
  robot.Mode1();
  robot.setTimABS(45, 65, 70, 75, 80, 85);
}

void loop() {
  robot.moveForward(160);
  delay(1200);

  // Non-blocking internally in the new library.
  // Old sketches do NOT need robot.update().
  robot.ABS(200);

  // The CPU is immediately free to continue application code here.
  delay(500);

  robot.moveRightSide(140);
  delay(1000);
  robot.ABS(180);

  delay(1000);
}
