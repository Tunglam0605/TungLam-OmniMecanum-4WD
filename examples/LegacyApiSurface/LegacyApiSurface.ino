#include <TungLam_Control_MotorV5.h>

// Compile-time regression sketch for the complete original V5 public API.
// This example is intentionally conservative at runtime; exerciseLegacyApi()
// exists so CI compiles every legacy symbol and catches accidental API removal.

TungLam_Control_MotorV5 robot;

void exerciseLegacyApi() {
  robot.Mode0();
  robot.Mode1();

  robot.Init_Timer1(0, 0);
  robot.Init_Timer2(0, 0);

  robot.STOP();
  robot.moveForward(100);
  robot.moveBackward(100);
  robot.Forward_Right(100);
  robot.Backward_Right(100);
  robot.moveRight(100);
  robot.moveLeft(100);
  robot.moveLeftSide(100);
  robot.moveRightSide(100);
  robot.Forward_Left(100);
  robot.Backward_Left(100);

  robot.Dir(1, false);
  robot.Dir(2, true);
  robot.Dir(3, false);
  robot.Dir(4, true);

  robot.Tien(100, 100, 100, 100);
  robot.Lui(100, 100, 100, 100);
  robot.Trai(100, 100, 100, 100);
  robot.Phai(100, 100, 100, 100);
  robot.T_Trai(100, 100, 100, 100);
  robot.T_Phai(100, 100, 100, 100);
  robot.L_Trai(100, 100, 100, 100);
  robot.L_Phai(100, 100, 100, 100);
  robot.N_Trai(100, 100, 100, 100);
  robot.N_Phai(100, 100, 100, 100);

  robot.setTimABS(45, 65, 70, 75, 80, 85);
  robot.ABS(100);
}

void setup() {
  robot.Mode1();
  robot.STOP();
}

void loop() {
  // Keep the runtime example safe; CI still compiles exerciseLegacyApi().
}
