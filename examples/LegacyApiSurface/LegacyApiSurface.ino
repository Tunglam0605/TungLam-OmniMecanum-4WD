/**
 * @file LegacyApiSurface.ino
 * @brief Compile-time regression coverage for the complete original V5 public API.
 *
 * This is primarily a CI/API compatibility example, not a motion demo.
 * exerciseLegacyApi() is intentionally NOT called from loop(), because calling
 * every movement function back-to-back would not be a useful physical test.
 */

// Include the exact header name used by original V5 projects.
#include <TungLam_Control_MotorV5.h>

// Instantiate the legacy-compatible class exactly as old sketches do.
TungLam_Control_MotorV5 robot;

/**
 * @brief Reference every preserved V5 public symbol so CI catches API regressions.
 *
 * The compiler must successfully resolve every call below. If a future change
 * removes or changes a legacy function signature, this example should fail CI.
 */
void exerciseLegacyApi() {
  // Configure low-frequency legacy motor PWM.
  robot.Mode0();

  // Configure high-frequency legacy motor PWM.
  robot.Mode1();

  // Exercise historical auxiliary Timer1 PWM initialization.
  robot.Init_Timer1(0, 0);

  // Exercise historical auxiliary Timer2 PWM initialization.
  robot.Init_Timer2(0, 0);

  // Stop all four drive motors.
  robot.STOP();

  // Drive straight forward using one common duty.
  robot.moveForward(100);

  // Drive straight backward using one common duty.
  robot.moveBackward(100);

  // Drive diagonally forward-right.
  robot.Forward_Right(100);

  // Drive diagonally backward-right.
  robot.Backward_Right(100);

  // Rotate right / clockwise.
  robot.moveRight(100);

  // Rotate left / counter-clockwise.
  robot.moveLeft(100);

  // Strafe left.
  robot.moveLeftSide(100);

  // Strafe right.
  robot.moveRightSide(100);

  // Drive diagonally forward-left.
  robot.Forward_Left(100);

  // Drive diagonally backward-left.
  robot.Backward_Left(100);

  // Exercise direct wheel-1 direction selection with the old boolean convention.
  robot.Dir(1, false);

  // Exercise direct wheel-2 direction selection.
  robot.Dir(2, true);

  // Exercise direct wheel-3 direction selection.
  robot.Dir(3, false);

  // Exercise direct wheel-4 direction selection.
  robot.Dir(4, true);

  // Per-wheel-duty forward command.
  robot.Tien(100, 100, 100, 100);

  // Per-wheel-duty backward command.
  robot.Lui(100, 100, 100, 100);

  // Per-wheel-duty left rotation.
  robot.Trai(100, 100, 100, 100);

  // Per-wheel-duty right rotation.
  robot.Phai(100, 100, 100, 100);

  // Per-wheel-duty forward-left diagonal.
  robot.T_Trai(100, 100, 100, 100);

  // Per-wheel-duty forward-right diagonal.
  robot.T_Phai(100, 100, 100, 100);

  // Per-wheel-duty backward-left diagonal.
  robot.L_Trai(100, 100, 100, 100);

  // Per-wheel-duty backward-right diagonal.
  robot.L_Phai(100, 100, 100, 100);

  // Per-wheel-duty left strafe.
  robot.N_Trai(100, 100, 100, 100);

  // Per-wheel-duty right strafe.
  robot.N_Phai(100, 100, 100, 100);

  // Configure all six legacy ABS timing ranges.
  robot.setTimABS(45, 65, 70, 75, 80, 85);

  // Reference the legacy ABS function with a user-selected brake duty.
  robot.ABS(100);
}

void setup() {
  // Initialize the real hardware in the normal high-frequency legacy mode.
  robot.Mode1();

  // Leave the robot safely stopped because exerciseLegacyApi() is not executed.
  robot.STOP();
}

void loop() {
  // Intentionally empty: this sketch exists to compile-check the API surface.
}
