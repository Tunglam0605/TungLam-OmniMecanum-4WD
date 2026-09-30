/**
 * @file BasicMotion.ino
 * @brief Beginner example using the original V5 movement-function names.
 *
 * Sequence:
 * 1. Move forward.
 * 2. Stop.
 * 3. Strafe right.
 * 4. Stop.
 * 5. Rotate left.
 * 6. Stop and repeat.
 *
 * The delay() calls are used only to make each movement easy to observe.
 */

// Import the legacy-compatible V5 header.
#include <TungLam_Control_MotorV5.h>

// Create one controller for all four drive motors.
TungLam_Control_MotorV5 robot;

void setup() {
  // Configure the historical Mode1 hardware PWM frequency (~7.8125 kHz).
  robot.Mode1();

  // Ensure the chassis starts with zero PWM and a cleared movement state.
  robot.STOP();
}

void loop() {
  // Drive all four wheels in the logical forward direction at PWM 140.
  robot.moveForward(140);

  // Hold the forward command for 1.2 s so the motion is visible.
  delay(1200);

  // Stop all four drive motors.
  robot.STOP();

  // Pause before the next movement.
  delay(500);

  // Translate the Mecanum chassis to the right at PWM 140.
  robot.moveRightSide(140);

  // Hold the right-strafe command for 1.2 s.
  delay(1200);

  // Stop before changing to a rotation command.
  robot.STOP();

  // Pause between demonstrations.
  delay(500);

  // Rotate the chassis to the left / counter-clockwise at PWM 120.
  robot.moveLeft(120);

  // Hold the rotation for 0.8 s.
  delay(800);

  // Return the drive system to the stopped state.
  robot.STOP();

  // Wait 1 s before repeating the complete sequence.
  delay(1000);
}
