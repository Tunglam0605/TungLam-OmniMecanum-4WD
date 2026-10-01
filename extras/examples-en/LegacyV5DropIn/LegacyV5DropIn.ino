/**
 * @file LegacyV5DropIn.ino
 * @brief Demonstrates zero-source-change style migration from the original V5 library.
 *
 * The include name, class name, movement calls, ABS call, and setTimABS call are
 * intentionally the same style used by historical V5 sketches.
 */

// Include the compatibility header with the original V5 filename.
#include <TungLam_Control_MotorV5.h>

// Instantiate the original V5 class name.
TungLam_Control_MotorV5 robot;

void setup() {
  // Configure the same Mode1 hardware PWM profile used by old projects.
  robot.Mode1();

  // Configure the original six ABS timing intervals.
  robot.setTimABS(45, 65, 70, 75, 80, 85);
}

void loop() {
  // Drive forward at PWM 160 using the original V5 function.
  robot.moveForward(160);

  // Keep moving forward for 1.2 s.
  delay(1200);

  // Apply active reverse braking at exact PWM 200.
  // The implementation is now non-blocking and Timer3 stops the pulse automatically.
  robot.ABS(200);

  // Application execution continues immediately after ABS() returns.
  // This delay is only part of the demonstration; it is not needed by ABS itself.
  delay(500);

  // Command the original V5 right-strafe movement at PWM 140.
  robot.moveRightSide(140);

  // Hold the strafe for one second.
  delay(1000);

  // Brake the right-strafe motion with reverse PWM 180.
  robot.ABS(180);

  // Pause before repeating the example.
  delay(1000);
}
