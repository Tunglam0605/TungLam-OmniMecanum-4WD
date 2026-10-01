/**
 * @file ActiveBrake.ino
 * @brief Demonstrates legacy V5 active reverse braking with the current library.
 *
 * Hardware:
 * - Arduino Mega 2560
 * - 2x L298N
 * - 4 DC motors
 *
 * What to observe:
 * 1. The robot drives forward at PWM 150.
 * 2. After 1.5 s, ABS applies reverse torque at PWM 100.
 * 3. Timer3 automatically terminates the brake pulse.
 * 4. The old V5 API does not require robot.update().
 *
 * Safety:
 * - Lift the robot for the first test.
 * - Start with conservative ABS duty values.
 */

// Import the original V5-compatible header so old projects can keep the same include.
#include <TungLam_Control_MotorV5.h>

// Create one legacy-compatible controller for the four-wheel drive hardware.
TungLam_Control_MotorV5 robot;

void setup() {
  // Configure Timer3/Timer4 in the legacy high-frequency PWM mode (~7.8125 kHz).
  robot.Mode1();

  // Configure the six ABS pulse durations in milliseconds.
  // The ranges are: <500, <1000, <1500, <2000, <3000, and >=3000 ms of prior motion.
  robot.setTimABS(45, 65, 70, 75, 80, 85);

  // Start from a known safe state with all drive PWM and direction outputs inactive.
  robot.STOP();
}

void loop() {
  // Command all four wheels forward with a common PWM duty of 150.
  robot.moveForward(150);

  // Keep driving forward long enough to enter the 1500..1999 ms ABS timing range.
  delay(1500);

  // Apply active reverse braking with the exact user-selected reverse PWM of 100.
  // The function returns immediately; Timer3 overflow ISR owns the brake cutoff.
  robot.ABS(100);

  // Demonstration pause only.
  // No robot.update() call is required by TungLam_Control_MotorV5.
  delay(2000);
}
