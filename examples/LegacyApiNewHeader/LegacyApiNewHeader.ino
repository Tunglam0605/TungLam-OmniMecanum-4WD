/**
 * @file LegacyApiNewHeader.ino
 * @brief Shows that the new umbrella header still exposes the complete V5 class.
 *
 * Use this migration style when you want to include the new main header but
 * keep the original TungLam_Control_MotorV5 class and function calls.
 */

// Include the new main header instead of the historical compatibility header.
#include <TungLam_OmniMecanum_4WD.h>

// Instantiate the old V5 class; its public API remains available unchanged.
TungLam_Control_MotorV5 robot;

void setup() {
  // Configure the drive timers in the historical high-frequency Mode1.
  robot.Mode1();

  // Keep the original six-stage ABS timing configuration model.
  robot.setTimABS(45, 65, 70, 75, 80, 85);
}

void loop() {
  // V5 Vietnamese API: Tien() drives forward with independent M1..M4 PWM values.
  robot.Tien(150, 150, 150, 150);

  // Drive forward for one second.
  delay(1000);

  // Apply strong reverse braking at exact PWM 210 using the same old function name.
  robot.ABS(210);

  // Pause after the hardware-timed ABS pulse.
  delay(1000);

  // N_Phai() performs a right strafe using the legacy per-wheel API.
  robot.N_Phai(140, 140, 140, 140);

  // Hold the right-strafe command for 800 ms.
  delay(800);

  // Apply active reverse braking at exact PWM 190.
  robot.ABS(190);

  // Pause before repeating the demonstration.
  delay(1000);
}
