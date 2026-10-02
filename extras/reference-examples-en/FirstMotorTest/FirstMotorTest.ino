/**
 * @file FirstMotorTest.ino
 * @brief Safe first-power-on commissioning test for M1, M2, M3, and M4.
 *
 * IMPORTANT BEFORE UPLOAD:
 * - Lift the robot so all four wheels are off the floor.
 * - Check the README wiring table before applying motor power.
 * - Start with TEST_PWM around 60..80.
 * - Verify that the physical wheel positions are:
 *     M1 = front-left
 *     M2 = rear-left
 *     M3 = rear-right
 *     M4 = front-right
 *
 * The sketch tests each wheel forward and reverse one at a time. It is intended
 * to find wiring, polarity, and wheel-numbering mistakes before chassis motion.
 */

// Import the modern API because direct signed per-wheel commands are ideal for commissioning.
#include <TungLam_OmniMecanum_4WD.h>

// Create one controller for the single four-motor hardware resource.
TungLamDrive4WD robot;

// Use a low PWM so the first electrical/mechanical test is gentle.
constexpr uint8_t TEST_PWM = 80;

// Run each forward/reverse wheel test for 900 ms.
constexpr uint16_t RUN_MS = 900;

// Pause 500 ms after each direction so the result is easy to observe.
constexpr uint16_t PAUSE_MS = 500;

/**
 * @brief Drive only one selected wheel while commanding the other three to zero.
 * @param wheel Logical motor number 1..4.
 * @param duty Signed PWM: positive=logical forward, negative=logical reverse.
 */
void driveOnly(uint8_t wheel, int16_t duty) {
  // Select which element of the four-wheel vector receives the test duty.
  switch (wheel) {
    case 1:
      // M1 only; M2/M3/M4 remain stopped.
      robot.setWheels(duty, 0, 0, 0);
      break;

    case 2:
      // M2 only; M1/M3/M4 remain stopped.
      robot.setWheels(0, duty, 0, 0);
      break;

    case 3:
      // M3 only; M1/M2/M4 remain stopped.
      robot.setWheels(0, 0, duty, 0);
      break;

    case 4:
      // M4 only; M1/M2/M3 remain stopped.
      robot.setWheels(0, 0, 0, duty);
      break;

    default:
      // Invalid motor number: fail safely by stopping the drive system.
      robot.stop();
      break;
  }
}

/**
 * @brief Test one wheel in logical forward and reverse directions.
 * @param wheel Logical wheel number 1..4.
 */
void testWheel(uint8_t wheel) {
  // Print the motor number before the forward test.
  Serial.print(F("M"));
  Serial.print(wheel);
  Serial.println(F(" forward"));

  // Apply positive TEST_PWM only to this wheel.
  driveOnly(wheel, TEST_PWM);

  // Keep the wheel running long enough for visual inspection.
  delay(RUN_MS);

  // Remove motor drive after the forward test.
  robot.stop();

  // Pause before reversing the same wheel.
  delay(PAUSE_MS);

  // Print the motor number before the reverse test.
  Serial.print(F("M"));
  Serial.print(wheel);
  Serial.println(F(" reverse"));

  // Apply the same PWM magnitude with a negative sign for logical reverse.
  driveOnly(wheel, -(int16_t)TEST_PWM);

  // Keep reverse motion active for the same observation time.
  delay(RUN_MS);

  // Stop the wheel after the reverse test.
  robot.stop();

  // Pause before moving to the next motor.
  delay(PAUSE_MS);
}

void setup() {
  // Open Serial Monitor output at 115200 baud for test progress messages.
  Serial.begin(115200);

  // Configure Timer3/Timer4 for the recommended ~7.8125 kHz drive PWM.
  robot.begin(TungLamPwmMode::High7k8Hz);

  // Explicitly enforce a stopped state before the countdown begins.
  robot.stop();

  // Identify the sketch in Serial Monitor.
  Serial.println(F("TungLam 4WD - First Motor Test"));

  // Remind the operator that the robot should be lifted.
  Serial.println(F("Lift the robot so all wheels are free."));

  // Announce the startup delay before any wheel moves.
  Serial.println(F("Starting in 3 seconds..."));

  // Give the operator three seconds to react before motor movement begins.
  delay(3000);

  // Test M1, then M2, then M3, then M4.
  for (uint8_t wheel = 1; wheel <= 4; ++wheel) {
    // Run both forward and reverse checks for the selected wheel.
    testWheel(wheel);
  }

  // Leave all motors stopped after the one-shot commissioning sequence.
  robot.stop();

  // Tell the operator the test completed successfully from the sketch's perspective.
  Serial.println(F("Motor test complete."));
}

void loop() {
  // Intentionally empty: this commissioning test runs once from setup().
}
