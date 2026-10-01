/**
 * @file ActiveBrakeNonBlocking.ino
 * @brief Demonstrates hardware-timed ABS with a non-blocking application state machine.
 *
 * This example intentionally avoids delay() inside the motion sequence so the
 * application can continue processing other work while the brake pulse is active.
 *
 * Modern ABS behavior:
 * - ABS(duty) applies reverse torque immediately.
 * - Timer3 overflow ISR stops the physical brake pulse at the deadline.
 * - update() is optional for safety and only synchronizes high-level software state.
 */

// Import the modern 4WD API.
#include <TungLam_OmniMecanum_4WD.h>

// Create the single modern drive controller for this Arduino Mega.
TungLamDrive4WD robot;

// Define the three states used by this demonstration loop.
enum class DemoState : uint8_t {
  Driving,  // Robot is actively driving forward.
  Braking,  // An ABS reverse-torque pulse is currently active.
  Waiting   // Robot is stopped and waiting before the next cycle.
};

// Start the state machine in the driving state.
DemoState state = DemoState::Driving;

// Store the millis() timestamp at which the current state started.
uint32_t stateStart = 0;

void setup() {
  // Initialize motor GPIO plus Timer3/Timer4 using the default high-frequency PWM mode.
  robot.begin();

  // Select the Mecanum mixer that matches the original TungLam V5 movement convention.
  robot.setChassis(TungLamChassis::MecanumX);

  // Configure the same six ABS timing ranges used by the legacy API.
  robot.setTimABS(45, 65, 70, 75, 80, 85);

  // Begin the demonstration by driving forward at PWM 150.
  robot.forward(150);

  // Remember when the forward-driving interval started.
  stateStart = millis();
}

void loop() {
  // Synchronize software state after a Timer3 ISR brake cutoff.
  // The physical ABS pulse is already hardware-timed, so this is not a safety requirement.
  robot.update();

  // Read millis() once so all state comparisons in this loop use the same timestamp.
  const uint32_t now = millis();

  // After 1.5 s of forward motion, start active reverse braking.
  if (state == DemoState::Driving && now - stateStart >= 1500UL) {
    // Use PWM 150 as the exact reverse-brake strength.
    robot.ABS(150);

    // Move the application state machine into the braking state.
    state = DemoState::Braking;
  }

  // Wait until the Timer3 one-shot reports that the ABS pulse has finished.
  if (state == DemoState::Braking && !robot.isBraking()) {
    // Enter the stopped waiting state.
    state = DemoState::Waiting;

    // Record when the waiting interval started.
    stateStart = now;
  }

  // After waiting for 2 s, start the next forward-driving cycle.
  if (state == DemoState::Waiting && now - stateStart >= 2000UL) {
    // Drive forward again at the same PWM.
    robot.forward(150);

    // Return to the driving state.
    state = DemoState::Driving;

    // Restart the driving interval timer.
    stateStart = now;
  }

  // Other non-blocking work such as Serial, sensors, joystick parsing, or ROS
  // communication could be added here without extending the ABS pulse duration.
}
