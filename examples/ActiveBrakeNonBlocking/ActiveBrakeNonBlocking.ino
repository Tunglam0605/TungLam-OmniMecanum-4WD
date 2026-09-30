#include <TungLam_OmniMecanum_4WD.h>

TungLamDrive4WD robot;

enum class DemoState : uint8_t {
  Driving,
  Braking,
  Waiting
};

DemoState state = DemoState::Driving;
uint32_t stateStart = 0;

void setup() {
  robot.begin();
  robot.setChassis(TungLamChassis::MecanumX);

  // Same configurable time table as the legacy V5 library.
  robot.setTimABS(45, 65, 70, 75, 80, 85);

  robot.forward(150);
  stateStart = millis();
}

void loop() {
  // Required service call for the non-blocking brake state machine.
  robot.update();

  const uint32_t now = millis();

  if (state == DemoState::Driving && now - stateStart >= 1500UL) {
    // User directly chooses the reverse-brake strength, exactly like V5.
    // This starts the brake pulse and returns immediately (non-blocking).
    robot.ABS(150);
    state = DemoState::Braking;
  }

  if (state == DemoState::Braking && !robot.isBraking()) {
    state = DemoState::Waiting;
    stateStart = now;
  }

  if (state == DemoState::Waiting && now - stateStart >= 2000UL) {
    robot.forward(150);
    state = DemoState::Driving;
    stateStart = now;
  }
}
