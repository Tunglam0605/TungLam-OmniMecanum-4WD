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

  // Conservative starting point. Tune on your real robot.
  robot.setAutoBrakeDuty(60, 130, 55);
  robot.setBrakeTimings(35, 45, 55, 60, 65, 70);

  robot.forward(150);
  stateStart = millis();
}

void loop() {
  // Required service call for the non-blocking brake state machine.
  robot.update();

  const uint32_t now = millis();

  if (state == DemoState::Driving && now - stateStart >= 1500UL) {
    robot.activeBrake();
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
