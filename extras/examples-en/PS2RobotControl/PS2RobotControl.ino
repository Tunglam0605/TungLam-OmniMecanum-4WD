/**
 * @file PS2RobotControl.ino
 * @brief Recommended robot + PS2 template for RoboBall/Mecanum projects.
 *
 * Left stick: forward/backward/strafe.
 * Right stick: rotate left/right with higher priority than the left stick.
 *
 * onCrossPressed(), onCirclePressed(), onR1Held()... are ready-made hooks.
 * Fill only the TODO mechanism functions.
 */

#include <TungLam_PS2.h>
#include <TungLam_OmniMecanum_4WD.h>

TungLamPS2 ps2;
TungLamDrive4WD robot;

constexpr uint8_t PS2_CS_PIN = 53;
constexpr uint8_t MOVE_SPEED = 180;
constexpr uint8_t TURN_SPEED = 155;

void setupRobot();
void setupController();
void handleDrive();
void handleButtons();
void safeStop();

void onCrossPressed();
void onCirclePressed();
void onR1Held();
void onR1Released();

void setup() {
  setupRobot();
  setupController();
}

void loop() {
  ps2.update();
  robot.update();

  if (!ps2.connected()) {
    safeStop();
    return;
  }

  handleDrive();
  handleButtons();
}

void setupRobot() {
  robot.begin(TungLamPwmMode::High7k8Hz);
  robot.setChassis(TungLamChassis::MecanumX);
  robot.stop();
}

void setupController() {
  ps2.begin(PS2_CS_PIN);
  ps2.setPollRate(PS2PollRate::Hz50);
}

void handleDrive() {
  // PRIORITY 1: the right stick owns rotation when active.
  switch (ps2.rightDirection()) {
    case PS2StickDirection::Left:
      robot.rotateLeft(TURN_SPEED);
      return;

    case PS2StickDirection::Right:
      robot.rotateRight(TURN_SPEED);
      return;

    default:
      break;
  }

  // PRIORITY 2: the left stick controls translation.
  switch (ps2.leftDirection()) {
    case PS2StickDirection::Up:
      robot.forward(MOVE_SPEED);
      break;

    case PS2StickDirection::Down:
      robot.backward(MOVE_SPEED);
      break;

    case PS2StickDirection::Left:
      robot.strafeLeft(MOVE_SPEED);
      break;

    case PS2StickDirection::Right:
      robot.strafeRight(MOVE_SPEED);
      break;

    case PS2StickDirection::Center:
    case PS2StickDirection::Unknown:
    default:
      robot.stop();
      break;
  }
}

void handleButtons() {
  if (ps2.pressed(PS2Button::Cross)) {
    onCrossPressed();
  }

  if (ps2.pressed(PS2Button::Circle)) {
    onCirclePressed();
  }

  if (ps2.button(PS2Button::R1)) {
    onR1Held();
  }

  if (ps2.released(PS2Button::R1)) {
    onR1Released();
  }

  // Add L1/L2/R2/START/SELECT... with the same pattern when needed.
}

void safeStop() {
  robot.stop();

  // TODO: disable other hazardous mechanisms if present.
}

// ============================================================================
// USER FUNCTIONS - fill project mechanism behavior here.
// ============================================================================

void onCrossPressed() {
  // TODO: e.g. toggle gripper.
}

void onCirclePressed() {
  // TODO: e.g. switch mode.
}

void onR1Held() {
  // TODO: e.g. lift while R1 is held.
}

void onR1Released() {
  // TODO: e.g. stop the lift when R1 is released.
}
