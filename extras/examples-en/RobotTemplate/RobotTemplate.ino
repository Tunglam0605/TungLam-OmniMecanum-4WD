/**
 * @file RobotTemplate.ino
 * @brief General robot-base template: copy it and fill only project input/mechanism hooks.
 *
 * Architecture:
 *   readInputs() -> driveCommand -> handleDrive() -> TungLamDrive4WD
 *                      |
 *                      +-> handleMechanisms()
 *   motionAllowed() -> emergencyStop()
 */

#include <TungLam_OmniMecanum_4WD.h>

TungLamDrive4WD robot;

enum class DriveCommand : uint8_t {
  Stop,
  Forward,
  Backward,
  Left,
  Right,
  RotateLeft,
  RotateRight
};

DriveCommand driveCommand = DriveCommand::Stop;

constexpr uint8_t MOVE_SPEED = 180;
constexpr uint8_t TURN_SPEED = 150;

void setupRobot();
void readInputs();
bool motionAllowed();
void handleDrive();
void handleMechanisms();
void emergencyStop();

void setup() {
  setupRobot();
}

void loop() {
  robot.update();

  readInputs();

  if (!motionAllowed()) {
    emergencyStop();
    return;
  }

  handleDrive();
  handleMechanisms();
}

void setupRobot() {
  robot.begin(TungLamPwmMode::High7k8Hz);
  robot.setChassis(TungLamChassis::MecanumX);
  robot.stop();
}

void readInputs() {
  // TODO: set driveCommand from your project's input source.
  //
  // Example:
  // driveCommand = DriveCommand::Forward;
  //
  // Input may come from PS2, buttons, Serial, ROS2, sensors or a state machine.
}

bool motionAllowed() {
  // TODO: add E-stop, bumper, timeout or interlocks if needed.
  return true;
}

void handleDrive() {
  switch (driveCommand) {
    case DriveCommand::Forward:     robot.forward(MOVE_SPEED); break;
    case DriveCommand::Backward:    robot.backward(MOVE_SPEED); break;
    case DriveCommand::Left:        robot.strafeLeft(MOVE_SPEED); break;
    case DriveCommand::Right:       robot.strafeRight(MOVE_SPEED); break;
    case DriveCommand::RotateLeft:  robot.rotateLeft(TURN_SPEED); break;
    case DriveCommand::RotateRight: robot.rotateRight(TURN_SPEED); break;
    case DriveCommand::Stop:
    default:
      robot.stop();
      break;
  }
}

void handleMechanisms() {
  // TODO: implement project-specific mechanisms.
}

void emergencyStop() {
  robot.stop();

  // TODO: disable other actuators if required.
}
