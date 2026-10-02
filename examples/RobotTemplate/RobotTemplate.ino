/**
 * @file RobotTemplate.ino
 * @brief Template đế robot tổng quát: copy sketch rồi chỉ điền phần input/cơ cấu.
 *
 * Kiến trúc:
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
  // TODO: chỉ cần đổi driveCommand từ nguồn input của project.
  //
  // Ví dụ:
  // driveCommand = DriveCommand::Forward;
  //
  // Có thể lấy lệnh từ PS2, nút nhấn, Serial, ROS2, sensor hoặc state machine.
}

bool motionAllowed() {
  // TODO: thêm E-stop, bumper, timeout, interlock... nếu project có.
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
  // TODO: điền cơ cấu riêng của robot:
  // - nâng/hạ
  // - servo
  // - gripper
  // - bắn bóng
  // - relay...
}

void emergencyStop() {
  robot.stop();

  // TODO: tắt thêm các actuator khác nếu có.
}
