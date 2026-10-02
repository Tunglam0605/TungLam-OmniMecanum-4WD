/**
 * @file PS2RobotControl.ino
 * @brief Template robot + PS2 khuyến nghị cho RoboBall/Mecanum.
 *
 * JOY trái : tiến/lùi/ngang.
 * JOY phải : quay trái/phải và có priority cao hơn JOY trái.
 *
 * Các hàm onCrossPressed(), onCirclePressed(), onR1Held()... đã viết sẵn.
 * Người dùng chỉ cần điền chức năng cơ cấu vào phần TODO.
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
  // PRIORITY 1: joystick phải chiếm quyền khi có lệnh quay.
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

  // PRIORITY 2: joystick trái điều khiển tịnh tiến.
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

  // Thêm L1/L2/R2/START/SELECT... theo cùng pattern khi cần.
}

void safeStop() {
  robot.stop();

  // TODO: tắt thêm cơ cấu nguy hiểm nếu project có.
}

// ============================================================================
// USER FUNCTIONS - điền chức năng cơ cấu tại đây.
// ============================================================================

void onCrossPressed() {
  // TODO: ví dụ đóng/mở gripper.
}

void onCirclePressed() {
  // TODO: ví dụ đổi mode.
}

void onR1Held() {
  // TODO: ví dụ nâng cơ cấu trong lúc giữ R1.
}

void onR1Released() {
  // TODO: ví dụ dừng cơ cấu nâng khi nhả R1.
}
