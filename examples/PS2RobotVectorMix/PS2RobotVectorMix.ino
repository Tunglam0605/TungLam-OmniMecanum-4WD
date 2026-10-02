/**
 * @file PS2RobotVectorMix.ino
 * @brief Ví dụ NÂNG CAO: vừa tịnh tiến vừa quay bằng vector vx/vy/wz.
 *
 * Đây KHÔNG phải example khuyến nghị mặc định cho phong cách RoboBall cũ.
 * Dùng file này khi muốn:
 *
 *   joystick trái  -> vx / vy
 *   joystick phải  -> wz
 *
 * và cho phép cả hai tác động CÙNG LÚC.
 *
 * Ví dụ:
 * - giữ LEFT UP;
 * - đồng thời gạt RIGHT RIGHT;
 * -> robot vừa tiến vừa quay phải theo mixer Mecanum.
 *
 * Nếu muốn joystick phải GHI ĐÈ joystick trái như các project cũ,
 * hãy dùng example PS2RobotControl.
 */

#include <TungLam_PS2.h>
#include <TungLam_OmniMecanum_4WD.h>

#define PS2_ROBOT_DEBUG 0

TungLamPS2 ps2;
TungLamDrive4WD robot;

constexpr uint8_t PS2_CS_PIN = 53;
constexpr int16_t MOVE_DUTY = 180;
constexpr int16_t TURN_DUTY = 150;

void setup() {
#if PS2_ROBOT_DEBUG
  Serial.begin(115200);
#endif

  robot.begin();
  robot.setChassis(TungLamChassis::MecanumX);

  ps2.begin(PS2_CS_PIN);
  ps2.setPollRate(PS2PollRate::Hz50);
}

void loop() {
  ps2.update();
  robot.update();

#if PS2_ROBOT_DEBUG
  ps2.debug(Serial);
#endif

  if (!ps2.connected()) {
    robot.stop();
    return;
  }

  int16_t vx = 0;
  int16_t vy = 0;
  int16_t wz = 0;

  // Joystick trái -> tịnh tiến.
  switch (ps2.leftDirection()) {
    case PS2StickDirection::Up:
      vx = MOVE_DUTY;
      break;

    case PS2StickDirection::Down:
      vx = -MOVE_DUTY;
      break;

    case PS2StickDirection::Left:
      vy = MOVE_DUTY;
      break;

    case PS2StickDirection::Right:
      vy = -MOVE_DUTY;
      break;

    default:
      break;
  }

  // Joystick phải -> quay.
  switch (ps2.rightDirection()) {
    case PS2StickDirection::Left:
      wz = TURN_DUTY;
      break;

    case PS2StickDirection::Right:
      wz = -TURN_DUTY;
      break;

    default:
      break;
  }

  if (vx == 0 && vy == 0 && wz == 0) {
    robot.stop();
  } else {
    robot.drive(vx, vy, wz);
  }
}
