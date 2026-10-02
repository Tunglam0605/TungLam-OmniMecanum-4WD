/**
 * @file PS2RobotControl.ino
 * @brief Ví dụ KHUYẾN NGHỊ điều khiển đế robot bằng TungLam_PS2.
 *
 * PHONG CÁCH ĐIỀU KHIỂN
 * ==========================================================================
 * Ví dụ này bám theo cách điều khiển đã dùng trong các project RoboBall/V5:
 *
 *   JOYSTICK TRÁI = chuyển động chính
 *     UP    -> tiến
 *     DOWN  -> lùi
 *     LEFT  -> ngang trái
 *     RIGHT -> ngang phải
 *
 *   JOYSTICK PHẢI = quay, có ƯU TIÊN CAO HƠN joystick trái
 *     LEFT  -> xoay trái / CCW
 *     RIGHT -> xoay phải / CW
 *
 * Ví dụ:
 *
 *   1. Đang giữ joystick trái UP -> robot tiến.
 *   2. Vẫn giữ UP, gạt joystick phải RIGHT -> robot xoay phải.
 *   3. Thả joystick phải về CENTER -> robot tự tiếp tục tiến,
 *      vì joystick trái vẫn đang giữ UP.
 *
 * Điểm khác code cũ:
 * - Code cũ đạt priority vì các if của joystick phải chạy sau joystick trái.
 * - Example mới viết priority rõ bằng return, dễ đọc và không phụ thuộc thứ tự if.
 *
 * ĐẤU DÂY PS2 - ARDUINO MEGA 2560
 * ==========================================================================
 *   PS2 DAT/MISO -> D50
 *   PS2 CMD/MOSI -> D51
 *   PS2 CLK/SCK  -> D52
 *   PS2 CS/ATT   -> D53
 *   PS2 GND      -> GND chung
 *   PS2 VCC      -> nguồn đúng theo receiver
 *
 * MOTOR V5 / MODERN
 * ==========================================================================
 *   M1 = trước trái  = PWM D5
 *   M2 = sau trái    = PWM D6
 *   M3 = sau phải    = PWM D7
 *   M4 = trước phải  = PWM D8
 *
 *   DIR dùng D30..D37 theo thư viện motor.
 *
 * AN TOÀN
 * ==========================================================================
 * - Mất PS2 -> robot.stop() ngay.
 * - CENTER/UNKNOWN joystick trái -> stop.
 * - Không dùng delay().
 * - PS2 poll mặc định 50 Hz.
 *
 * DEBUG
 * ==========================================================================
 * Đổi PS2_ROBOT_DEBUG = 1 để bật debug event một dòng.
 * Khi = 0, production path không in Serial.
 */

#include <TungLam_PS2.h>
#include <TungLam_OmniMecanum_4WD.h>

#define PS2_ROBOT_DEBUG 0

TungLamPS2 ps2;
TungLamDrive4WD robot;

constexpr uint8_t PS2_CS_PIN = 53;

constexpr uint8_t SPEED_SLOW = 120;
constexpr uint8_t SPEED_NORMAL = 180;
constexpr uint8_t SPEED_FAST = 230;

constexpr uint8_t TURN_SLOW = 110;
constexpr uint8_t TURN_NORMAL = 155;
constexpr uint8_t TURN_FAST = 200;

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

  // ------------------------------------------------------------------------
  // FAIL-SAFE
  // ------------------------------------------------------------------------
  if (!ps2.connected()) {
    robot.stop();
    return;
  }

  // ------------------------------------------------------------------------
  // TỐC ĐỘ - chỉ là mapping minh họa.
  // L1 = chậm, R1 = nhanh, không giữ gì = bình thường.
  // Nếu giữ đồng thời L1 và R1 thì L1 ưu tiên để an toàn hơn.
  // ------------------------------------------------------------------------
  uint8_t moveSpeed = SPEED_NORMAL;
  uint8_t turnSpeed = TURN_NORMAL;

  if (ps2.button(PS2Button::L1)) {
    moveSpeed = SPEED_SLOW;
    turnSpeed = TURN_SLOW;
  } else if (ps2.button(PS2Button::R1)) {
    moveSpeed = SPEED_FAST;
    turnSpeed = TURN_FAST;
  }

  // ------------------------------------------------------------------------
  // PRIORITY 1 - JOYSTICK PHẢI: XOAY
  // ------------------------------------------------------------------------
  // Đây là phần quan trọng nhất của example.
  // Khi joystick phải có lệnh quay, bỏ qua lệnh tịnh tiến của joystick trái.
  // Thả joystick phải về tâm -> code đi xuống và đọc joystick trái trở lại.
  switch (ps2.rightDirection()) {
    case PS2StickDirection::Left:
      robot.rotateLeft(turnSpeed);
      return;

    case PS2StickDirection::Right:
      robot.rotateRight(turnSpeed);
      return;

    default:
      // UP/DOWN/CENTER/UNKNOWN của joystick phải không chiếm quyền.
      break;
  }

  // ------------------------------------------------------------------------
  // PRIORITY 2 - JOYSTICK TRÁI: TIẾN/LÙI/NGANG
  // ------------------------------------------------------------------------
  switch (ps2.leftDirection()) {
    case PS2StickDirection::Up:
      robot.forward(moveSpeed);
      break;

    case PS2StickDirection::Down:
      robot.backward(moveSpeed);
      break;

    case PS2StickDirection::Left:
      robot.strafeLeft(moveSpeed);
      break;

    case PS2StickDirection::Right:
      robot.strafeRight(moveSpeed);
      break;

    case PS2StickDirection::Center:
    case PS2StickDirection::Unknown:
    default:
      robot.stop();
      break;
  }
}
