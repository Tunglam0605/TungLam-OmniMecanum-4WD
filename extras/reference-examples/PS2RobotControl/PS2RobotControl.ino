/**
 * @file PS2RobotControl.ino
 * @brief Ví dụ KHUYẾN NGHỊ: điều khiển đế robot theo phong cách RoboBall/V5.
 *
 * MỤC TIÊU
 * ==========================================================================
 * Ví dụ này chỉ tập trung vào LOGIC LÁI XE, không trộn thêm cơ cấu phụ:
 *
 *   JOYSTICK TRÁI = chuyển động chính
 *     UP    -> tiến
 *     DOWN  -> lùi
 *     LEFT  -> ngang trái
 *     RIGHT -> ngang phải
 *
 *   JOYSTICK PHẢI = quay, ƯU TIÊN CAO HƠN joystick trái
 *     LEFT  -> xoay trái / CCW
 *     RIGHT -> xoay phải / CW
 *
 * HÀNH VI QUAN TRỌNG
 * ==========================================================================
 * 1. Giữ joystick trái UP -> robot tiến.
 * 2. Vẫn giữ UP, gạt joystick phải RIGHT -> robot xoay phải.
 * 3. Thả joystick phải về CENTER -> robot tiếp tục tiến vì joystick trái
 *    vẫn đang giữ UP.
 *
 * Đây chính là priority đã xuất hiện trong các project RoboBall cũ, nhưng
 * code mới biểu diễn rõ bằng return thay vì phụ thuộc câu if nào chạy sau.
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
 * MOTOR BASELINE V5
 * ==========================================================================
 *   M1 = trước-trái  = PWM D5
 *   M2 = sau-trái    = PWM D6
 *   M3 = sau-phải    = PWM D7
 *   M4 = trước-phải  = PWM D8
 *
 *   Thứ tự code    : M1, M2, M3, M4
 *   Theo kim đồng hồ: M1 -> M4 -> M3 -> M2
 *
 * AN TOÀN
 * ==========================================================================
 * - Mất PS2 -> robot.stop() ngay.
 * - Joystick phải chỉ chiếm quyền khi LEFT/RIGHT.
 * - CENTER/UNKNOWN joystick trái -> stop.
 * - Không dùng delay().
 * - Poll PS2 mặc định 50 Hz.
 *
 * DEBUG
 * ==========================================================================
 * Đổi PS2_ROBOT_DEBUG = 1 để bật ps2.debug(Serial).
 * Debug chỉ in khi state thay đổi; production để = 0.
 *
 * Muốn học button(), pressed(), released(), raw analog hay reconnect:
 * xem các example trong thư viện TungLam_PS2.
 */

#include <TungLam_PS2.h>
#include <TungLam_OmniMecanum_4WD.h>

#define PS2_ROBOT_DEBUG 0

TungLamPS2 ps2;
TungLamDrive4WD robot;

constexpr uint8_t PS2_CS_PIN = 53;
constexpr uint8_t MOVE_SPEED = 180;
constexpr uint8_t TURN_SPEED = 155;

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

  // FAIL-SAFE: mất receiver/tay cầm thì không giữ lệnh cũ.
  if (!ps2.connected()) {
    robot.stop();
    return;
  }

  // ========================================================================
  // PRIORITY 1 - JOYSTICK PHẢI: XOAY
  // ========================================================================
  // Nếu có lệnh quay, thực hiện ngay và return.
  // Vì vậy joystick phải luôn ghi đè joystick trái.
  switch (ps2.rightDirection()) {
    case PS2StickDirection::Left:
      robot.rotateLeft(TURN_SPEED);
      return;

    case PS2StickDirection::Right:
      robot.rotateRight(TURN_SPEED);
      return;

    default:
      // UP/DOWN/CENTER/UNKNOWN không chiếm quyền điều khiển.
      break;
  }

  // ========================================================================
  // PRIORITY 2 - JOYSTICK TRÁI: TIẾN/LÙI/NGANG
  // ========================================================================
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
