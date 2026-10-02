/**
 * @file PS2RobotControl.ino
 * @brief Điều khiển trực tiếp đế Mecanum/Omni bằng TungLam_PS2.
 *
 * MỤC TIÊU
 * ==========================================================================
 * Ví dụ này ghép hai thư viện nhưng vẫn giữ chúng độc lập:
 *
 *   Tay cầm PS2
 *        |
 *        v
 *   TungLam_PS2
 *        |
 *        |  joystick/button state đã lọc
 *        v
 *   application mapping
 *        |
 *        |  vx, vy, wz normalized
 *        v
 *   TungLam_OmniMecanum_4WD
 *        |
 *        v
 *   4 motor
 *
 * ĐẤU DÂY PS2 TRÊN ARDUINO MEGA 2560
 * ==========================================================================
 *   PS2 DAT/MISO -> D50
 *   PS2 CMD/MOSI -> D51
 *   PS2 CLK/SCK  -> D52
 *   PS2 CS/ATT   -> D53
 *   PS2 GND      -> GND chung
 *   PS2 VCC      -> nguồn đúng theo receiver
 *
 * ĐẤU MOTOR CỦA THƯ VIỆN ĐẾ
 * ==========================================================================
 *   PWM: M1=D5, M2=D6, M3=D7, M4=D8
 *   DIR: D30..D37 theo tài liệu TungLam_OmniMecanum_4WD
 *
 * Hai nhóm chân không xung đột nhau.
 *
 * MAPPING TRONG VÍ DỤ NÀY
 * ==========================================================================
 * Joystick trái:
 *   UP    -> tiến
 *   DOWN  -> lùi
 *   LEFT  -> đi ngang trái
 *   RIGHT -> đi ngang phải
 *
 * Joystick phải:
 *   LEFT  -> quay trái / CCW
 *   RIGHT -> quay phải / CW
 *
 * Button:
 *   L1 giữ  -> tốc độ chậm
 *   R1 giữ  -> tốc độ nhanh
 *   START   -> toggle enable/disable điều khiển
 *
 * Đây CHỈ LÀ mapping của example. TungLam_PS2 không gắn cứng chức năng
 * cho bất kỳ nút nào.
 *
 * AN TOÀN
 * ==========================================================================
 * - Mất kết nối PS2 -> robot.stop() ngay.
 * - Unknown joystick -> trục tương ứng về 0.
 * - Không dùng delay().
 * - Poll tay cầm mặc định 50 Hz.
 *
 * DEBUG
 * ==========================================================================
 * Đổi PS2_ROBOT_DEBUG thành 1 để bật Serial 115200 và event debug:
 *
 *   [PS2] BTN=START:PRESSED | LEFT=UP(0,-117)
 *
 * Khi = 0, đường chạy production không gọi Serial debug.
 */

#include <TungLam_PS2.h>
#include <TungLam_OmniMecanum_4WD.h>

#define PS2_ROBOT_DEBUG 0

TungLamPS2 ps2;
TungLamDrive4WD robot;

constexpr uint8_t PS2_CS_PIN = 53;

// PWM normalized của ví dụ. Có thể tune theo robot thật.
constexpr int16_t SPEED_SLOW = 100;
constexpr int16_t SPEED_NORMAL = 170;
constexpr int16_t SPEED_FAST = 220;
constexpr int16_t TURN_NORMAL = 140;
constexpr int16_t TURN_FAST = 190;

bool driveEnabled = true;

void setup() {
#if PS2_ROBOT_DEBUG
  Serial.begin(115200);
#endif

  // Khởi tạo đế robot.
  robot.begin();
  robot.setChassis(TungLamChassis::MecanumX);

  // Khởi tạo PS2 bằng hardware SPI mặc định của Mega.
  // DAT=D50, CMD=D51, CLK=D52, CS=D53.
  ps2.begin(PS2_CS_PIN);
  ps2.setPollRate(PS2PollRate::Hz50);
}

void loop() {
  // Hai update đều gọi liên tục, không delay().
  ps2.update();
  robot.update();

#if PS2_ROBOT_DEBUG
  // Chỉ in khi tay cầm có thay đổi; không spam Serial.
  ps2.debug(Serial);
#endif

  // Fail-safe: mất receiver/tay cầm -> dừng ngay.
  if (!ps2.connected()) {
    robot.stop();
    return;
  }

  // pressed() là event one-shot nên START chỉ toggle đúng một lần mỗi lần nhấn.
  if (ps2.pressed(PS2Button::Start)) {
    driveEnabled = !driveEnabled;

    if (!driveEnabled) {
      robot.stop();
    }
  }

  if (!driveEnabled) {
    robot.stop();
    return;
  }

  // ------------------------------------------------------------------------
  // 1. Chọn tốc độ từ button HOLD.
  // ------------------------------------------------------------------------
  int16_t moveDuty = SPEED_NORMAL;
  int16_t turnDuty = TURN_NORMAL;

  // Nếu đồng thời giữ L1 và R1, L1 được ưu tiên để giảm tốc an toàn.
  if (ps2.button(PS2Button::L1)) {
    moveDuty = SPEED_SLOW;
    turnDuty = SPEED_SLOW;
  } else if (ps2.button(PS2Button::R1)) {
    moveDuty = SPEED_FAST;
    turnDuty = TURN_FAST;
  }

  // ------------------------------------------------------------------------
  // 2. Joystick trái -> vx, vy.
  // Quy ước motor library:
  //   +vx = tiến
  //   +vy = trái
  // ------------------------------------------------------------------------
  int16_t vx = 0;
  int16_t vy = 0;

  switch (ps2.leftDirection()) {
    case PS2StickDirection::Up:
      vx = moveDuty;
      break;

    case PS2StickDirection::Down:
      vx = -moveDuty;
      break;

    case PS2StickDirection::Left:
      vy = moveDuty;
      break;

    case PS2StickDirection::Right:
      vy = -moveDuty;
      break;

    case PS2StickDirection::Center:
    case PS2StickDirection::Unknown:
    default:
      // Center/Unknown -> không tịnh tiến.
      break;
  }

  // ------------------------------------------------------------------------
  // 3. Joystick phải ngang -> wz.
  // Quy ước motor library:
  //   +wz = quay trái / CCW
  //   -wz = quay phải / CW
  // ------------------------------------------------------------------------
  int16_t wz = 0;

  switch (ps2.rightDirection()) {
    case PS2StickDirection::Left:
      wz = turnDuty;
      break;

    case PS2StickDirection::Right:
      wz = -turnDuty;
      break;

    default:
      // UP/DOWN/CENTER/UNKNOWN của joystick phải không quay trong example này.
      break;
  }

  // ------------------------------------------------------------------------
  // 4. Gửi một vector duy nhất vào mixer.
  // ------------------------------------------------------------------------
  // Dùng drive(vx, vy, wz) cho phép vừa tịnh tiến vừa quay cùng lúc.
  // Không gọi forward() rồi rotate() liên tiếp vì lệnh sau có thể ghi đè lệnh trước.
  if (vx == 0 && vy == 0 && wz == 0) {
    robot.stop();
  } else {
    robot.drive(vx, vy, wz);
  }
}
