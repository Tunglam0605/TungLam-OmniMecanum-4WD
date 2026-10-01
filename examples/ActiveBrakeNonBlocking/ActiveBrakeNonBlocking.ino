/**
 * @file ActiveBrakeNonBlocking.ino
 * @brief Minh họa ABS hardware-timed kết hợp state machine không block chương trình.
 *
 * Ví dụ này cố tình không dùng delay() trong chu trình chuyển động chính để
 * chương trình vẫn có thể đọc sensor, joystick, Serial hoặc xử lý tác vụ khác
 * trong khi xung hãm ABS đang hoạt động.
 *
 * Cơ chế ABS modern:
 * - ABS(duty) áp mô-men ngược ngay lập tức.
 * - Timer3 overflow ISR tự dừng xung hãm đúng deadline.
 * - update() không cần cho an toàn phần cứng; nó chỉ đồng bộ state phần mềm.
 */

// Nạp API modern của thư viện.
#include <TungLam_OmniMecanum_4WD.h>

// Tạo một controller duy nhất cho phần cứng 4 bánh.
TungLamDrive4WD robot;

// Ba trạng thái của ví dụ.
enum class DemoState : uint8_t {
  Driving,  // Robot đang chạy tiến.
  Braking,  // Robot đang trong xung hãm ABS.
  Waiting   // Robot đã dừng và đang chờ trước chu kỳ tiếp theo.
};

// Bắt đầu state machine ở trạng thái đang chạy.
DemoState state = DemoState::Driving;

// Lưu thời điểm bắt đầu state hiện tại.
uint32_t stateStart = 0;

void setup() {
  // Khởi tạo GPIO và Timer3/Timer4 với PWM mặc định tần số cao.
  robot.begin();

  // Chọn Mecanum-X.
  robot.setChassis(TungLamChassis::MecanumX);

  // Cấu hình bảng thời gian ABS 6 khoảng.
  robot.setTimABS(45, 65, 70, 75, 80, 85);

  // Bắt đầu ví dụ bằng việc chạy tiến PWM 150.
  robot.forward(150);

  // Ghi lại thời điểm bắt đầu chạy.
  stateStart = millis();
}

void loop() {
  // Đồng bộ state phần mềm sau khi Timer3 ISR đã cắt xung ABS.
  // Phần cứng vẫn an toàn ngay cả khi bỏ lệnh update() này.
  robot.update();

  // Đọc millis() một lần để mọi phép so sánh trong vòng loop dùng cùng timestamp.
  const uint32_t now = millis();

  // Sau 1,5 giây chạy tiến thì bắt đầu ABS.
  if (state == DemoState::Driving && now - stateStart >= 1500UL) {
    // Dùng PWM 150 làm lực hãm ngược thực tế.
    robot.ABS(150);

    // Chuyển state ứng dụng sang Braking.
    state = DemoState::Braking;
  }

  // Chờ đến khi Timer3 báo xung ABS đã kết thúc.
  if (state == DemoState::Braking && !robot.isBraking()) {
    // Chuyển sang trạng thái chờ.
    state = DemoState::Waiting;

    // Lưu thời điểm bắt đầu chờ.
    stateStart = now;
  }

  // Sau 2 giây chờ thì chạy lại chu kỳ mới.
  if (state == DemoState::Waiting && now - stateStart >= 2000UL) {
    // Chạy tiến lại với PWM 150.
    robot.forward(150);

    // Trở về trạng thái Driving.
    state = DemoState::Driving;

    // Bắt đầu lại timer của pha chạy.
    stateStart = now;
  }

  // Có thể đặt các tác vụ không block khác ở đây:
  // đọc Serial, cảm biến, PS2, IMU, ROS2 bridge...
}
