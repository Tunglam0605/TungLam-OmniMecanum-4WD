/**
 * @file StudentQuickStart.ino
 * @brief Ví dụ khuyến nghị cho học sinh: khai báo robot một lần rồi ra lệnh bằng m/s.
 *
 * Bạn chỉ cần nhớ:
 * 1. robot.begin()
 * 2. robot.setChassis(...)
 * 3. robot.setDriveConfig(...)
 * 4. robot.enableSmartSafety(...)
 * 5. robot.update() trong loop()
 * 6. robot.driveVelocity(vx, vy, wz)
 *
 * Các việc khó như động học nghịch, giới hạn tốc độ 4 bánh, quy đổi RPM -> PWM,
 * watchdog mất lệnh và làm mượt tăng/giảm tốc được thư viện tự xử lý.
 */

#include <TungLam_OmniMecanum_4WD.h>

// Tạo bộ điều khiển đế.
TungLamDrive4WD robot;

// Khai báo thông số robot thật.
// Hãy thay các số dưới đây bằng motor/bánh/khung xe của bạn.
TungLamDriveConfig model(
    12.0f,   // điện áp danh định motor [V]
    300.0f,  // RPM đầu ra hộp số
    12.0f,   // điện áp nguồn motor [V]
    0.050f,  // bán kính bánh [m]
    0.320f,  // khoảng cách tâm bánh trước-sau [m]
    0.280f,  // khoảng cách tâm bánh trái-phải [m]
    0.85f    // hệ số hiệu chỉnh thực tế
);

void setup() {
  // Khởi tạo phần cứng motor.
  robot.begin();

  // Chọn đúng loại đế đang dùng.
  robot.setChassis(TungLamChassis::MecanumX);

  // Nạp thông số vật lý để thư viện tự tính động học và tốc độ.
  robot.setDriveConfig(model);

  // Bật chế độ an toàn thông minh:
  // - nếu mất lệnh quá 500 ms -> tự dừng;
  // - vx/vy chỉ thay đổi tối đa 1.0 m/s mỗi giây;
  // - wz chỉ thay đổi tối đa 2.0 rad/s mỗi giây.
  robot.enableSmartSafety(500, 1.0f, 2.0f);
}

void loop() {
  // Gọi liên tục để thư viện xử lý watchdog, ABS state và các tác vụ nội bộ.
  robot.update();

  // Yêu cầu robot:
  // vx = +0.30 m/s -> tiến
  // vy =  0.00 m/s -> không đi ngang
  // wz =  0.00 rad/s -> không quay
  //
  // Nếu tốc độ này vượt khả năng motor/đế, thư viện tự giảm theo đúng tỷ lệ.
  robot.driveVelocity(0.30f, 0.00f, 0.00f);

  // Không cần tự tính PWM từng bánh.
  // Không cần tự viết phương trình Mecanum.
  // Không cần tự clamp tốc độ.
  // Không cần tự viết soft-start.
}
