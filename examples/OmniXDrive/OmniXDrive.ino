/**
 * @file OmniXDrive.ino
 * @brief Ví dụ tối thiểu dùng modern API cho đế Omni X-drive 4 bánh.
 *
 * Hệ tọa độ:
 * - +vx = tiến
 * - +vy = trái
 * - +wz = quay trái / CCW
 *
 * Omni có nhiều kiểu bố trí cơ khí. Hãy dùng FirstMotorTest để xác nhận M1..M4
 * và polarity trước khi chạy chuyển động X-drive tổng hợp.
 */

// Nạp modern 4WD API.
#include <TungLam_OmniMecanum_4WD.h>

// Tạo một controller cho đế 4 bánh.
TungLamDrive4WD robot;

void setup() {
  // Khởi tạo chân motor và PWM khoảng 7,8125 kHz.
  robot.begin(TungLamPwmMode::High7k8Hz);

  // Chọn mixer Omni-X cho drive(vx, vy, wz).
  robot.setChassis(TungLamChassis::OmniX);
}

void loop() {
  // Đồng bộ state phần mềm sau các xung hãm hardware-timed nếu có.
  robot.update();

  // Ra lệnh vừa tiến vừa sang trái, không quay:
  // vx=140 -> thành phần tiến
  // vy=80  -> thành phần sang trái (+Y)
  // wz=0   -> không có thành phần quay
  robot.drive(140, 80, 0);

  // Mixer Omni-X normalized dùng mô hình canonical với trục lăn 45 độ.
  // Nếu muốn dùng m/s và rad/s thì chuyển sang driveVelocity().
}
