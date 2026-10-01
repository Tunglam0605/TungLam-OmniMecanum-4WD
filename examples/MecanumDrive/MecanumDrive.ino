/**
 * @file MecanumDrive.ino
 * @brief Ví dụ tối thiểu dùng modern API với đế Mecanum-X.
 *
 * Hệ tọa độ:
 * - +vx = tiến
 * - +vy = ngang trái
 * - +wz = quay trái / CCW
 *
 * drive() dùng đơn vị normalized kiểu -255..255, không phải m/s.
 */

// Nạp modern API.
#include <TungLam_OmniMecanum_4WD.h>

// Tạo controller cho đế 4 bánh.
TungLamDrive4WD robot;

void setup() {
  // Khởi tạo D5..D8 PWM và D30..D37 DIR.
  // High7k8Hz là chế độ PWM khoảng 7,8125 kHz.
  robot.begin(TungLamPwmMode::High7k8Hz);

  // Chọn mixer Mecanum-X cho drive(vx, vy, wz).
  robot.setChassis(TungLamChassis::MecanumX);

  // Nếu một motor lắp ngược chiều logic, chỉ đảo riêng motor đó.
  // Ví dụ: robot.setMotorInverted(3, true);
}

void loop() {
  // Đồng bộ state phần mềm sau ABS.
  // Việc cắt ABS phần cứng không phụ thuộc lệnh này.
  robot.update();

  // Chạy tiến bằng command normalized:
  // +vx = tiến, +vy = trái, +wz = CCW.
  // Ở đây chỉ có vx=150, còn vy=wz=0.
  robot.drive(150, 0, 0);

  // Nếu muốn điều khiển bằng m/s và rad/s, hãy khai báo TungLamDriveConfig
  // rồi dùng driveVelocity(vxMps, vyMps, wzRadps).
}
