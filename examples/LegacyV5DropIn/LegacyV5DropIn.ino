/**
 * @file LegacyV5DropIn.ino
 * @brief Minh họa khả năng cập nhật thư viện mà gần như không sửa code V5 cũ.
 *
 * Tên include, tên class, lệnh chuyển động, ABS và setTimABS đều giữ đúng phong
 * cách của các sketch V5 trước đây.
 */

// Nạp header compatibility với đúng tên cũ.
#include <TungLam_Control_MotorV5.h>

// Tạo object bằng class V5.
TungLam_Control_MotorV5 robot;

void setup() {
  // Dùng Mode1 như project cũ.
  robot.Mode1();

  // Cấu hình 6 khoảng timing ABS.
  robot.setTimABS(45, 65, 70, 75, 80, 85);
}

void loop() {
  // Chạy tiến PWM 160.
  robot.moveForward(160);

  // Giữ tiến 1,2 giây.
  delay(1200);

  // Hãm ngược với duty 200.
  // Implementation mới không block và Timer3 tự cắt xung hãm.
  robot.ABS(200);

  // delay này chỉ để minh họa, ABS không phụ thuộc vào nó.
  delay(500);

  // Đi ngang phải PWM 140 bằng hàm V5 cũ.
  robot.moveRightSide(140);

  // Giữ đi ngang 1 giây.
  delay(1000);

  // Hãm chuyển động ngang bằng duty 180.
  robot.ABS(180);

  // Chờ trước khi lặp lại.
  delay(1000);
}
