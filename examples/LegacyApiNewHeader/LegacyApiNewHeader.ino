/**
 * @file LegacyApiNewHeader.ino
 * @brief Minh họa việc dùng header mới nhưng vẫn gọi class/API V5 cũ.
 *
 * Cách này phù hợp khi project muốn chuyển sang package mới nhưng chưa muốn
 * đổi tên class hay các hàm điều khiển đã dùng trong code V5.
 */

// Nạp header chính mới của thư viện.
#include <TungLam_OmniMecanum_4WD.h>

// Vẫn tạo object bằng class V5 cũ.
TungLam_Control_MotorV5 robot;

void setup() {
  // Khởi tạo PWM theo Mode1 như các project V5.
  robot.Mode1();

  // Giữ nguyên cách cấu hình 6 mốc thời gian ABS.
  robot.setTimABS(45, 65, 70, 75, 80, 85);
}

void loop() {
  // Tien(): chạy tiến với PWM độc lập cho M1..M4.
  robot.Tien(150, 150, 150, 150);

  // Giữ chạy tiến 1 giây.
  delay(1000);

  // Hãm ngược với duty chính xác bằng 210.
  robot.ABS(210);

  // Chờ sau xung ABS hardware-timed.
  delay(1000);

  // N_Phai(): đi ngang phải theo API V5.
  robot.N_Phai(140, 140, 140, 140);

  // Giữ đi ngang 800 ms.
  delay(800);

  // Hãm chuyển động ngang với duty 190.
  robot.ABS(190);

  // Chờ trước khi lặp lại.
  delay(1000);
}
