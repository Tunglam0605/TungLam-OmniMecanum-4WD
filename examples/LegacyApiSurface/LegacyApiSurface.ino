/**
 * @file LegacyApiSurface.ino
 * @brief Bài regression compile cho toàn bộ bề mặt API V5 cũ.
 *
 * Đây chủ yếu là ví dụ dành cho CI/API compatibility, không phải demo chạy xe.
 * exerciseLegacyApi() cố tình KHÔNG được gọi trong loop(), vì gọi liên tiếp mọi
 * lệnh chuyển động không có ý nghĩa khi test robot thật.
 */

// Nạp đúng tên header mà project V5 cũ từng sử dụng.
#include <TungLam_Control_MotorV5.h>

// Tạo đúng class V5 để kiểm tra khả năng tương thích source.
TungLam_Control_MotorV5 robot;

/**
 * @brief Tham chiếu toàn bộ symbol public V5 để CI phát hiện API bị phá.
 *
 * Nếu về sau một hàm cũ bị xóa hoặc đổi signature, sketch này phải compile fail.
 */
void exerciseLegacyApi() {
  // Chế độ PWM legacy tần số thấp.
  robot.Mode0();

  // Chế độ PWM legacy tần số cao.
  robot.Mode1();

  // Kiểm tra API khởi tạo Timer1 phụ.
  robot.Init_Timer1(0, 0);

  // Kiểm tra API khởi tạo Timer2 phụ.
  robot.Init_Timer2(0, 0);

  // Dừng toàn bộ 4 bánh.
  robot.STOP();

  // Chạy tiến bằng duty chung.
  robot.moveForward(100);

  // Chạy lùi bằng duty chung.
  robot.moveBackward(100);

  // Tiến chéo phải.
  robot.Forward_Right(100);

  // Lùi chéo phải.
  robot.Backward_Right(100);

  // Quay phải / CW.
  robot.moveRight(100);

  // Quay trái / CCW.
  robot.moveLeft(100);

  // Đi ngang trái.
  robot.moveLeftSide(100);

  // Đi ngang phải.
  robot.moveRightSide(100);

  // Tiến chéo trái.
  robot.Forward_Left(100);

  // Lùi chéo trái.
  robot.Backward_Left(100);

  // Kiểm tra API đổi chiều trực tiếp bánh 1.
  robot.Dir(1, false);

  // Kiểm tra API đổi chiều trực tiếp bánh 2.
  robot.Dir(2, true);

  // Kiểm tra API đổi chiều trực tiếp bánh 3.
  robot.Dir(3, false);

  // Kiểm tra API đổi chiều trực tiếp bánh 4.
  robot.Dir(4, true);

  // Tiến với PWM độc lập từng bánh.
  robot.Tien(100, 100, 100, 100);

  // Lùi với PWM độc lập từng bánh.
  robot.Lui(100, 100, 100, 100);

  // Quay trái với PWM độc lập từng bánh.
  robot.Trai(100, 100, 100, 100);

  // Quay phải với PWM độc lập từng bánh.
  robot.Phai(100, 100, 100, 100);

  // Tiến-trái.
  robot.T_Trai(100, 100, 100, 100);

  // Tiến-phải.
  robot.T_Phai(100, 100, 100, 100);

  // Lùi-trái.
  robot.L_Trai(100, 100, 100, 100);

  // Lùi-phải.
  robot.L_Phai(100, 100, 100, 100);

  // Đi ngang trái.
  robot.N_Trai(100, 100, 100, 100);

  // Đi ngang phải.
  robot.N_Phai(100, 100, 100, 100);

  // Kiểm tra API cấu hình timing ABS.
  robot.setTimABS(45, 65, 70, 75, 80, 85);

  // Kiểm tra symbol ABS với duty do người dùng chọn.
  robot.ABS(100);
}

void setup() {
  // Khởi tạo phần cứng ở Mode1.
  robot.Mode1();

  // Giữ robot dừng vì exerciseLegacyApi() không chạy thật.
  robot.STOP();
}

void loop() {
  // Cố tình để trống: sketch dùng để compile-check API.
}
