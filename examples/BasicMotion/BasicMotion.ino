/**
 * @file BasicMotion.ino
 * @brief Ví dụ cơ bản cho người mới bằng các hàm chuyển động kiểu V5.
 *
 * Trình tự:
 * 1. Chạy tiến.
 * 2. Dừng.
 * 3. Đi ngang phải.
 * 4. Dừng.
 * 5. Quay trái.
 * 6. Dừng và lặp lại.
 *
 * delay() chỉ dùng để người học dễ quan sát từng chuyển động.
 */

// Nạp header tương thích V5.
#include <TungLam_Control_MotorV5.h>

// Tạo một controller cho 4 động cơ.
TungLam_Control_MotorV5 robot;

void setup() {
  // Dùng Mode1 để chạy PWM khoảng 7,8125 kHz như các project V5 thực tế.
  robot.Mode1();

  // Đảm bảo robot khởi động ở trạng thái dừng.
  robot.STOP();
}

void loop() {
  // Cho cả 4 bánh chạy tiến với PWM 140.
  robot.moveForward(140);

  // Giữ chuyển động tiến 1,2 giây.
  delay(1200);

  // Dừng robot.
  robot.STOP();

  // Chờ 0,5 giây trước chuyển động tiếp theo.
  delay(500);

  // Đi ngang sang phải với PWM 140.
  robot.moveRightSide(140);

  // Giữ chuyển động ngang 1,2 giây.
  delay(1200);

  // Dừng trước khi đổi sang quay.
  robot.STOP();

  // Chờ 0,5 giây.
  delay(500);

  // Quay trái / ngược chiều kim đồng hồ với PWM 120.
  robot.moveLeft(120);

  // Giữ quay 0,8 giây.
  delay(800);

  // Dừng toàn bộ 4 bánh.
  robot.STOP();

  // Chờ 1 giây rồi lặp lại toàn bộ chu trình.
  delay(1000);
}
