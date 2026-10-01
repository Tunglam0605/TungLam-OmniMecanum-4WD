/**
 * @file ActiveBrake.ino
 * @brief Minh họa cơ chế hãm ngược chủ động ABS với API V5.
 *
 * Phần cứng:
 * - Arduino Mega 2560
 * - 2 module L298N
 * - 4 động cơ DC
 *
 * Trình tự:
 * 1. Robot chạy tiến với PWM 150.
 * 2. Sau 1,5 giây, ABS tạo mô-men ngược với PWM 100.
 * 3. Timer3 tự động kết thúc xung hãm.
 * 4. API V5 không cần gọi robot.update().
 *
 * An toàn:
 * - Lần đầu nên kê robot lên khỏi mặt sàn.
 * - Bắt đầu với duty ABS thấp rồi tăng dần theo thực tế.
 */

// Nạp header tương thích V5 để project cũ vẫn giữ nguyên cách include.
#include <TungLam_Control_MotorV5.h>

// Tạo một bộ điều khiển V5 cho toàn bộ 4 động cơ của đế.
TungLam_Control_MotorV5 robot;

void setup() {
  // Khởi tạo Timer3/Timer4 ở chế độ PWM tần số cao khoảng 7,8125 kHz.
  robot.Mode1();

  // Cấu hình 6 khoảng thời gian hãm ABS theo thời gian robot đã chạy trước đó.
  // Các khoảng lần lượt là: <500, <1000, <1500, <2000, <3000 và >=3000 ms.
  robot.setTimABS(45, 65, 70, 75, 80, 85);

  // Đưa phần cứng về trạng thái dừng an toàn trước khi bắt đầu.
  robot.STOP();
}

void loop() {
  // Cho cả 4 bánh chạy tiến với PWM chung bằng 150.
  robot.moveForward(150);

  // Duy trì chạy tiến 1,5 giây để rơi vào khoảng timing ABS 1500..1999 ms.
  delay(1500);

  // Hãm ngược với đúng duty người dùng chọn là 100.
  // Hàm trả về ngay; Timer3 ISR sẽ tự cắt lực hãm đúng thời gian.
  robot.ABS(100);

  // Delay này chỉ để quan sát ví dụ.
  // TungLam_Control_MotorV5 không yêu cầu robot.update().
  delay(2000);
}
