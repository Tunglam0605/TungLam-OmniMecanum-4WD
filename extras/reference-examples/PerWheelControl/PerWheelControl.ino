/**
 * @file PerWheelControl.ino
 * @brief Minh họa tầng điều khiển trực tiếp từng bánh bằng giá trị có dấu.
 *
 * setWheels(m1, m2, m3, m4) quy ước:
 * - giá trị dương -> chiều tiến logic
 * - giá trị âm    -> chiều lùi logic
 * - bằng 0        -> dừng bánh đó
 * - độ lớn        -> PWM 0..255
 *
 * Nếu đang commissioning phần cứng lần đầu, nên dùng FirstMotorTest vì đầy đủ hơn.
 */

// Nạp modern API có setWheels().
#include <TungLam_OmniMecanum_4WD.h>

// Tạo controller sở hữu 4 motor của đế.
TungLamDrive4WD robot;

void setup() {
  // Khởi tạo GPIO motor và chế độ PWM tần số cao mặc định.
  robot.begin();

  // Chỉ cho M1 chạy tiến PWM 80; M2/M3/M4 giữ 0.
  robot.setWheels(80, 0, 0, 0);

  // Quan sát M1 trong 800 ms.
  delay(800);

  // Dừng toàn bộ trước khi đổi sang bánh khác.
  robot.stop();

  // Nghỉ để dễ phân biệt từng bài test.
  delay(500);

  // Chỉ cho M2 chạy tiến PWM 80.
  robot.setWheels(0, 80, 0, 0);

  // Quan sát M2 trong 800 ms.
  delay(800);

  // Dừng toàn bộ.
  robot.stop();

  // Nghỉ trước khi test M3.
  delay(500);

  // Chỉ cho M3 chạy tiến PWM 80.
  robot.setWheels(0, 0, 80, 0);

  // Quan sát M3 trong 800 ms.
  delay(800);

  // Dừng toàn bộ.
  robot.stop();

  // Nghỉ trước khi test M4.
  delay(500);

  // Chỉ cho M4 chạy tiến PWM 80.
  robot.setWheels(0, 0, 0, 80);

  // Quan sát M4 trong 800 ms.
  delay(800);

  // Kết thúc commissioning ở trạng thái dừng.
  robot.stop();
}

void loop() {
  // Giữ pattern gọi service của modern API.
  // Khi không có ABS thì lệnh này không đổi motor output, chỉ đồng bộ state.
  robot.update();
}
