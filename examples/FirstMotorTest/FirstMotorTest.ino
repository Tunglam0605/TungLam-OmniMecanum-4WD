/**
 * @file FirstMotorTest.ino
 * @brief Bài kiểm tra đầu tiên để xác nhận M1, M2, M3, M4 và chiều quay.
 *
 * QUAN TRỌNG TRƯỚC KHI CHẠY:
 * - Kê robot lên để cả 4 bánh không chạm đất.
 * - Kiểm tra lại bảng chân trong README.
 * - Bắt đầu với TEST_PWM khoảng 60..80.
 * - Xác nhận vị trí:
 *     M1 = trước-trái
 *     M2 = sau-trái
 *     M3 = sau-phải
 *     M4 = trước-phải
 *
 * Sketch sẽ test từng bánh riêng lẻ theo chiều tiến rồi chiều lùi.
 * Đây là bước commissioning nên làm trước khi test chuyển động Mecanum/Omni.
 */

// Nạp modern API vì setWheels() rất phù hợp để test từng bánh.
#include <TungLam_OmniMecanum_4WD.h>

// Tạo một controller duy nhất cho 4 motor.
TungLamDrive4WD robot;

// PWM test thấp để lần kiểm tra đầu tiên an toàn hơn.
constexpr uint8_t TEST_PWM = 80;

// Mỗi chiều quay chạy trong 900 ms.
constexpr uint16_t RUN_MS = 900;

// Nghỉ 500 ms giữa các lần đổi chiều/bánh.
constexpr uint16_t PAUSE_MS = 500;

/**
 * @brief Chỉ chạy một bánh, ba bánh còn lại bằng 0.
 * @param wheel Số bánh logic từ 1 đến 4.
 * @param duty PWM có dấu: dương=tiến logic, âm=lùi logic.
 */
void driveOnly(uint8_t wheel, int16_t duty) {
  // Chọn phần tử tương ứng trong vector M1..M4.
  switch (wheel) {
    case 1:
      // Chỉ M1 chạy.
      robot.setWheels(duty, 0, 0, 0);
      break;

    case 2:
      // Chỉ M2 chạy.
      robot.setWheels(0, duty, 0, 0);
      break;

    case 3:
      // Chỉ M3 chạy.
      robot.setWheels(0, 0, duty, 0);
      break;

    case 4:
      // Chỉ M4 chạy.
      robot.setWheels(0, 0, 0, duty);
      break;

    default:
      // Nếu số bánh sai thì dừng an toàn.
      robot.stop();
      break;
  }
}

/**
 * @brief Test một bánh theo cả chiều tiến và chiều lùi.
 * @param wheel Số bánh logic từ 1 đến 4.
 */
void testWheel(uint8_t wheel) {
  // In tên bánh trước khi test chiều tiến.
  Serial.print(F("M"));
  Serial.print(wheel);
  Serial.println(F(" forward"));

  // Duty dương = chiều tiến logic.
  driveOnly(wheel, TEST_PWM);

  // Giữ bánh chạy đủ lâu để quan sát.
  delay(RUN_MS);

  // Dừng sau bài test tiến.
  robot.stop();

  // Chờ trước khi đảo chiều.
  delay(PAUSE_MS);

  // In tên bánh trước khi test chiều lùi.
  Serial.print(F("M"));
  Serial.print(wheel);
  Serial.println(F(" reverse"));

  // Duty âm = chiều lùi logic.
  driveOnly(wheel, -(int16_t)TEST_PWM);

  // Quan sát chiều lùi trong cùng khoảng thời gian.
  delay(RUN_MS);

  // Dừng bánh.
  robot.stop();

  // Chờ trước khi sang bánh tiếp theo.
  delay(PAUSE_MS);
}

void setup() {
  // Mở Serial Monitor để theo dõi tiến trình test.
  Serial.begin(115200);

  // Khởi tạo PWM tần số cao khoảng 7,8125 kHz.
  robot.begin(TungLamPwmMode::High7k8Hz);

  // Đảm bảo robot đang dừng trước countdown.
  robot.stop();

  // In tên bài test.
  Serial.println(F("TungLam 4WD - First Motor Test"));

  // Nhắc người dùng kê robot lên.
  Serial.println(F("Lift the robot so all wheels are free."));

  // Báo trước khi bắt đầu chạy motor.
  Serial.println(F("Starting in 3 seconds..."));

  // Cho người dùng 3 giây để phản ứng trước khi motor quay.
  delay(3000);

  // Test lần lượt M1 → M2 → M3 → M4.
  for (uint8_t wheel = 1; wheel <= 4; ++wheel) {
    // Mỗi bánh được test cả tiến và lùi.
    testWheel(wheel);
  }

  // Kết thúc bằng trạng thái dừng.
  robot.stop();

  // Báo hoàn tất trên Serial Monitor.
  Serial.println(F("Motor test complete."));
}

void loop() {
  // Để trống: bài test chỉ chạy một lần trong setup().
}
