/**
 * @file MetricKinematics.ino
 * @brief Ví dụ từ mức cơ bản đến sinh viên về động học Mecanum dùng đơn vị SI.
 *
 * Ví dụ này minh họa mô hình vật lý từ v0.9:
 * - điện áp định mức của motor;
 * - RPM không tải ở đầu ra hộp số;
 * - điện áp nguồn thực cấp cho driver;
 * - bán kính bánh;
 * - wheelbase;
 * - track width;
 * - hệ số hiệu chỉnh vòng hở speedScale.
 *
 * Hệ tọa độ:
 * - +X / +vx = tiến
 * - +Y / +vy = trái
 * - +Z / +wz = quay ngược chiều kim đồng hồ (CCW)
 *
 * QUAN TRỌNG:
 * driveVelocity() hiện là feed-forward vòng hở. Khi chưa có encoder, thư viện
 * chỉ ước lượng PWM từ model motor; không thể đảm bảo robot thật đạt đúng vận
 * tốc SI đã yêu cầu dưới mọi tải.
 */

// Nạp modern API gồm cả drive-base và phần động học.
#include <TungLam_OmniMecanum_4WD.h>

// Tạo một controller cho toàn bộ 4 motor.
TungLamDrive4WD robot;

// Khai báo model vật lý của robot.
//
// Ví dụ motor:
// - điện áp định mức 12 V;
// - tốc độ không tải đầu ra hộp số = 300 RPM tại 12 V.
//
// Ví dụ đế:
// - bán kính bánh = 50 mm = 0,050 m;
// - khoảng cách tâm bánh trước-sau = 320 mm = 0,320 m;
// - khoảng cách tâm bánh trái-phải = 280 mm = 0,280 m.
//
// speedScale = 0,85 là ví dụ hiệu chỉnh thực nghiệm cho sụt áp L298N,
// pin sụt áp và tải cơ khí. Robot thật nên đo lại và tune giá trị này.
TungLamDriveConfig driveModel(
    12.0f,   // motorNominalVoltageV: điện áp danh định motor [V]
    300.0f,  // motorNoLoadRpm: RPM đầu ra hộp số
    12.0f,   // supplyVoltageV: điện áp cấp vào driver [V]
    0.050f,  // wheelRadiusM: bán kính bánh [m]
    0.320f,  // wheelbaseM: tâm bánh trước đến tâm bánh sau [m]
    0.280f,  // trackWidthM: tâm bánh trái đến tâm bánh phải [m]
    0.85f    // speedScale: hệ số hiệu chỉnh vòng hở
);

void setup() {
  // Mở Serial Monitor để xem các giá trị vật lý do thư viện tính.
  Serial.begin(115200);

  // Khởi tạo drive outputs với PWM tần số cao khuyến nghị.
  robot.begin(TungLamPwmMode::High7k8Hz);

  // Chọn mô hình động học Mecanum-X.
  robot.setChassis(TungLamChassis::MecanumX);

  // Lưu và kiểm tra tính hợp lệ của bộ thông số motor + chassis.
  const bool configOk = robot.setDriveConfig(driveModel);

  // Nếu có thông số bắt buộc bằng 0 hoặc âm thì dừng tại đây.
  if (!configOk) {
    Serial.println(F("Invalid drive model. Check voltage, RPM and geometry."));
    robot.stop();
    while (true) {
      // Giữ robot dừng cho tới khi reset.
    }
  }

  // In RPM lý thuyết/ước lượng tại điện áp nguồn đã khai báo.
  Serial.print(F("Estimated motor RPM at supply: "));
  Serial.println(robot.estimatedMotorRpmAtSupply());

  // In tốc độ tiếp tuyến cực đại của bánh.
  Serial.print(F("Estimated max wheel speed [m/s]: "));
  Serial.println(robot.maxWheelLinearSpeedMps(), 4);

  // In tốc độ tịnh tiến cực đại ước lượng của đế.
  Serial.print(F("Estimated max body translation [m/s]: "));
  Serial.println(robot.maxBodyLinearSpeedMps(), 4);

  // In tốc độ quay cực đại ước lượng của đế.
  Serial.print(F("Estimated max yaw rate [rad/s]: "));
  Serial.println(robot.maxYawRateRadps(), 4);

  // Minh họa động học nghịch mà chưa cần chạy motor.
  //
  // Yêu cầu chuyển động thân robot:
  //   vx = +0,40 m/s   -> tiến
  //   vy = +0,15 m/s   -> sang trái
  //   wz = +0,50 rad/s -> quay CCW
  const TungLamWheelVelocity wheels =
      robot.inverseKinematics(0.40f, 0.15f, 0.50f);

  // In vận tốc tiếp tuyến từng bánh do inverse kinematics tính ra.
  Serial.print(F("M1 [m/s]: "));
  Serial.println(wheels.m1Mps, 4);

  Serial.print(F("M2 [m/s]: "));
  Serial.println(wheels.m2Mps, 4);

  Serial.print(F("M3 [m/s]: "));
  Serial.println(wheels.m3Mps, 4);

  Serial.print(F("M4 [m/s]: "));
  Serial.println(wheels.m4Mps, 4);

  // Đưa chính 4 vận tốc bánh đó qua động học thuận.
  const TungLamBodyVelocity body = robot.forwardKinematics(wheels);

  // Kết quả thu hồi phải gần 0,40; 0,15; 0,50 ban đầu.
  Serial.print(F("Recovered vx [m/s]: "));
  Serial.println(body.vxMps, 4);

  Serial.print(F("Recovered vy [m/s]: "));
  Serial.println(body.vyMps, 4);

  Serial.print(F("Recovered wz [rad/s]: "));
  Serial.println(body.wzRadps, 4);

  // Sau phần demo tính toán, giữ robot ở trạng thái dừng.
  robot.stop();
}

void loop() {
  // Đồng bộ state phần mềm modern.
  robot.update();

  // Ví dụ lệnh chạy thật bằng đơn vị SI:
  //
  // robot.driveVelocity(
  //     0.30f,  // vx: tiến 0,30 m/s
  //     0.00f,  // vy: không đi ngang
  //     0.00f   // wz: không quay
  // );
  //
  // Sau này IMU giữ hướng có thể cộng correction PID trực tiếp theo rad/s:
  //
  // float wzCorrectionRadps = headingPidOutput;
  // robot.driveVelocity(vxMps, vyMps, wzManualRadps + wzCorrectionRadps);
}
