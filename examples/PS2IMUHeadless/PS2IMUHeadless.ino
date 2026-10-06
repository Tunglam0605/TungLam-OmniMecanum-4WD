/**
 * @file PS2IMUHeadless.ino
 * @brief Đế Mecanum không đầu: PS2 + HWT901B + Fuzzy PID + TungLam Drive.
 *
 * MỤC TIÊU
 * ---------------------------------------------------------------------------
 * - Joystick trái ra lệnh tịnh tiến theo HỆ SÂN (field frame), không phụ thuộc
 *   đầu robot đang quay về hướng nào.
 * - Joystick phải X quay robot thủ công.
 * - Khi thả joystick phải, robot tự khóa heading hiện tại bằng PID/Fuzzy PID.
 * - Có thể vừa tịnh tiến vừa quay; vector tịnh tiến ngoài sân vẫn giữ hướng.
 * - Mất PS2 hoặc mất dữ liệu IMU mới -> dừng robot fail-safe.
 *
 * KIẾN TRÚC
 * ---------------------------------------------------------------------------
 * PS2 joystick trái -> vx_field, vy_field
 *                                |
 * IMU yaw -----------------------+--> Field -> Body transform
 *                                |             |
 * PS2 joystick phải -> yaw cmd   |             +--> vx_body, vy_body
 *              |                 |
 *              +-- manual turn --+
 *              |
 *              +-- thả cần -> Heading PID/Fuzzy PID -> wz
 *
 * vx_body + vy_body + wz
 *          |
 *          v
 * robot.driveVelocity()
 *
 * GIẢ ĐỊNH PHẦN CỨNG
 * ---------------------------------------------------------------------------
 * - Arduino Mega 2560.
 * - PS2 dùng hardware SPI mặc định: D50/D51/D52 và CS D53.
 * - HWT901B nối Serial1: TX IMU -> RX1 D19, RX IMU -> TX1 D18.
 * - HWT901B đã được cấu hình sẵn 115200 baud, có ANGLE + GYRO và nên chạy 100 Hz.
 * - Trục Z/yaw của IMU cùng chiều dương CCW với +wz của robot.
 *   Nếu ngược chiều, đổi IMU_YAW_SIGN từ +1 thành -1.
 * - Thông số motor/chassis bên dưới chỉ là baseline; phải thay theo robot thật.
 *
 * LƯU Ý ĐIỀU KHIỂN
 * ---------------------------------------------------------------------------
 * Drive hiện là feed-forward vòng hở ở từng bánh nếu chưa có encoder.
 * Heading PID sửa sai yaw của toàn thân nhưng không biến 4 bánh thành velocity
 * closed-loop. Muốn bám vx/vy chính xác hơn cần encoder PID từng bánh ở tầng dưới.
 */

#include <math.h>

#include <TungLam_FuzzyPID.h>
#include <TungLam_HWT901B.h>
#include <TungLam_OmniMecanum_4WD.h>
#include <TungLam_PS2.h>

// ============================================================================
// CẤU HÌNH NGƯỜI DÙNG
// ============================================================================

constexpr uint8_t PS2_CS_PIN = 53;
constexpr uint32_t IMU_BAUD = 115200UL;

// Vòng heading/control 100 Hz. PS2 vẫn tự poll 50 Hz và giữ state mới nhất.
constexpr uint32_t CONTROL_PERIOD_US = 10000UL;

// Dữ liệu angle/gyro cũ hơn ngưỡng này bị coi là không an toàn để headless.
constexpr uint32_t IMU_DATA_TIMEOUT_MS = 120UL;

// Tốc độ lệnh tối đa từ tay cầm.
constexpr float MAX_LINEAR_MPS = 0.55f;
constexpr float MAX_MANUAL_YAW_RADPS = 2.0f;

// Deadzone sau khi PS2 library đã lọc/trừ tâm.
constexpr float TRANSLATION_DEADZONE = 0.12f;
constexpr float TURN_DEADZONE = 0.15f;

// Đặt -1.0f nếu yaw/gyro Z của IMU tăng theo chiều CW so với quy ước robot.
constexpr float IMU_YAW_SIGN = 1.0f;

// Bật 1 khi tune; production nên để 0 để không làm Serial ảnh hưởng timing.
#define HEADLESS_DEBUG 0

// Baseline mẫu cho motor 12 V, 300 rpm, bánh R=50 mm.
// Hãy đo và sửa đúng với robot thật trước khi chạy tốc độ cao.
const TungLamDriveConfig DRIVE_MODEL(
    12.0f,   // điện áp danh định motor [V]
    300.0f,  // RPM đầu ra hộp số tại điện áp danh định
    12.0f,   // điện áp nguồn driver [V]
    0.050f,  // bán kính bánh [m]
    0.320f,  // wheelbase [m]
    0.280f,  // track width [m]
    0.85f    // hiệu chỉnh vòng hở
);

// ============================================================================
// MODULE
// ============================================================================

TungLamPS2 ps2;
TungLamDrive4WD robot;
HWT901B imu;
TungLamFuzzyPID headingController;

// Mốc hệ sân: yaw robot tại thời điểm lấy "đầu sân".
float fieldZeroYawDeg = 0.0f;

// Heading mà robot phải giữ khi joystick phải đang ở Center.
float headingTargetYawDeg = 0.0f;

bool referenceReady = false;
bool manualTurning = false;
uint32_t lastControlUs = 0;

struct TranslationCommand {
  float vxFieldMps;
  float vyFieldMps;
};

struct BodyTranslation {
  float vxBodyMps;
  float vyBodyMps;
};

// ============================================================================
// KHAI BÁO HÀM
// ============================================================================

void setupDrive();
void setupController();
void setupImu();
void setupHeadingController();

bool readFreshImu(TungLamHWT901BData& data);
void initializeReference(const TungLamHWT901BData& data);
void handleReferenceButton(const TungLamHWT901BData& data);
void runControl(const TungLamHWT901BData& data, float dtSeconds);
void safeStop();

TranslationCommand readTranslationCommand();
BodyTranslation fieldToBody(const TranslationCommand& field,
                            float currentYawDeg);
float readManualYawCommand();
float axisWithDeadzone(int16_t axis, float deadzone);
float clampf(float value, float minimum, float maximum);
float signedYawDeg(float imuYawDeg);
float signedGyroZDps(float imuGyroZDps);

#if HEADLESS_DEBUG
void debugTelemetry(const TungLamHWT901BData& data,
                    const TranslationCommand& field,
                    const BodyTranslation& body,
                    float wzRadps);
#endif

// ============================================================================
// SETUP / LOOP
// ============================================================================

void setup() {
#if HEADLESS_DEBUG
  Serial.begin(115200);
#endif

  setupDrive();
  setupController();
  setupImu();
  setupHeadingController();

  robot.stop();
}

void loop() {
  // Ba update này đều phải được gọi liên tục, không dùng delay().
  ps2.update();
  imu.update();
  robot.update();

  TungLamHWT901BData imuData;

  // Headless phụ thuộc cả controller và heading sensor.
  // Không có một trong hai nguồn -> dừng thay vì giữ lệnh cũ.
  if (!ps2.connected() || !readFreshImu(imuData)) {
    safeStop();
    return;
  }

  if (!referenceReady) {
    initializeReference(imuData);
    return;
  }

  handleReferenceButton(imuData);

  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - lastControlUs;

  if (elapsedUs < CONTROL_PERIOD_US) {
    return;
  }

  float dtSeconds = elapsedUs * 1.0e-6f;
  lastControlUs = nowUs;

  // Nếu loop từng bị treo/chậm quá lâu, không cho PID dùng một dt bất thường.
  // Đồng bộ lại target tại heading hiện tại rồi bắt đầu vòng điều khiển mới.
  if (dtSeconds > 0.050f) {
    headingTargetYawDeg = signedYawDeg(imuData.yaw_deg);
    headingController.reset();
    manualTurning = false;
    dtSeconds = CONTROL_PERIOD_US * 1.0e-6f;
  }

  runControl(imuData, dtSeconds);
}

// ============================================================================
// KHỞI TẠO TỪNG MODULE
// ============================================================================

void setupDrive() {
  robot.begin(TungLamPwmMode::High7k8Hz);
  robot.setChassis(TungLamChassis::MecanumX);
  robot.setDriveConfig(DRIVE_MODEL);

  // Watchdog 250 ms + ramp vận tốc giúp tránh bước lệnh quá gắt.
  robot.enableSmartSafety(
      250,   // timeout lệnh [ms]
      1.5f,  // gia tốc tịnh tiến tối đa [m/s^2]
      4.0f   // gia tốc yaw tối đa [rad/s^2]
  );
}

void setupController() {
  ps2.begin(PS2_CS_PIN);
  ps2.setPollRate(PS2PollRate::Hz50);
}

void setupImu() {
  // QUAN TRỌNG:
  // Ví dụ giả định HWT901B đã được cấu hình 115200 baud trước đó.
  // Nếu cảm biến vẫn là 9600, hãy dùng example cấu hình của TungLam_HWT901B
  // để đổi baud/output/rate một lần rồi mới chạy sketch này.
  imu.begin(Serial1, IMU_BAUD);
}

void setupHeadingController() {
  // Đơn vị output được tune trực tiếp thành rad/s cho driveVelocity().
  headingController.begin(
      0.035f,   // Kp
      0.0015f,  // Ki
      0.0050f   // Kd
  );

  headingController.useDegreeAngleDomain();

  headingController.setOutputLimits(
      -2.2f,
      +2.2f
  );

  headingController.setIntegralLimits(
      -0.40f,
      +0.40f
  );

  // Chỉ tích phân gần setpoint để hạn chế windup.
  headingController.setIntegralZone(12.0f);

  // Tránh robot rung sửa những sai số heading rất nhỏ.
  headingController.setDeadband(0.4f);

  // 0.25: có lọc D-term nhưng vẫn đủ nhanh cho vòng 100 Hz.
  headingController.setDerivativeFilter(0.25f);

  // Fuzzy chỉ schedule gain quanh PID nền, không thay thế PID.
  headingController.enableFuzzy(
      45.0f,    // |error| tại biên fuzzy [deg]
      120.0f,   // |dError/dt| tại biên fuzzy [deg/s]
      0.015f,   // biên thay đổi Kp
      0.0010f,  // biên thay đổi Ki
      0.0030f   // biên thay đổi Kd
  );
}

// ============================================================================
// HEALTH / REFERENCE
// ============================================================================

bool readFreshImu(TungLamHWT901BData& data) {
  if (!imu.getData(data)) {
    return false;
  }

  if (!data.has_angle || !data.has_gyro) {
    return false;
  }

  const uint32_t nowMs = millis();

  const bool angleFresh =
      (nowMs - data.last_angle_ms) <= IMU_DATA_TIMEOUT_MS;
  const bool gyroFresh =
      (nowMs - data.last_gyro_ms) <= IMU_DATA_TIMEOUT_MS;

  return angleFresh && gyroFresh;
}

void initializeReference(const TungLamHWT901BData& data) {
  const float yawDeg = signedYawDeg(data.yaw_deg);

  // Tư thế robot lúc khởi động/reconnect được coi là +X của hệ sân.
  fieldZeroYawDeg = yawDeg;

  // Đồng thời giữ đúng heading này khi người lái không ra lệnh quay.
  headingTargetYawDeg = yawDeg;

  headingController.reset();
  manualTurning = false;
  referenceReady = true;
  lastControlUs = micros();
}

void handleReferenceButton(const TungLamHWT901BData& data) {
  // SELECT = lấy lại "đầu sân" tại đúng tư thế hiện tại.
  // Không sửa offset bên trong IMU; field reference thuộc application layer.
  if (ps2.pressed(PS2Button::Select)) {
    const float yawDeg = signedYawDeg(data.yaw_deg);

    fieldZeroYawDeg = yawDeg;
    headingTargetYawDeg = yawDeg;

    headingController.reset();
    manualTurning = false;
    lastControlUs = micros();
  }
}

void safeStop() {
  robot.stop();

  // Khi link trở lại, bắt đầu bằng một reference mới tại tư thế hiện tại.
  // Cách này tránh robot bất ngờ quay về heading cũ sau sự cố.
  referenceReady = false;
  manualTurning = false;
  headingController.reset();
  lastControlUs = micros();
}

// ============================================================================
// CONTROL PIPELINE
// ============================================================================

void runControl(const TungLamHWT901BData& data, float dtSeconds) {
  const float currentYawDeg = signedYawDeg(data.yaw_deg);
  const float gyroZDps = signedGyroZDps(data.gz_dps);

  // 1) Joystick trái sinh vector tịnh tiến theo HỆ SÂN.
  const TranslationCommand field = readTranslationCommand();

  // 2) Dùng yaw để đổi vector hệ sân -> hệ thân robot.
  const BodyTranslation body = fieldToBody(field, currentYawDeg);

  // 3) Joystick phải điều khiển yaw thủ công.
  const float manualWzRadps = readManualYawCommand();
  const bool turningNow = fabsf(manualWzRadps) > 1.0e-5f;

  float wzRadps = 0.0f;

  if (turningNow) {
    // Khi người lái chủ động quay, không để PID "đánh nhau" với joystick.
    if (!manualTurning) {
      headingController.reset();
    }

    wzRadps = manualWzRadps;

    // Target bám theo heading hiện tại để khi thả cần, robot khóa ngay hướng mới.
    headingTargetYawDeg = currentYawDeg;
  } else {
    if (manualTurning) {
      // Chốt chính xác heading tại thời điểm vừa thả joystick phải.
      headingTargetYawDeg = currentYawDeg;
      headingController.reset();
    }

    // D-term dùng gyro Z trực tiếp thay vì lấy vi phân yaw.
    wzRadps = headingController.computeWithMeasurementRate(
        headingTargetYawDeg,
        currentYawDeg,
        gyroZDps,
        dtSeconds
    );
  }

  manualTurning = turningNow;

  // 4) Mixer/kinematics của thư viện đế nhận body velocity cuối cùng.
  robot.driveVelocity(
      body.vxBodyMps,
      body.vyBodyMps,
      wzRadps
  );

#if HEADLESS_DEBUG
  debugTelemetry(data, field, body, wzRadps);
#endif
}

// ============================================================================
// PS2 -> FIELD COMMAND
// ============================================================================

TranslationCommand readTranslationCommand() {
  // PS2:
  //   leftX > 0 = gạt sang phải
  //   leftY < 0 = gạt lên
  //
  // Hệ sân của robot:
  //   +X = tiến theo sân
  //   +Y = trái theo sân
  float forward = clampf(-ps2.leftY() / 127.0f, -1.0f, 1.0f);
  float left = clampf(-ps2.leftX() / 127.0f, -1.0f, 1.0f);

  const float magnitude = sqrtf(forward * forward + left * left);

  TranslationCommand result = {0.0f, 0.0f};

  if (magnitude <= TRANSLATION_DEADZONE) {
    return result;
  }

  // Giữ đúng góc vector joystick, chỉ loại deadzone theo bán kính.
  const float limitedMagnitude = magnitude > 1.0f ? 1.0f : magnitude;
  const float directionScale = 1.0f / magnitude;
  const float rescaledMagnitude =
      (limitedMagnitude - TRANSLATION_DEADZONE) /
      (1.0f - TRANSLATION_DEADZONE);

  result.vxFieldMps =
      forward * directionScale * rescaledMagnitude * MAX_LINEAR_MPS;
  result.vyFieldMps =
      left * directionScale * rescaledMagnitude * MAX_LINEAR_MPS;

  return result;
}

float readManualYawCommand() {
  // rightX > 0 = gạt phải.
  // Robot +wz = CCW/trái, nên gạt phải phải tạo wz âm (CW).
  const float rightX = axisWithDeadzone(ps2.rightX(), TURN_DEADZONE);
  return -rightX * MAX_MANUAL_YAW_RADPS;
}

// ============================================================================
// FIELD -> BODY TRANSFORM
// ============================================================================

BodyTranslation fieldToBody(const TranslationCommand& field,
                            float currentYawDeg) {
  // Heading tương đối giữa thân robot và hệ sân đã chốt.
  const float relativeYawDeg =
      TungLamFuzzyPID::normalizeDegrees(
          currentYawDeg - fieldZeroYawDeg
      );

  const float yawRad = relativeYawDeg * 0.01745329251994329577f;
  const float c = cosf(yawRad);
  const float s = sinf(yawRad);

  // R(-yaw):
  // [vx_body]   [ cos(yaw)  sin(yaw)] [vx_field]
  // [vy_body] = [-sin(yaw)  cos(yaw)] [vy_field]
  BodyTranslation result;

  result.vxBodyMps =
      c * field.vxFieldMps +
      s * field.vyFieldMps;

  result.vyBodyMps =
      -s * field.vxFieldMps +
       c * field.vyFieldMps;

  return result;
}

// ============================================================================
// HELPER
// ============================================================================

float axisWithDeadzone(int16_t axis, float deadzone) {
  float normalized = clampf(axis / 127.0f, -1.0f, 1.0f);
  const float magnitude = fabsf(normalized);

  if (magnitude <= deadzone) {
    return 0.0f;
  }

  const float rescaled =
      (magnitude - deadzone) /
      (1.0f - deadzone);

  return normalized < 0.0f ? -rescaled : rescaled;
}

float clampf(float value, float minimum, float maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

float signedYawDeg(float imuYawDeg) {
  return TungLamFuzzyPID::normalizeDegrees(
      imuYawDeg * IMU_YAW_SIGN
  );
}

float signedGyroZDps(float imuGyroZDps) {
  return imuGyroZDps * IMU_YAW_SIGN;
}

#if HEADLESS_DEBUG
void debugTelemetry(const TungLamHWT901BData& data,
                    const TranslationCommand& field,
                    const BodyTranslation& body,
                    float wzRadps) {
  static uint32_t lastPrintMs = 0;
  const uint32_t nowMs = millis();

  if (nowMs - lastPrintMs < 100UL) {
    return;
  }

  lastPrintMs = nowMs;

  const TungLamFuzzyPIDStatus pid = headingController.status();

  Serial.print("yaw=");
  Serial.print(signedYawDeg(data.yaw_deg), 2);
  Serial.print(" target=");
  Serial.print(headingTargetYawDeg, 2);
  Serial.print(" e=");
  Serial.print(pid.error, 2);

  Serial.print(" | field=(");
  Serial.print(field.vxFieldMps, 2);
  Serial.print(",");
  Serial.print(field.vyFieldMps, 2);
  Serial.print(")");

  Serial.print(" body=(");
  Serial.print(body.vxBodyMps, 2);
  Serial.print(",");
  Serial.print(body.vyBodyMps, 2);
  Serial.print(")");

  Serial.print(" wz=");
  Serial.print(wzRadps, 2);

  Serial.print(" K=(");
  Serial.print(pid.activeGains.kp, 4);
  Serial.print(",");
  Serial.print(pid.activeGains.ki, 4);
  Serial.print(",");
  Serial.print(pid.activeGains.kd, 4);
  Serial.println(")");
}
#endif
