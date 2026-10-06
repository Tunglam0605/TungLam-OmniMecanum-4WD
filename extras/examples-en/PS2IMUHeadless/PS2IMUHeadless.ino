/**
 * @file PS2IMUHeadless.ino
 * @brief Điều khiển đế Mecanum bằng PS2 + HWT901B + Fuzzy PID.
 *
 * MỤC TIÊU
 * ---------------------------------------------------------------------------
 * Ví dụ này tách rõ hai chức năng độc lập:
 *
 * 1) GIỮ HƯỚNG (Heading Hold)
 *    - Khi xe bắt đầu có lệnh tịnh tiến và không có lệnh quay tay, controller
 *      chốt góc yaw của CHU KỲ CONTROL TRƯỚC ĐÓ rồi giữ góc này.
 *    - Tiến, lùi, ngang và chéo đều được giữ heading.
 *    - Khi người lái điều khiển joystick phải X để quay, PID yaw tắt hoàn toàn.
 *    - Khi nhả joystick quay, yaw hiện tại trở thành heading target mới.
 *    - Nút START bật/tắt việc tiếp tục giữ heading khi xe đang đứng yên.
 *
 * 2) XE KHÔNG ĐẦU / HEADLESS / FIELD-CENTRIC
 *    - Nút R1 bật/tắt chế độ.
 *    - Khi OFF: joystick trái điều khiển theo thân robot (body-centric).
 *    - Khi ON: hướng robot tại thời điểm bật R1 được lấy làm hướng 0 độ của sân.
 *      Từ đó joystick trái điều khiển theo hệ sân, không phụ thuộc đầu xe.
 *    - Dù vừa tịnh tiến vừa quay bằng wz, vector dịch chuyển ngoài sân vẫn giữ
 *      đúng hướng người lái đang chỉ trên joystick.
 *    - SELECT lấy lại mốc 0 độ của sân khi Headless đang bật.
 *
 * THỨ TỰ ƯU TIÊN
 * ---------------------------------------------------------------------------
 * 1. Safety: mất PS2 hoặc mất dữ liệu IMU mới -> dừng robot.
 * 2. Manual yaw: joystick phải X có quyền điều khiển wz tuyệt đối.
 * 3. Translation heading hold: đang tịnh tiến, không manual yaw -> PID giữ yaw.
 * 4. Idle heading hold: đứng yên và START bật -> PID giữ yaw.
 * 5. Idle free: đứng yên và START tắt -> không PID.
 *
 * KIẾN TRÚC
 * ---------------------------------------------------------------------------
 *                           PS2
 *                            |
 *              +-------------+-------------+
 *              |             |             |
 *          Joy trái      Joy phải X       R1
 *              |             |             |
 *        Translation      Manual wz    Headless toggle
 *              |             |
 *              v             |
 *         Headless ?         |
 *          /      \          |
 *       OFF        ON         |
 *       |           |         |
 *  body vx/vy   Field -> Body <----- IMU yaw
 *       |           |         |
 *       +-----+-----+         |
 *             |               |
 *             v               v
 *          vx_body         yaw policy
 *          vy_body      manual / PID hold
 *             |               |
 *             +-------+-------+
 *                     |
 *                     v
 *             robot.driveVelocity()
 *
 * GIẢ ĐỊNH PHẦN CỨNG
 * ---------------------------------------------------------------------------
 * - Arduino Mega 2560.
 * - PS2 dùng hardware SPI mặc định: D50/D51/D52 và CS D53.
 * - HWT901B nối Serial1: TX IMU -> RX1 D19, RX IMU -> TX1 D18.
 * - HWT901B đã cấu hình 115200 baud, có ANGLE + GYRO, nên chạy 100 Hz.
 * - +yaw / +gyro Z của IMU cùng chiều +wz robot (CCW).
 *   Nếu ngược chiều, đổi IMU_YAW_SIGN từ +1 thành -1.
 *
 * LƯU Ý
 * ---------------------------------------------------------------------------
 * Drive hiện vẫn là feed-forward vòng hở ở từng bánh nếu chưa có encoder.
 * IMU giúp giữ yaw và thực hiện field-centric, nhưng không đo được trượt ngang.
 * Muốn bám vx/vy chính xác khi tải thay đổi cần PID tốc độ từng bánh bằng encoder.
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

// Vòng điều khiển 100 Hz. PS2 tự poll 50 Hz và giữ state mới nhất.
constexpr uint32_t CONTROL_PERIOD_US = 10000UL;

// Angle/gyro cũ hơn ngưỡng này bị coi là stale.
constexpr uint32_t IMU_DATA_TIMEOUT_MS = 120UL;

// Tốc độ lệnh tối đa.
constexpr float MAX_LINEAR_MPS = 0.55f;
constexpr float MAX_MANUAL_YAW_RADPS = 2.0f;

// Deadzone joystick sau tầng lọc của thư viện PS2.
constexpr float TRANSLATION_DEADZONE = 0.12f;
constexpr float TURN_DEADZONE = 0.15f;

// Ngưỡng logic sau deadzone để xác định có command hay không.
constexpr float COMMAND_EPSILON = 1.0e-5f;

// Đổi thành -1.0f nếu chiều yaw thực tế của IMU ngược quy ước +wz của robot.
constexpr float IMU_YAW_SIGN = 1.0f;

// Bật 1 khi tune. Production nên để 0 để giảm tải Serial.
#define HEADLESS_DEBUG 0

// Baseline mẫu: motor 12 V, 300 rpm, bánh R=50 mm.
// Cần đo và sửa theo đúng robot thật trước khi chạy tốc độ cao.
const TungLamDriveConfig DRIVE_MODEL(
    12.0f,   // điện áp danh định motor [V]
    300.0f,  // RPM đầu ra hộp số
    12.0f,   // điện áp nguồn driver [V]
    0.050f,  // bán kính bánh [m]
    0.320f,  // wheelbase [m]
    0.280f,  // track width [m]
    0.85f    // hệ số hiệu chỉnh feed-forward
);

// ============================================================================
// MODULE
// ============================================================================

TungLamPS2 ps2;
TungLamDrive4WD robot;
HWT901B imu;
TungLamFuzzyPID headingController;

// ============================================================================
// TRẠNG THÁI ĐIỀU KHIỂN
// ============================================================================

// R1 điều khiển flag này.
// false: joystick trái theo thân robot.
// true : joystick trái theo hệ sân.
bool headlessEnabled = false;

// START chỉ điều khiển việc giữ yaw khi xe đang đứng yên.
// Khi đang tịnh tiến, heading hold vẫn tự động hoạt động bất kể flag này.
bool idleHeadingHoldEnabled = false;

// Mốc 0 độ của hệ sân.
// Khi bật Headless bằng R1, yaw hiện tại được lấy làm field zero.
float fieldZeroYawDeg = 0.0f;

// Góc mà PID phải giữ khi heading hold hoạt động.
float headingTargetYawDeg = 0.0f;

// Yaw của CHU KỲ CONTROL TRƯỚC.
// Dùng để chốt heading trước khi motor bắt đầu nhận lệnh tịnh tiến một nhịp.
float previousControlYawDeg = 0.0f;

// Trạng thái command của chu kỳ trước để phát hiện cạnh chuyển trạng thái.
bool previousTranslationActive = false;
bool previousRotationActive = false;

bool referenceReady = false;
uint32_t lastControlUs = 0;

enum class MotionState : uint8_t {
  Idle,
  TranslationHold,
  ManualRotation,
  TranslationWithManualRotation
};

// Command joystick trái sau deadzone.
// xMps/yMps là vector logic người lái muốn.
// Khi Headless OFF: vector này được hiểu trực tiếp theo body.
// Khi Headless ON : vector này được hiểu theo field.
struct TranslationCommand {
  float xMps;
  float yMps;
  float normalizedMagnitude;
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
void handleModeButtons(const TungLamHWT901BData& data);
void runControl(const TungLamHWT901BData& data, float dtSeconds);
void safeStop();

TranslationCommand readTranslationCommand();
BodyTranslation resolveBodyTranslation(const TranslationCommand& command,
                                       float currentYawDeg);
BodyTranslation fieldToBody(const TranslationCommand& field,
                            float currentYawDeg);
float readManualYawCommand();

MotionState determineMotionState(bool translationActive,
                                 bool rotationActive);

float computeHeadingCorrection(float currentYawDeg,
                               float gyroZDps,
                               float dtSeconds);

float axisWithDeadzone(int16_t axis, float deadzone);
float clampf(float value, float minimum, float maximum);
float signedYawDeg(float imuYawDeg);
float signedGyroZDps(float imuGyroZDps);

#if HEADLESS_DEBUG
const char* motionStateName(MotionState state);
void debugTelemetry(const TungLamHWT901BData& data,
                    MotionState state,
                    const TranslationCommand& command,
                    const BodyTranslation& body,
                    float manualWzRadps,
                    float finalWzRadps);
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
  // Các update đều non-blocking. Không dùng delay().
  ps2.update();
  imu.update();
  robot.update();

  TungLamHWT901BData imuData;

  // Cả tay cầm và IMU đều là nguồn bắt buộc của template này.
  if (!ps2.connected() || !readFreshImu(imuData)) {
    safeStop();
    return;
  }

  if (!referenceReady) {
    initializeReference(imuData);
    return;
  }

  // Xử lý event nút trước gate 100 Hz để không bỏ lỡ cạnh pressed của PS2.
  handleModeButtons(imuData);

  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - lastControlUs;

  if (elapsedUs < CONTROL_PERIOD_US) {
    return;
  }

  float dtSeconds = elapsedUs * 1.0e-6f;
  lastControlUs = nowUs;

  const float currentYawDeg = signedYawDeg(imuData.yaw_deg);

  // Nếu scheduler từng trễ >50 ms, không cho PID tích phân/đạo hàm với dt bất thường.
  // Reset lịch sử state để vòng sau coi đây là một khởi đầu sạch.
  if (dtSeconds > 0.050f) {
    headingTargetYawDeg = currentYawDeg;
    previousControlYawDeg = currentYawDeg;
    previousTranslationActive = false;
    previousRotationActive = false;
    headingController.reset();
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

  // Watchdog lệnh + giới hạn gia tốc để giảm bước lệnh quá gắt.
  robot.enableSmartSafety(
      250,   // command timeout [ms]
      1.5f,  // linear acceleration limit [m/s^2]
      4.0f   // yaw acceleration limit [rad/s^2]
  );
}

void setupController() {
  ps2.begin(PS2_CS_PIN);
  ps2.setPollRate(PS2PollRate::Hz50);
}

void setupImu() {
  // Ví dụ giả định HWT901B đã được cấu hình 115200 baud.
  // Nếu cảm biến còn 9600 baud, hãy dùng example cấu hình của TungLam_HWT901B
  // để đổi baud/rate/output một lần trước khi chạy sketch này.
  imu.begin(Serial1, IMU_BAUD);
}

void setupHeadingController() {
  // PID nền. Output được tune trực tiếp thành rad/s cho driveVelocity().
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

  // Chỉ cho I-term hoạt động gần target để hạn chế windup.
  headingController.setIntegralZone(12.0f);

  // Tránh rung sửa các sai số yaw rất nhỏ.
  headingController.setDeadband(0.4f);

  // Lọc D-term. D-term sẽ lấy measurement rate trực tiếp từ gyro Z.
  headingController.setDerivativeFilter(0.25f);

  // Fuzzy chỉ schedule Kp/Ki/Kd quanh PID nền, không thay thế control law PID.
  headingController.enableFuzzy(
      45.0f,    // biên |error| fuzzy [deg]
      120.0f,   // biên |dError/dt| fuzzy [deg/s]
      0.015f,   // biên hiệu chỉnh Kp
      0.0010f,  // biên hiệu chỉnh Ki
      0.0030f   // biên hiệu chỉnh Kd
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

  // Khởi động/reconnect:
  // - yaw hiện tại là heading an toàn ban đầu;
  // - yaw hiện tại cũng là field zero dự phòng.
  // Headless mặc định vẫn OFF cho tới khi người lái bấm R1.
  fieldZeroYawDeg = yawDeg;
  headingTargetYawDeg = yawDeg;
  previousControlYawDeg = yawDeg;

  previousTranslationActive = false;
  previousRotationActive = false;

  headlessEnabled = false;
  idleHeadingHoldEnabled = false;

  headingController.reset();

  referenceReady = true;
  lastControlUs = micros();
}

void handleModeButtons(const TungLamHWT901BData& data) {
  const float currentYawDeg = signedYawDeg(data.yaw_deg);

  // --------------------------------------------------------------------------
  // R1: TOGGLE HEADLESS
  // --------------------------------------------------------------------------
  if (ps2.pressed(PS2Button::R1)) {
    headlessEnabled = !headlessEnabled;

    if (headlessEnabled) {
      // Khi vừa bật, hướng thân hiện tại trở thành 0 độ của sân.
      // Vì relativeYaw = 0 ngay tại thời điểm bật nên command tịnh tiến không bị
      // đổi hướng đột ngột chỉ vì chuyển từ body-centric sang field-centric.
      fieldZeroYawDeg = currentYawDeg;
    }
  }

  // --------------------------------------------------------------------------
  // START: TOGGLE GIỮ HƯỚNG KHI ĐỨNG YÊN
  // --------------------------------------------------------------------------
  if (ps2.pressed(PS2Button::Start)) {
    idleHeadingHoldEnabled = !idleHeadingHoldEnabled;

    if (idleHeadingHoldEnabled) {
      // Nếu lần control trước xe đang thực sự idle, khóa đúng yaw hiện tại.
      // Nếu đang chạy/quay, flag chỉ có tác dụng khi xe trở về idle.
      if (!previousTranslationActive && !previousRotationActive) {
        headingTargetYawDeg = currentYawDeg;
        headingController.reset();
      }
    } else {
      // Tắt idle hold không được làm mất translation hold.
      // Chỉ reset ngay nếu vòng trước đang đứng yên.
      if (!previousTranslationActive && !previousRotationActive) {
        headingController.reset();
      }
    }
  }

  // --------------------------------------------------------------------------
  // SELECT: RE-ZERO HỆ SÂN KHI HEADLESS ĐANG BẬT
  // --------------------------------------------------------------------------
  if (headlessEnabled && ps2.pressed(PS2Button::Select)) {
    fieldZeroYawDeg = currentYawDeg;
  }
}

void safeStop() {
  robot.stop();

  // Sau khi PS2/IMU trở lại, lấy reference mới tại tư thế hiện tại.
  // Không quay bất ngờ về target cũ trước khi mất kết nối.
  referenceReady = false;

  previousTranslationActive = false;
  previousRotationActive = false;

  headingController.reset();
  lastControlUs = micros();
}

// ============================================================================
// CONTROL PIPELINE
// ============================================================================

void runControl(const TungLamHWT901BData& data, float dtSeconds) {
  const float currentYawDeg = signedYawDeg(data.yaw_deg);
  const float gyroZDps = signedGyroZDps(data.gz_dps);

  // 1) Đọc vector joystick trái sau radial deadzone.
  const TranslationCommand translation = readTranslationCommand();

  const bool translationActive =
      translation.normalizedMagnitude > COMMAND_EPSILON;

  // 2) R1 quyết định cách diễn giải vector:
  //    OFF -> body-centric.
  //    ON  -> field-centric rồi quay về body bằng yaw IMU hiện tại.
  const BodyTranslation body =
      resolveBodyTranslation(translation, currentYawDeg);

  // 3) Đọc manual yaw từ joystick phải.
  const float manualWzRadps = readManualYawCommand();
  const bool rotationActive =
      fabsf(manualWzRadps) > COMMAND_EPSILON;

  const MotionState state =
      determineMotionState(translationActive, rotationActive);

  float finalWzRadps = 0.0f;

  // ==========================================================================
  // ƯU TIÊN 1 SAU SAFETY: MANUAL WZ
  // ==========================================================================
  if (state == MotionState::ManualRotation ||
      state == MotionState::TranslationWithManualRotation) {
    // Cạnh bắt đầu manual rotation: xóa toàn bộ trạng thái PID cũ để controller
    // không chống lại người lái bằng I-term/D-history từ heading trước.
    if (!previousRotationActive) {
      headingController.reset();
    }

    // Khi có manual wz, PID yaw OFF hoàn toàn.
    finalWzRadps = manualWzRadps;
  }

  // ==========================================================================
  // KHÔNG CÓ MANUAL WZ: HEADING HOLD TÙY TRẠNG THÁI
  // ==========================================================================
  else {
    // Vừa nhả joystick quay:
    // góc HIỆN TẠI chính là heading mới mà người lái vừa chọn.
    if (previousRotationActive) {
      headingTargetYawDeg = currentYawDeg;
      headingController.reset();
    }

    // Bắt đầu tịnh tiến từ trạng thái trước đó không tịnh tiến, và cũng không
    // phải vừa nhả manual rotation:
    //
    // CHỐT YAW CỦA CHU KỲ CONTROL TRƯỚC.
    //
    // Đây là chủ đích: không đợi motor đã bắt đầu tạo vận tốc rồi mới lấy góc,
    // nhờ vậy controller chống ngay xu hướng lệch đầu tiên do motor/bánh không đều.
    else if (translationActive && !previousTranslationActive) {
      headingTargetYawDeg = previousControlYawDeg;
      headingController.reset();
    }

    if (state == MotionState::TranslationHold) {
      // Tiến/lùi/ngang/chéo đều tự động giữ heading.
      finalWzRadps =
          computeHeadingCorrection(currentYawDeg, gyroZDps, dtSeconds);
    }
    else if (idleHeadingHoldEnabled) {
      // START ON: xe đứng yên vẫn khóa hướng.
      finalWzRadps =
          computeHeadingCorrection(currentYawDeg, gyroZDps, dtSeconds);
    }
    else {
      // START OFF: xe đứng yên tự do, không PID.
      finalWzRadps = 0.0f;

      // Chỉ reset ở cạnh vừa kết thúc translation để không giữ I-term cũ.
      if (previousTranslationActive) {
        headingController.reset();
      }
    }
  }

  // 4) Gửi command cuối cùng xuống tầng Mecanum.
  //
  // Trong Headless ON, body vx/vy được tính lại bằng current yaw ở MỖI chu kỳ.
  // Vì vậy ngay cả khi manualWzRadps != 0 và thân robot đang xoay liên tục,
  // vector dịch chuyển ngoài sân vẫn giữ đúng hướng joystick.
  robot.driveVelocity(
      body.vxBodyMps,
      body.vyBodyMps,
      finalWzRadps
  );

#if HEADLESS_DEBUG
  debugTelemetry(
      data,
      state,
      translation,
      body,
      manualWzRadps,
      finalWzRadps
  );
#endif

  // 5) Lưu trạng thái chu kỳ này để phát hiện transition ở chu kỳ kế tiếp.
  previousTranslationActive = translationActive;
  previousRotationActive = rotationActive;

  // Quan trọng: lưu yaw SAU KHI đã xử lý toàn bộ command của chu kỳ hiện tại.
  // Nếu chu kỳ sau vừa phát hiện bắt đầu translation, biến này chính là yaw của
  // nhịp ngay trước khi xe có lệnh vận tốc.
  previousControlYawDeg = currentYawDeg;
}

MotionState determineMotionState(bool translationActive,
                                 bool rotationActive) {
  if (!translationActive && !rotationActive) {
    return MotionState::Idle;
  }

  if (translationActive && !rotationActive) {
    return MotionState::TranslationHold;
  }

  if (!translationActive && rotationActive) {
    return MotionState::ManualRotation;
  }

  return MotionState::TranslationWithManualRotation;
}

float computeHeadingCorrection(float currentYawDeg,
                               float gyroZDps,
                               float dtSeconds) {
  // Fuzzy PID dùng:
  // - error góc có wrap ±180°;
  // - measurement rate trực tiếp từ gyro Z cho D-term.
  //
  // Với target cố định:
  // d(error)/dt = -d(yaw)/dt ≈ -gyroZ.
  return headingController.computeWithMeasurementRate(
      headingTargetYawDeg,
      currentYawDeg,
      gyroZDps,
      dtSeconds
  );
}

// ============================================================================
// PS2 -> TRANSLATION COMMAND
// ============================================================================

TranslationCommand readTranslationCommand() {
  // PS2 library:
  //   leftX > 0 = gạt sang phải
  //   leftY < 0 = gạt lên
  //
  // Quy ước command:
  //   +X = tiến
  //   +Y = trái
  float forward =
      clampf(-ps2.leftY() / 127.0f, -1.0f, 1.0f);

  float left =
      clampf(-ps2.leftX() / 127.0f, -1.0f, 1.0f);

  const float magnitude =
      sqrtf(forward * forward + left * left);

  TranslationCommand result = {
      0.0f,
      0.0f,
      0.0f
  };

  if (magnitude <= TRANSLATION_DEADZONE) {
    return result;
  }

  // Radial deadzone:
  // - giữ nguyên góc vector joystick;
  // - bỏ vùng tâm tròn;
  // - rescale phần còn lại về 0..1.
  const float limitedMagnitude =
      magnitude > 1.0f ? 1.0f : magnitude;

  const float directionScale = 1.0f / magnitude;

  const float rescaledMagnitude =
      (limitedMagnitude - TRANSLATION_DEADZONE) /
      (1.0f - TRANSLATION_DEADZONE);

  result.xMps =
      forward * directionScale *
      rescaledMagnitude * MAX_LINEAR_MPS;

  result.yMps =
      left * directionScale *
      rescaledMagnitude * MAX_LINEAR_MPS;

  result.normalizedMagnitude = rescaledMagnitude;

  return result;
}

float readManualYawCommand() {
  // rightX > 0 khi gạt joystick sang phải.
  // Quy ước Drive: +wz = CCW/trái.
  // Vì vậy gạt phải phải tạo -wz (CW).
  const float rightX =
      axisWithDeadzone(ps2.rightX(), TURN_DEADZONE);

  return -rightX * MAX_MANUAL_YAW_RADPS;
}

// ============================================================================
// BODY-CENTRIC / HEADLESS
// ============================================================================

BodyTranslation resolveBodyTranslation(
    const TranslationCommand& command,
    float currentYawDeg) {

  if (!headlessEnabled) {
    // BODY-CENTRIC:
    // joystick luôn bám theo đầu/thân robot.
    BodyTranslation result;
    result.vxBodyMps = command.xMps;
    result.vyBodyMps = command.yMps;
    return result;
  }

  // HEADLESS / FIELD-CENTRIC:
  // command.x/y được hiểu là vector cố định ngoài sân.
  // IMU yaw hiện tại chỉ được dùng để đổi vector field -> body.
  return fieldToBody(command, currentYawDeg);
}

BodyTranslation fieldToBody(const TranslationCommand& field,
                            float currentYawDeg) {
  // Góc thân robot so với mốc 0 độ của sân đã chốt khi bật R1/SELECT.
  const float relativeYawDeg =
      TungLamFuzzyPID::normalizeDegrees(
          currentYawDeg - fieldZeroYawDeg
      );

  const float yawRad =
      relativeYawDeg * 0.01745329251994329577f;

  const float c = cosf(yawRad);
  const float s = sinf(yawRad);

  // R(-yaw):
  //
  // [vx_body]   [ cos(yaw)  sin(yaw)] [vx_field]
  // [vy_body] = [-sin(yaw)  cos(yaw)] [vy_field]
  //
  // Ví dụ joystick giữ "tiến theo sân":
  // yaw 0°   -> robot tiến theo thân.
  // yaw 90°  -> robot đi ngang theo thân.
  // yaw 180° -> robot lùi theo thân.
  // Nhưng ngoài sân vector chuyển động vẫn giữ nguyên.
  BodyTranslation result;

  result.vxBodyMps =
      c * field.xMps +
      s * field.yMps;

  result.vyBodyMps =
      -s * field.xMps +
       c * field.yMps;

  return result;
}

// ============================================================================
// HELPER
// ============================================================================

float axisWithDeadzone(int16_t axis, float deadzone) {
  float normalized =
      clampf(axis / 127.0f, -1.0f, 1.0f);

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

// ============================================================================
// DEBUG
// ============================================================================

#if HEADLESS_DEBUG

const char* motionStateName(MotionState state) {
  switch (state) {
    case MotionState::Idle:
      return "IDLE";

    case MotionState::TranslationHold:
      return "TRANSLATION_HOLD";

    case MotionState::ManualRotation:
      return "MANUAL_ROTATION";

    case MotionState::TranslationWithManualRotation:
      return "TRANSLATION_ROTATE";
  }

  return "UNKNOWN";
}

void debugTelemetry(const TungLamHWT901BData& data,
                    MotionState state,
                    const TranslationCommand& command,
                    const BodyTranslation& body,
                    float manualWzRadps,
                    float finalWzRadps) {
  static uint32_t lastPrintMs = 0;
  const uint32_t nowMs = millis();

  if (nowMs - lastPrintMs < 100UL) {
    return;
  }

  lastPrintMs = nowMs;

  const TungLamFuzzyPIDStatus pid = headingController.status();

  Serial.print("mode=");
  Serial.print(headlessEnabled ? "HEADLESS" : "BODY");

  Serial.print(" idleHold=");
  Serial.print(idleHeadingHoldEnabled ? "ON" : "OFF");

  Serial.print(" state=");
  Serial.print(motionStateName(state));

  Serial.print(" yaw=");
  Serial.print(signedYawDeg(data.yaw_deg), 2);

  Serial.print(" target=");
  Serial.print(headingTargetYawDeg, 2);

  Serial.print(" field0=");
  Serial.print(fieldZeroYawDeg, 2);

  Serial.print(" e=");
  Serial.print(pid.error, 2);

  Serial.print(" | cmd=(");
  Serial.print(command.xMps, 2);
  Serial.print(",");
  Serial.print(command.yMps, 2);
  Serial.print(")");

  Serial.print(" body=(");
  Serial.print(body.vxBodyMps, 2);
  Serial.print(",");
  Serial.print(body.vyBodyMps, 2);
  Serial.print(")");

  Serial.print(" manualWz=");
  Serial.print(manualWzRadps, 2);

  Serial.print(" wz=");
  Serial.print(finalWzRadps, 2);

  Serial.print(" K=(");
  Serial.print(pid.activeGains.kp, 4);
  Serial.print(",");
  Serial.print(pid.activeGains.ki, 4);
  Serial.print(",");
  Serial.print(pid.activeGains.kd, 4);
  Serial.println(")");
}

#endif
