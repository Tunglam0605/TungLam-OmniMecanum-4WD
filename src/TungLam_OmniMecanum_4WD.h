/*==============================================================================
  TUNGLAM OMNI / MECANUM 4WD MOTOR LIBRARY
  ==============================================================================
  TÁC GIẢ
  ------------------------------------------------------------------------------
  Họ và tên : Nguyễn Khắc Tùng Lâm
  Lớp       : DHTD16A2CL
  MSSV      : 2210430016
  Thương hiệu: Tung Lâm Automation

  MỤC TIÊU
  ------------------------------------------------------------------------------
  - Điều khiển đế robot 4 bánh Mecanum-X / Omni-X.
  - Giữ tương thích với TungLam_Control_MotorV5.
  - Hỗ trợ PWM trực tiếp, động học vx/vy/wz và vận tốc SI.
  - Hỗ trợ ABS hãm ngược hardware-timed.
  - Ưu tiên tài liệu/comment tiếng Việt cho học sinh và sinh viên Việt Nam.

  SƠ ĐỒ MOTOR
  ------------------------------------------------------------------------------
                    ĐẦU XE / FRONT
                         +X
                          ^

             M1                        M3
        TRƯỚC-TRÁI                TRƯỚC-PHẢI
           PWM D5                    PWM D7

             M2                        M4
          SAU-TRÁI                 SAU-PHẢI
           PWM D6                    PWM D8

  CHÂN PWM / DIR
  ------------------------------------------------------------------------------
    M1: PWM D5   | tiến D30 | lùi D31
    M2: PWM D6   | tiến D32 | lùi D33
    M3: PWM D7   | tiến D34 | lùi D35
    M4: PWM D8   | tiến D37 | lùi D36

  HỆ TỌA ĐỘ MODERN API
  ------------------------------------------------------------------------------
    +vx = tiến
    -vx = lùi
    +vy = trái
    -vy = phải
    +wz = quay trái / CCW
    -wz = quay phải / CW

  LƯU Ý
  ------------------------------------------------------------------------------
  Thư viện sử dụng trực tiếp Timer3, Timer4 và PORTC D30..D37.
  Thư viện khác cấu hình lại các tài nguyên này có thể gây xung đột.
==============================================================================*/

#ifndef TUNGLAM_OMNIMECANUM_4WD_H
#define TUNGLAM_OMNIMECANUM_4WD_H

#include <Arduino.h>

// ============================================================================
// API LEGACY V5
// ============================================================================

/**
 * @brief Class tương thích với thư viện TungLam_Control_MotorV5 cũ.
 *
 * Mục tiêu là để các project RoboBall/robot cũ có thể cập nhật thư viện mà
 * không phải đổi tên class hay các hàm chuyển động đã sử dụng.
 */
class TungLam_Control_MotorV5 {
 public:
  /** @brief Tạo controller V5. Phần cứng được khởi tạo khi gọi Mode0/Mode1. */
  TungLam_Control_MotorV5();

  /**
   * @brief Khởi tạo V5 với PWM motor khoảng 976,56 Hz.
   *
   * Dùng khi cần đúng hành vi/tần số PWM legacy cũ.
   */
  void Mode0();

  /**
   * @brief Khởi tạo V5 với PWM motor khoảng 7,8125 kHz.
   *
   * Đây là mode thường dùng trong các project V5/RoboBall trước đây.
   */
  void Mode1();

  /**
   * @brief Khởi tạo Timer1 phụ theo API lịch sử V5.
   * @param pwmD11 Duty kênh 11, giá trị 0..255.
   * @param pwmD12 Duty kênh 12, giá trị 0..255.
   * @warning Có thể xung đột với thư viện khác cũng dùng Timer1.
   */
  void Init_Timer1(uint8_t pwmD11, uint8_t pwmD12);

  /**
   * @brief Khởi tạo Timer2 phụ theo API lịch sử V5.
   * @param pwmD9 Duty kênh 9, giá trị 0..255.
   * @param pwmD10 Duty kênh 10, giá trị 0..255.
   * @warning Có thể xung đột với thư viện khác cũng dùng Timer2.
   */
  void Init_Timer2(uint8_t pwmD9, uint8_t pwmD10);

  /** @brief Dừng 4 motor và reset state chuyển động/ABS của V5. */
  void STOP();

  /**
   * @brief Cho robot chạy thẳng tiến bằng API V5.
   * @param pwm PWM chung cho 4 bánh, 0..255.
   */
  void moveForward(uint8_t pwm);

  /**
   * @brief Cho robot chạy thẳng lùi bằng API V5.
   * @param pwm PWM chung cho 4 bánh, 0..255.
   */
  void moveBackward(uint8_t pwm);

  /**
   * @brief Cho robot đi chéo tiến-phải.
   * @param pwm PWM của các bánh đang hoạt động, 0..255.
   */
  void Forward_Right(uint8_t pwm);

  /**
   * @brief Cho robot đi chéo lùi-phải.
   * @param pwm PWM của các bánh đang hoạt động, 0..255.
   */
  void Backward_Right(uint8_t pwm);

  /**
   * @brief Quay robot sang phải / cùng chiều kim đồng hồ.
   * @param pwm PWM quay, 0..255.
   */
  void moveRight(uint8_t pwm);

  /**
   * @brief Quay robot sang trái / ngược chiều kim đồng hồ.
   * @param pwm PWM quay, 0..255.
   */
  void moveLeft(uint8_t pwm);

  /**
   * @brief Cho đế Mecanum đi ngang sang trái.
   * @param pwm PWM dịch ngang, 0..255.
   */
  void moveLeftSide(uint8_t pwm);

  /**
   * @brief Cho đế Mecanum đi ngang sang phải.
   * @param pwm PWM dịch ngang, 0..255.
   */
  void moveRightSide(uint8_t pwm);

  /**
   * @brief Cho robot đi chéo tiến-trái.
   * @param pwm PWM của các bánh đang hoạt động, 0..255.
   */
  void Forward_Left(uint8_t pwm);

  /**
   * @brief Cho robot đi chéo lùi-trái.
   * @param pwm PWM của các bánh đang hoạt động, 0..255.
   */
  void Backward_Left(uint8_t pwm);

  /**
   * @brief Hãm ngược chủ động theo hướng chuyển động V5 trước đó.
   * @param doManhHam Duty hãm thực tế từ 0..255.
   *
   * Timer3 overflow ISR tự kết thúc xung hãm, nên code V5 không cần update().
   */
  void ABS(uint8_t doManhHam);

  /**
   * @brief Điều khiển trực tiếp chiều một bánh theo convention V5.
   * @param soBanh Số bánh từ 1 đến 4.
   * @param chieu Giá trị chiều theo convention lịch sử.
   */
  void Dir(uint8_t soBanh, bool chieu);

  /**
   * @brief Chạy tiến với PWM riêng cho từng bánh.
   * @param pwmM1 PWM M1 trước-trái, 0..255.
   * @param pwmM2 PWM M2 sau-trái, 0..255.
   * @param pwmM3 PWM M3 trước-phải, 0..255.
   * @param pwmM4 PWM M4 sau-phải, 0..255.
   */
  void Tien(uint8_t pwmM1, uint8_t pwmM2, uint8_t pwmM3, uint8_t pwmM4);

  /**
   * @brief Chạy lùi với PWM riêng cho từng bánh.
   * @param pwmM1 PWM M1, 0..255.
   * @param pwmM2 PWM M2, 0..255.
   * @param pwmM3 PWM M3, 0..255.
   * @param pwmM4 PWM M4, 0..255.
   */
  void Lui(uint8_t pwmM1, uint8_t pwmM2, uint8_t pwmM3, uint8_t pwmM4);

  /**
   * @brief Quay trái với PWM riêng cho từng bánh.
   * @param pwmM1 PWM M1, 0..255.
   * @param pwmM2 PWM M2, 0..255.
   * @param pwmM3 PWM M3, 0..255.
   * @param pwmM4 PWM M4, 0..255.
   */
  void Trai(uint8_t pwmM1, uint8_t pwmM2, uint8_t pwmM3, uint8_t pwmM4);

  /**
   * @brief Quay phải với PWM riêng cho từng bánh.
   * @param pwmM1 PWM M1, 0..255.
   * @param pwmM2 PWM M2, 0..255.
   * @param pwmM3 PWM M3, 0..255.
   * @param pwmM4 PWM M4, 0..255.
   */
  void Phai(uint8_t pwmM1, uint8_t pwmM2, uint8_t pwmM3, uint8_t pwmM4);

  /**
   * @brief Đi chéo tiến-trái với PWM riêng cho từng bánh.
   * @param pwmM1 PWM M1 trước-trái, 0..255.
   * @param pwmM2 PWM M2 sau-trái, 0..255.
   * @param pwmM3 PWM M3 trước-phải, 0..255.
   * @param pwmM4 PWM M4 sau-phải, 0..255.
   */
  void T_Trai(uint8_t pwmM1, uint8_t pwmM2, uint8_t pwmM3, uint8_t pwmM4);

  /**
   * @brief Đi chéo tiến-phải với PWM riêng cho từng bánh.
   * @param pwmM1 PWM M1 trước-trái, 0..255.
   * @param pwmM2 PWM M2 sau-trái, 0..255.
   * @param pwmM3 PWM M3 trước-phải, 0..255.
   * @param pwmM4 PWM M4 sau-phải, 0..255.
   */
  void T_Phai(uint8_t pwmM1, uint8_t pwmM2, uint8_t pwmM3, uint8_t pwmM4);

  /**
   * @brief Đi chéo lùi-trái với PWM riêng cho từng bánh.
   * @param pwmM1 PWM M1 trước-trái, 0..255.
   * @param pwmM2 PWM M2 sau-trái, 0..255.
   * @param pwmM3 PWM M3 trước-phải, 0..255.
   * @param pwmM4 PWM M4 sau-phải, 0..255.
   */
  void L_Trai(uint8_t pwmM1, uint8_t pwmM2, uint8_t pwmM3, uint8_t pwmM4);

  /**
   * @brief Đi chéo lùi-phải với PWM riêng cho từng bánh.
   * @param pwmM1 PWM M1 trước-trái, 0..255.
   * @param pwmM2 PWM M2 sau-trái, 0..255.
   * @param pwmM3 PWM M3 trước-phải, 0..255.
   * @param pwmM4 PWM M4 sau-phải, 0..255.
   */
  void L_Phai(uint8_t pwmM1, uint8_t pwmM2, uint8_t pwmM3, uint8_t pwmM4);

  /**
   * @brief Đi ngang trái với PWM riêng cho từng bánh.
   * @param pwmM1 PWM M1 trước-trái, 0..255.
   * @param pwmM2 PWM M2 sau-trái, 0..255.
   * @param pwmM3 PWM M3 trước-phải, 0..255.
   * @param pwmM4 PWM M4 sau-phải, 0..255.
   */
  void N_Trai(uint8_t pwmM1, uint8_t pwmM2, uint8_t pwmM3, uint8_t pwmM4);

  /**
   * @brief Đi ngang phải với PWM riêng cho từng bánh.
   * @param pwmM1 PWM M1 trước-trái, 0..255.
   * @param pwmM2 PWM M2 sau-trái, 0..255.
   * @param pwmM3 PWM M3 trước-phải, 0..255.
   * @param pwmM4 PWM M4 sau-phải, 0..255.
   */
  void N_Phai(uint8_t pwmM1, uint8_t pwmM2, uint8_t pwmM3, uint8_t pwmM4);

  /**
   * @brief Cấu hình thời gian ABS theo thời gian robot đã chạy trước khi hãm.
   * @param hamKhiChayDuoi500Ms Xung hãm [ms] khi thời gian chạy <500 ms.
   * @param hamKhiChayDuoi1000Ms Xung hãm [ms] khi thời gian chạy 500..999 ms.
   * @param hamKhiChayDuoi1500Ms Xung hãm [ms] khi thời gian chạy 1000..1499 ms.
   * @param hamKhiChayDuoi2000Ms Xung hãm [ms] khi thời gian chạy 1500..1999 ms.
   * @param hamKhiChayDuoi3000Ms Xung hãm [ms] khi thời gian chạy 2000..2999 ms.
   * @param hamKhiChayTu3000Ms Xung hãm [ms] khi thời gian chạy >=3000 ms.
   */
  void setTimABS(uint8_t hamKhiChayDuoi500Ms,
                 uint8_t hamKhiChayDuoi1000Ms,
                 uint8_t hamKhiChayDuoi1500Ms,
                 uint8_t hamKhiChayDuoi2000Ms,
                 uint8_t hamKhiChayDuoi3000Ms,
                 uint8_t hamKhiChayTu3000Ms);

 private:
  /** @brief Reset thanh ghi timer legacy trước khi cấu hình lại. */
  void Reset_Timer(uint8_t timerNumber);

  /** @brief Bắt đầu đo thời gian của pha chuyển động hiện tại. */
  void Tim();

  /** @brief Chọn thời gian ABS theo thời gian robot đã chạy. */
  void Timer();

  bool isMoving = false;          // Robot đang ở trong một pha chuyển động.
  unsigned long startTime = 0;   // Thời điểm bắt đầu pha chuyển động.
  uint8_t TIM = 0;               // Thời gian hãm ABS đã chọn.
  uint8_t pre = 0;               // Mã chuyển động trước đó của V5.
  uint8_t tim500 = 45;           // ABS khi chạy <500 ms.
  uint8_t tim1000 = 65;          // ABS khi chạy <1000 ms.
  uint8_t tim1500 = 70;          // ABS khi chạy <1500 ms.
  uint8_t tim2000 = 75;          // ABS khi chạy <2000 ms.
  uint8_t tim3000 = 80;          // ABS khi chạy <3000 ms.
  uint8_t timAbove3000 = 85;     // ABS khi chạy >=3000 ms.
  uint8_t pwmMode = 1;           // 0≈976 Hz, 1≈7,8 kHz.
  bool Set = false;              // State chiều lịch sử của V5.
};

/** @brief Alias chuyển tiếp giữ lại cho code các phiên bản cũ. */
using TungLamMecanumL298N = TungLam_Control_MotorV5;

// ============================================================================
// MODERN API
// ============================================================================

/** @brief Chọn tần số PWM phần cứng cho chân EN motor. */
enum class TungLamPwmMode : uint8_t {
  Low976Hz = 0,   ///< PWM khoảng 976,56 Hz; chủ yếu để tương thích legacy.
  High7k8Hz = 1   ///< PWM khoảng 7,8125 kHz; khuyến nghị cho motor DC/L298N.
};

/** @brief Chọn mô hình động học dùng bởi drive()/driveVelocity(). */
enum class TungLamChassis : uint8_t {
  MecanumX = 0,  ///< Đế Mecanum-X 4 bánh.
  OmniX = 1      ///< Đế Omni X-drive canonical, trục lăn 45 độ.
};

/**
 * @brief Thông số vật lý dùng cho động học SI và feed-forward vòng hở.
 *
 * motorNoLoadRpm phải là RPM tại trục đầu ra cuối cùng kéo bánh.
 * speedScale dùng để hiệu chỉnh sụt áp driver, tải, ma sát và sai số thực tế.
 */
struct TungLamDriveConfig {
  float motorNominalVoltageV;  ///< Điện áp danh định gắn với thông số RPM [V].
  float motorNoLoadRpm;        ///< RPM không tải tại đầu ra hộp số/trục bánh.
  float supplyVoltageV;        ///< Điện áp thực cấp cho H-bridge [V].
  float wheelRadiusM;          ///< Bán kính lăn hiệu dụng của bánh [m].
  float wheelbaseM;            ///< Khoảng cách tâm bánh trước - sau [m].
  float trackWidthM;           ///< Khoảng cách tâm bánh trái - phải [m].
  float speedScale;            ///< Hệ số hiệu chỉnh vòng hở; 1.0 = lý tưởng.

  /**
   * @brief Tạo cấu hình vật lý của motor và kích thước đế.
   * @param dienApMotorV Điện áp danh định của motor [V], ví dụ 12 hoặc 24.
   * @param tocDoKhongTaiRpm RPM không tải tại trục đầu ra cuối cùng kéo bánh.
   * @param dienApNguonV Điện áp thực cấp cho driver/H-bridge [V].
   * @param banKinhBanhM Bán kính lăn hiệu dụng của bánh [m].
   * @param chieuDaiTamBanhM Khoảng cách tâm bánh trước - tâm bánh sau [m].
   * @param chieuRongTamBanhM Khoảng cách tâm bánh trái - tâm bánh phải [m].
   * @param heSoHieuChinh Hệ số hiệu chỉnh vòng hở; bắt đầu từ 1.0 rồi đo thực tế để tune.
   */
  TungLamDriveConfig(float dienApMotorV = 0.0f,
                     float tocDoKhongTaiRpm = 0.0f,
                     float dienApNguonV = 0.0f,
                     float banKinhBanhM = 0.0f,
                     float chieuDaiTamBanhM = 0.0f,
                     float chieuRongTamBanhM = 0.0f,
                     float heSoHieuChinh = 1.0f)
      : motorNominalVoltageV(dienApMotorV),
        motorNoLoadRpm(tocDoKhongTaiRpm),
        supplyVoltageV(dienApNguonV),
        wheelRadiusM(banKinhBanhM),
        wheelbaseM(chieuDaiTamBanhM),
        trackWidthM(chieuRongTamBanhM),
        speedScale(heSoHieuChinh) {}
};

/** @brief Vận tốc tiếp tuyến có dấu của từng bánh, đơn vị m/s. */
struct TungLamWheelVelocity {
  float m1Mps;  ///< Vận tốc M1 trước-trái [m/s].
  float m2Mps;  ///< Vận tốc M2 sau-trái [m/s].
  float m3Mps;  ///< Vận tốc M3 trước-phải [m/s].
  float m4Mps;  ///< Vận tốc M4 sau-phải [m/s].
};

/** @brief Vận tốc thân robot theo body frame chuẩn. */
struct TungLamBodyVelocity {
  float vxMps;    ///< Vận tốc X [m/s]: dương=tiến.
  float vyMps;    ///< Vận tốc Y [m/s]: dương=trái.
  float wzRadps;  ///< Tốc độ quay Z [rad/s]: dương=CCW/trái.
};

/**
 * @brief Controller hiện đại cho đế robot 4 bánh.
 *
 * Hệ tọa độ:
 * - +vx: tiến
 * - +vy: trái
 * - +wz: quay trái / CCW
 */
class TungLamDrive4WD {
 public:
  /** @brief Tạo controller với state an toàn, output ban đầu bằng 0. */
  TungLamDrive4WD();

  /**
   * @brief Khởi tạo thư viện và phần cứng điều khiển 4 motor.
   * @param cheDoPWM Chọn tần số PWM. High7k8Hz là lựa chọn khuyến nghị.
   *
   * Gọi một lần trong setup() trước các lệnh chuyển động.
   */
  void begin(TungLamPwmMode cheDoPWM = TungLamPwmMode::High7k8Hz);

  /**
   * @brief Đồng bộ state phần mềm sau các sự kiện hardware-timed.
   *
   * ABS được Timer3 ISR tự cắt nên update() không phải điều kiện an toàn.
   */
  void update();

  /**
   * @brief Chọn loại đế để thư viện dùng đúng phương trình động học.
   * @param kieuDe MecanumX hoặc OmniX.
   */
  void setChassis(TungLamChassis kieuDe);

  /**
   * @brief Đọc loại đế đang được chọn.
   * @return TungLamChassis::MecanumX hoặc TungLamChassis::OmniX.
   */
  TungLamChassis chassis() const;

  /**
   * @brief Đảo chiều logic riêng một motor mà không cần sửa phương trình động học.
   * @param soBanh Số motor: 1=M1 trước-trái, 2=M2 sau-trái, 3=M3 trước-phải, 4=M4 sau-phải.
   * @param daoChieu true = đảo chiều motor; false = dùng chiều mặc định.
   *
   * Dùng khi một motor lắp/đấu ngược so với quy ước của thư viện.
   */
  void setMotorInverted(uint8_t soBanh, bool daoChieu);

  /**
   * @brief Đặt thời gian chết khi đổi chiều H-bridge để tránh đảo chiều khi PWM còn hoạt động.
   * @param thoiGianChetUs Thời gian chết tính bằng micro giây [us]. Mặc định 100 us.
   */
  void setDirectionDeadTimeUs(uint16_t thoiGianChetUs);

  /**
   * @brief Điều khiển trực tiếp 4 bánh bằng PWM có dấu.
   * @param lenhM1 M1 trước-trái: -255..255.
   * @param lenhM2 M2 sau-trái: -255..255.
   * @param lenhM3 M3 trước-phải: -255..255.
   * @param lenhM4 M4 sau-phải: -255..255.
   *
   * Dương = tiến logic, âm = lùi logic, 0 = dừng bánh.
   * Phù hợp khi bạn tự viết thuật toán động học/điều khiển riêng.
   */
  void setWheels(int16_t lenhM1, int16_t lenhM2, int16_t lenhM3, int16_t lenhM4);

  /**
   * @brief Điều khiển robot theo vx, vy, wz dạng normalized -255..255.
   * @param vxTien Vận tốc tương đối trục X: dương=tiến, âm=lùi.
   * @param vyTrai Vận tốc tương đối trục Y: dương=trái, âm=phải.
   * @param wzQuayTrai Tốc độ quay tương đối trục Z: dương=CCW/trái, âm=CW/phải.
   *
   * Đây không phải đơn vị vật lý. Muốn dùng m/s và rad/s hãy dùng driveVelocity().
   */
  void drive(int16_t vxTien, int16_t vyTrai, int16_t wzQuayTrai);

  /**
   * @brief Khai báo thông số motor và kích thước đế cho động học SI.
   * @param thongSoRobot Cấu hình gồm điện áp, RPM, bán kính bánh, wheelbase, track width và speedScale.
   * @return true nếu cấu hình hợp lệ; false nếu có thông số bắt buộc <= 0.
   *
   * Cần gọi hàm này trước driveVelocity().
   */
  bool setDriveConfig(const TungLamDriveConfig& thongSoRobot);

  /**
   * @brief Đọc lại cấu hình vật lý motor/chassis hiện tại.
   * @return Tham chiếu tới TungLamDriveConfig đang được sử dụng.
   */
  const TungLamDriveConfig& driveConfig() const;

  /**
   * @brief Kiểm tra đã cấu hình đủ thông số để dùng SI API hay chưa.
   * @return true nếu driveVelocity()/kinematics có model hợp lệ.
   */
  bool hasDriveConfig() const;

  /**
   * @brief Tính RPM không tải ước lượng tại điện áp nguồn hiện tại.
   * @return RPM ước lượng sau khi áp dụng tỷ lệ điện áp và speedScale.
   */
  float estimatedMotorRpmAtSupply() const;

  /**
   * @brief Tính tốc độ tiếp tuyến cực đại ước lượng của bánh.
   * @return Tốc độ bánh [m/s] theo RPM, điện áp, bán kính và speedScale.
   */
  float maxWheelLinearSpeedMps() const;

  /**
   * @brief Tính tốc độ tịnh tiến cực đại ước lượng của đế.
   * @return Tốc độ thân robot [m/s] theo chassis/model hiện tại.
   */
  float maxBodyLinearSpeedMps() const;

  /**
   * @brief Tính tốc độ quay yaw cực đại ước lượng của đế.
   * @return Tốc độ góc quanh trục Z [rad/s].
   */
  float maxYawRateRadps() const;

  /**
   * @brief Động học nghịch: đổi vận tốc thân robot thành vận tốc từng bánh.
   * @param vxTienMps Vận tốc X [m/s]: dương=tiến, âm=lùi.
   * @param vyTraiMps Vận tốc Y [m/s]: dương=trái, âm=phải.
   * @param wzQuayTraiRadps Tốc độ quay Z [rad/s]: dương=CCW/trái, âm=CW/phải.
   * @return TungLamWheelVelocity chứa vận tốc M1..M4 theo m/s.
   *
   * Hàm chỉ tính toán, không làm motor quay.
   */
  TungLamWheelVelocity inverseKinematics(float vxTienMps,
                                         float vyTraiMps,
                                         float wzQuayTraiRadps) const;

  /**
   * @brief Động học thuận: đổi vận tốc từng bánh thành vx, vy, wz của robot.
   * @param tocDoBanh Vận tốc tiếp tuyến M1..M4 [m/s], thường lấy từ encoder sau này.
   * @return TungLamBodyVelocity gồm vx [m/s], vy [m/s], wz [rad/s].
   *
   * Hàm chỉ tính toán; thư viện hiện không tự đọc encoder.
   */
  TungLamBodyVelocity forwardKinematics(
      const TungLamWheelVelocity& tocDoBanh) const;

  /**
   * @brief Điều khiển robot bằng vận tốc vật lý SI: m/s và rad/s.
   * @param vxTienMps Vận tốc X [m/s]: dương=tiến, âm=lùi.
   * @param vyTraiMps Vận tốc Y [m/s]: dương=trái, âm=phải.
   * @param wzQuayTraiRadps Tốc độ quay Z [rad/s]: dương=CCW/trái, âm=CW/phải.
   * @return true nếu lệnh được xử lý; false nếu chưa có TungLamDriveConfig hợp lệ.
   *
   * Nếu yêu cầu vượt tốc độ model, thư viện tự scale đồng đều tốc độ 4 bánh
   * để giữ tỷ lệ động học. Hiện đây là feed-forward vòng hở, chưa phải PID encoder.
   */
  bool driveVelocity(float vxTienMps, float vyTraiMps, float wzQuayTraiRadps);

  /**
   * @brief Bật gói an toàn thông minh chỉ bằng một lệnh.
   * @param thoiGianMatLenhMs Thời gian tối đa không nhận lệnh mới trước khi tự dừng [ms]. 0 = tắt watchdog.
   * @param gioiHanGiaTocTinhTienMps2 Giới hạn thay đổi vx/vy [m/s^2]. <=0 = không giới hạn mềm.
   * @param gioiHanGiaTocQuayRadps2 Giới hạn thay đổi wz [rad/s^2]. <=0 = không giới hạn mềm.
   *
   * Sau khi bật, hãy gọi update() liên tục trong loop(). Thư viện tự xử lý
   * watchdog mất lệnh và làm mượt driveVelocity().
   */
  void enableSmartSafety(uint16_t thoiGianMatLenhMs = 500,
                         float gioiHanGiaTocTinhTienMps2 = 1.0f,
                         float gioiHanGiaTocQuayRadps2 = 2.0f);

  /** @brief Tắt watchdog và giới hạn tăng/giảm tốc thông minh. */
  void disableSmartSafety();

  /**
   * @brief Đặt watchdog tự dừng nếu chương trình mất lệnh điều khiển.
   * @param thoiGianMatLenhMs Timeout [ms]. 0 = tắt watchdog.
   *
   * Cần gọi update() liên tục trong loop() để watchdog hoạt động.
   */
  void setCommandTimeoutMs(uint16_t thoiGianMatLenhMs);

  /**
   * @brief Kiểm tra watchdog có vừa tự dừng robot do mất lệnh hay không.
   * @return true nếu timeout đã xảy ra kể từ lệnh điều khiển hợp lệ gần nhất.
   */
  bool commandTimedOut() const;

  /**
   * @brief Bật giới hạn tăng/giảm vận tốc cho driveVelocity().
   * @param gioiHanGiaTocTinhTienMps2 Mức thay đổi tối đa của vx và vy [m/s^2].
   * @param gioiHanGiaTocQuayRadps2 Mức thay đổi tối đa của wz [rad/s^2].
   * @return true nếu cả hai giới hạn >0; false nếu thông số không hợp lệ và ramp bị tắt.
   */
  bool setVelocityRamp(float gioiHanGiaTocTinhTienMps2, float gioiHanGiaTocQuayRadps2);

  /** @brief Tắt giới hạn tăng/giảm vận tốc của driveVelocity(). */
  void disableVelocityRamp();

  /**
   * @brief Kiểm tra lệnh SI gần nhất có vượt khả năng tốc độ của đế hay không.
   * @return true nếu thư viện phải scale tốc độ 4 bánh xuống để không vượt model.
   */
  bool wasVelocityLimited() const;

  /**
   * @brief Đọc hệ số scale của lệnh SI gần nhất.
   * @return 1.0 nếu không giới hạn; ví dụ 0.5 nghĩa là vector đã giảm còn 50%.
   */
  float lastVelocityScale() const;

  /**
   * @brief Đọc vx,vy,wz mà người dùng yêu cầu gần nhất trước ramp/saturation.
   * @return TungLamBodyVelocity theo m/s và rad/s.
   */
  TungLamBodyVelocity requestedBodyVelocity() const;

  /**
   * @brief Đọc vx,vy,wz ước lượng mà thư viện thực sự áp sau ramp/saturation.
   * @return TungLamBodyVelocity theo m/s và rad/s.
   */
  TungLamBodyVelocity appliedBodyVelocity() const;

  /**
   * @brief Điều khiển trực tiếp mixer Mecanum-X bằng vx, vy, wz normalized.
   * @param vxTien -255..255; dương=tiến.
   * @param vyTrai -255..255; dương=trái.
   * @param wzQuayTrai -255..255; dương=quay CCW/trái.
   */
  void driveMecanum(int16_t vxTien, int16_t vyTrai, int16_t wzQuayTrai);

  /**
   * @brief Điều khiển trực tiếp mixer Omni X-drive canonical bằng lệnh normalized.
   * @param vxTien -255..255; dương=tiến.
   * @param vyTrai -255..255; dương=trái.
   * @param wzQuayTrai -255..255; dương=quay CCW/trái.
   */
  void driveOmniX(int16_t vxTien, int16_t vyTrai, int16_t wzQuayTrai);

  /**
   * @brief Cho robot chạy thẳng tiến.
   * @param pwm PWM từ 0..255; số càng lớn thì duty càng cao.
   */
  void forward(uint8_t pwm);

  /**
   * @brief Cho robot chạy thẳng lùi.
   * @param pwm PWM từ 0..255.
   */
  void backward(uint8_t pwm);

  /**
   * @brief Cho đế đa hướng đi ngang sang phải (-Y).
   * @param pwm PWM từ 0..255.
   */
  void strafeRight(uint8_t pwm);

  /**
   * @brief Cho đế đa hướng đi ngang sang trái (+Y).
   * @param pwm PWM từ 0..255.
   */
  void strafeLeft(uint8_t pwm);

  /**
   * @brief Cho robot quay phải / cùng chiều kim đồng hồ (CW, -wz).
   * @param pwm PWM từ 0..255.
   */
  void rotateRight(uint8_t pwm);

  /**
   * @brief Cho robot quay trái / ngược chiều kim đồng hồ (CCW, +wz).
   * @param pwm PWM từ 0..255.
   */
  void rotateLeft(uint8_t pwm);

  /**
   * @brief Dừng robot theo kiểu coast: PWM về 0, không chủ động ghìm motor.
   */
  void stop();

  /**
   * @brief Dừng kiểu thả trôi/coast; tương đương stop().
   */
  void coast();

  /**
   * @brief Hãm điện bằng H-bridge L298N, không đảo mô-men như ABS.
   *
   * Dùng để ghìm motor bằng trạng thái điện của bridge.
   */
  void dynamicBrake();

  /**
   * @brief Hãm ngược chủ động (ABS): tạo mô-men ngược trong thời gian ngắn.
   * @param doManhHam Duty hãm thực tế từ 0..255.
   *
   * Timer3 ISR tự kết thúc xung hãm. Duty/thời gian quá lớn có thể gây dòng cao.
   */
  void ABS(uint8_t doManhHam);

  /**
   * @brief Tên dễ hiểu hơn của ABS(); thực hiện hãm ngược chủ động.
   * @param doManhHam Duty hãm từ 0..255.
   */
  void activeBrake(uint8_t doManhHam);

  /**
   * @brief Kiểm tra xung ABS hiện còn đang hoạt động hay không.
   * @return true khi Timer3 vẫn đang duy trì xung hãm.
   */
  bool isBraking() const;

  /**
   * @brief Hủy ABS/dynamic-brake hiện tại và đưa output về trạng thái coast.
   */
  void cancelBrake();

  /**
   * @brief Cấu hình thời gian xung ABS theo thời gian robot đã chạy.
   * @param hamKhiChayDuoi500Ms Thời gian hãm [ms] khi robot đã chạy <500 ms.
   * @param hamKhiChayDuoi1000Ms Thời gian hãm [ms] khi đã chạy 500..999 ms.
   * @param hamKhiChayDuoi1500Ms Thời gian hãm [ms] khi đã chạy 1000..1499 ms.
   * @param hamKhiChayDuoi2000Ms Thời gian hãm [ms] khi đã chạy 1500..1999 ms.
   * @param hamKhiChayDuoi3000Ms Thời gian hãm [ms] khi đã chạy 2000..2999 ms.
   * @param hamKhiChayTu3000Ms Thời gian hãm [ms] khi đã chạy >=3000 ms.
   */
  void setTimABS(uint8_t hamKhiChayDuoi500Ms,
                 uint8_t hamKhiChayDuoi1000Ms,
                 uint8_t hamKhiChayDuoi1500Ms,
                 uint8_t hamKhiChayDuoi2000Ms,
                 uint8_t hamKhiChayDuoi3000Ms,
                 uint8_t hamKhiChayTu3000Ms);

  /**
   * @brief Tên dễ hiểu hơn của setTimABS(); cấu hình 6 mốc thời gian hãm.
   * @param hamKhiChayDuoi500Ms Xung hãm [ms] cho <500 ms.
   * @param hamKhiChayDuoi1000Ms Xung hãm [ms] cho 500..999 ms.
   * @param hamKhiChayDuoi1500Ms Xung hãm [ms] cho 1000..1499 ms.
   * @param hamKhiChayDuoi2000Ms Xung hãm [ms] cho 1500..1999 ms.
   * @param hamKhiChayDuoi3000Ms Xung hãm [ms] cho 2000..2999 ms.
   * @param hamKhiChayTu3000Ms Xung hãm [ms] cho >=3000 ms.
   */
  void setBrakeTimings(uint8_t hamKhiChayDuoi500Ms,
                       uint8_t hamKhiChayDuoi1000Ms,
                       uint8_t hamKhiChayDuoi1500Ms,
                       uint8_t hamKhiChayDuoi2000Ms,
                       uint8_t hamKhiChayDuoi3000Ms,
                       uint8_t hamKhiChayTu3000Ms);

 private:
  /** @brief Vector lệnh bánh nội bộ theo thứ tự M1..M4. */
  struct Wheels {
    int16_t m1;
    int16_t m2;
    int16_t m3;
    int16_t m4;
  };

  TungLamChassis chassis_;             // Mixer hiện đang chọn.
  TungLamPwmMode pwmMode_;             // Chế độ PWM phần cứng.
  TungLamDriveConfig driveConfig_;     // Model vật lý motor/chassis.
  bool driveConfigValid_;              // Model vật lý đã hợp lệ hay chưa.

  float cachedLeverM_;                 // (wheelbase + trackWidth) / 2.
  float cachedMotorRpm_;               // RPM đã áp tỷ lệ điện áp + speedScale.
  float cachedMaxWheelMps_;            // Tốc độ bánh cực đại ước lượng.
  float cachedPwmPerMps_;              // Hệ số m/s -> PWM.
  float cachedMaxBodyMps_;             // Tốc độ tịnh tiến cực đại của chassis.
  float cachedMaxYawRadps_;            // Tốc độ quay cực đại của chassis.

  TungLamBodyVelocity requestedBodyVelocity_; // Lệnh SI người dùng yêu cầu.
  TungLamBodyVelocity rampedBodyVelocity_;    // Lệnh sau bộ làm mượt.
  TungLamBodyVelocity appliedBodyVelocity_;   // Lệnh sau saturation.
  float lastVelocityScale_;            // 1.0 = không saturation.
  bool velocityLimited_;               // Lệnh SI gần nhất có bị scale hay không.

  uint16_t commandTimeoutMs_;          // 0 = watchdog tắt.
  uint32_t lastCommandMs_;             // Thời điểm nhận lệnh motor gần nhất.
  bool commandTimedOut_;               // Watchdog vừa tự dừng robot.
  bool velocityRampEnabled_;           // Bật làm mượt SI.
  bool velocityRampPrimed_;            // Ramp đã có mốc thời gian hay chưa.
  float linearAccelMps2_;              // Slew rate vx/vy.
  float yawAccelRadps2_;               // Slew rate wz.
  uint32_t lastVelocityUpdateMs_;      // Mốc dt của ramp.

  bool inverted_[4];                   // Cờ đảo polarity từng motor.
  uint16_t deadTimeUs_;                // Dead-time khi đổi trạng thái chiều.

  Wheels commanded_;                   // Lệnh logic gần nhất.
  Wheels applied_;                     // Lệnh vật lý đã áp xuống HAL.
  Wheels preBrake_;                    // Lệnh trước khi bắt đầu ABS.

  bool moving_;                        // Đang có lệnh chuyển động.
  bool braking_;                       // Đang ABS active reverse.
  bool dynamicBraking_;                // Đang dynamic brake.
  uint32_t motionStartMs_;             // Thời điểm bắt đầu pha chuyển động.
  uint32_t brakeStartMs_;              // Thời điểm bắt đầu xung ABS.
  uint8_t brakeDurationMs_;            // Độ dài xung ABS hiện tại.

  uint8_t brakeT500_;                  // Timing cho <500 ms.
  uint8_t brakeT1000_;                 // Timing cho <1000 ms.
  uint8_t brakeT1500_;                 // Timing cho <1500 ms.
  uint8_t brakeT2000_;                 // Timing cho <2000 ms.
  uint8_t brakeT3000_;                 // Timing cho <3000 ms.
  uint8_t brakeTAbove3000_;            // Timing cho >=3000 ms.

  /** @brief Cấu hình Timer3/Timer4 theo pwmMode_. */
  void initPwm();

  /** @brief Áp vector bánh logic xuống common motor HAL. */
  void applyWheelsRaw(const Wheels& wheels);

  /** @brief Chọn thời gian ABS từ bảng 6 khoảng. */
  uint8_t selectBrakeDuration(uint32_t motionDurationMs) const;

  /** @brief Giới hạn một lệnh bánh về -255..255. */
  static int16_t clampWheel(int32_t value);

  /** @brief Trả -1, 0 hoặc +1 theo dấu của lệnh. */
  static int8_t signOf(int16_t value);

  /** @brief Kiểm tra có ít nhất một bánh đang được lệnh chạy hay không. */
  static bool anyMoving(const Wheels& wheels);

  /** @brief Phát hiện đảo dấu thực sự giữa hai vector bánh. */
  static bool directionChanged(const Wheels& a, const Wheels& b);

  /** @brief Kiểm tra các thông số model vật lý có hợp lệ hay không. */
  static bool driveConfigIsValid(const TungLamDriveConfig& config);

  /** @brief Tính trước các hằng số motor/động học để control loop chạy nhẹ hơn. */
  void rebuildDerivedModel();

  /** @brief Cập nhật giới hạn thân robot khi đổi chassis. */
  void rebuildChassisLimits();

  /** @brief Ghi nhận một lệnh mới để reset watchdog. */
  void noteCommandReceived();

  /** @brief Reset state của bộ làm mượt vận tốc. */
  void resetVelocityRampState();

  /** @brief Áp giới hạn tăng/giảm vận tốc trong body frame. */
  TungLamBodyVelocity applyVelocityRamp(const TungLamBodyVelocity& target,
                                        uint32_t nowMs);

  /** @brief Tiến một giá trị float về target nhưng không vượt maxDelta. */
  static float approachFloat(float current, float target, float maxDelta);

  /** @brief Quy đổi vận tốc tiếp tuyến bánh [m/s] thành PWM feed-forward. */
  int16_t wheelVelocityToPwm(float wheelMps) const;

  /** @brief Scale vector tốc độ bánh về giới hạn vật lý nhưng giữ tỷ lệ. */
  TungLamWheelVelocity limitWheelVelocities(
      const TungLamWheelVelocity& wheels,
      float* scaleOut = nullptr) const;

  /** @brief Chuẩn hóa vector bánh normalized để không vượt 255. */
  static Wheels normalize(int32_t m1, int32_t m2, int32_t m3, int32_t m4);
};

#endif  // TUNGLAM_OMNIMECANUM_4WD_H
