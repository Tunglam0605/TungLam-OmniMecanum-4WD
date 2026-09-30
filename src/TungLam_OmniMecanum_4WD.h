/*==============================================================================
  TUNGLAM OMNI / MECANUM 4WD MOTOR LIBRARY
  ==============================================================================
  TÁC GIẢ / AUTHOR
  ------------------------------------------------------------------------------
  Họ và tên : Nguyễn Khắc Tùng Lâm
  Lớp       : DHTD16A2CL
  MSSV       : 2210430016
  Thương hiệu: Tung Lâm Automation

  Thư viện gốc:
    TungLam_Control_MotorV5
    "Thư viện điều khiển xe 4 bánh đa hướng"

  Thư viện hiện tại:
    TungLam_OmniMecanum_4WD

  Mục tiêu tương thích:
    - Giữ nguyên API V5 để code robot cũ không phải sửa hàm.
    - Bổ sung API hiện đại cho Mecanum-X / Omni-X.
    - Giữ cơ chế ABS hãm ngược mạnh, cho phép người dùng tự đặt lực và thời gian.
    - Cải tiến ABS legacy thành non-blocking mà code cũ không cần gọi update().

  ==============================================================================
  SƠ ĐỒ ĐẤU NỐI V5 GỐC
  (Giữ lại nguyên tinh thần tài liệu gốc để người dùng cũ nhận biết ngay.)
  ------------------------------------------------------------------------------

  CHÂN PWM ------ CHÂN SỐ
    EN BÁNH 1 ----- 5
    EN BÁNH 2 ----- 6
    EN BÁNH 3 ----- 7
    EN BÁNH 4 ----- 8

  CHÂN CHIỀU DIR (BÁNH) ---- CHÂN CHIỀU TIẾN ---- CHÂN CHIỀU LÙI
              DIR1                    30                   31
              DIR2                    32                   33
              DIR3                    34                   35
              DIR4                    36                   37

  ==============================================================================
  MAPPING THỰC TẾ THEO CODE AVR HIỆN TẠI
  ------------------------------------------------------------------------------
  Arduino Mega 2560 ánh xạ PORTC như sau:

    D30 = PC7     D31 = PC6
    D32 = PC5     D33 = PC4
    D34 = PC3     D35 = PC2
    D36 = PC1     D37 = PC0

  Implementation thực tế của thư viện:

    Motor   PWM / EN       CHIỀU TIẾN (+)       CHIỀU LÙI (-)
    -----   -------------  -------------------  -------------------
    M1      D5  / OC3A     D30 / PC7            D31 / PC6
    M2      D6  / OC4A     D32 / PC5            D33 / PC4
    M3      D7  / OC4B     D34 / PC3            D35 / PC2
    M4      D8  / OC4C     D37 / PC0            D36 / PC1

  LƯU Ý QUAN TRỌNG:
    Comment V5 lịch sử ghi DIR4 "36 tiến / 37 lùi", nhưng code V5 thực tế
    dùng PC0 cho chiều tiến và PC1 cho chiều lùi. Trên Arduino Mega:
      PC0 = D37
      PC1 = D36
    Bảng "mapping thực tế" phía trên phản ánh đúng hành vi của code.

  ==============================================================================
  QUY ƯỚC THỨ TỰ 4 BÁNH - API HIỆN ĐẠI
  ------------------------------------------------------------------------------

                        ĐẦU XE / FRONT
                             +vx
                              ^
                              |
                 M1                         M3
            FRONT-LEFT                 FRONT-RIGHT
               D5                         D7

                 M2                         M4
             REAR-LEFT                  REAR-RIGHT
               D6                         D8
                              |
                       ĐUÔI XE / REAR

    +vx : tiến / forward
    -vx : lùi / backward
    +vy : ngang phải / strafe right
    -vy : ngang trái / strafe left
    +wz : quay phải, chiều kim đồng hồ / clockwise
    -wz : quay trái, ngược chiều kim đồng hồ / counter-clockwise

  Quy ước trên là thứ tự logic dùng bởi mixer Mecanum-X và Omni-X hiện đại.
  Với robot V5 cũ đã đấu dây và chạy ổn định, nên giữ nguyên wiring thực tế
  của xe cũ; compatibility API sẽ giữ nguyên hành vi V5.

  ==============================================================================
  QUICK CLASS REFERENCE
  ------------------------------------------------------------------------------

  Code cũ / Legacy:
    #include <TungLam_Control_MotorV5.h>
    TungLam_Control_MotorV5 motor;

  Code mới / Modern:
    #include <TungLam_OmniMecanum_4WD.h>
    TungLamDrive4WD motor;

  ==============================================================================
  ABS HÃM NGƯỢC / ACTIVE REVERSE BRAKE
  ------------------------------------------------------------------------------

  Legacy:
    motor.ABS(duty);
    motor.setTimABS(45, 65, 70, 75, 80, 85);

    - duty: lực hãm ngược do người dùng tự chọn 0..255.
    - setTimABS(): thời gian hãm theo 6 khoảng thời gian chạy.
    - Legacy ABS chạy non-blocking bằng Timer3 overflow interrupt.
    - Code V5 cũ KHÔNG cần thêm update().

  Modern:
    motor.ABS(duty);
    motor.update();  // gọi liên tục trong loop() khi dùng modern ABS.

  ==============================================================================
*/

/**
 * @file TungLam_OmniMecanum_4WD.h
 * @brief Public API for the TungLam 4-wheel holonomic motor library.
 *
 * @author Nguyen Khac Tung Lam - Tung Lam Automation
 *
 * @details
 * This library targets Arduino Mega 2560 / ATmega2560 and controls four
 * brushed-DC drive motors through two L298N dual H-bridge modules.
 *
 * Two public APIs intentionally coexist:
 *
 * 1. TungLam_Control_MotorV5
 *    - Source-compatible with the original V5 library.
 *    - Existing robot projects can keep the old class name and old functions.
 *    - Legacy ABS(duty) is non-blocking internally and does NOT require update().
 *
 * 2. TungLamDrive4WD
 *    - Modern API for new projects.
 *    - Supports Mecanum-X, Omni-X and direct signed wheel commands.
 *    - Modern active braking is serviced by update().
 *
 * Hardware mapping used by both APIs:
 *
 *   Motor   PWM pin   AVR output      Forward DIR   Reverse DIR
 *   -----   -------   --------------  -----------   -----------
 *   M1      D5        PE3 / OC3A      D30 / PC7     D31 / PC6
 *   M2      D6        PH3 / OC4A      D32 / PC5     D33 / PC4
 *   M3      D7        PH4 / OC4B      D34 / PC3     D35 / PC2
 *   M4      D8        PH5 / OC4C      D36 / PC0     D37 / PC1
 *
 * @warning
 * The library directly owns Timer3, Timer4 and PORTC D30..D37 for the four
 * drive motors. Another library that reconfigures these resources can conflict.
 *
 * @warning
 * Active reverse braking intentionally drives the motors in the opposite
 * direction for a short pulse. This can produce high current. Tune brake PWM
 * and brake duration for the real motor, gearbox, battery, chassis mass and
 * L298N thermal limits.
 */

#ifndef TUNGLAM_OMNIMECANUM_4WD_H
#define TUNGLAM_OMNIMECANUM_4WD_H

#include <Arduino.h>  // Arduino integer types, millis(), delayMicroseconds(), etc.

// ============================================================================
// LEGACY V5 API
// ============================================================================

/**
 * @class TungLam_Control_MotorV5
 * @brief Backward-compatible API matching the original TungLam V5 library.
 *
 * @details
 * This class deliberately preserves the old Vietnamese movement-function names
 * and signatures so existing competition robots can update the library without
 * rewriting application code.
 *
 * The implementation underneath has been hardened:
 * - active reverse braking no longer blocks with delay();
 * - Timer3 overflow interrupt terminates the legacy ABS pulse automatically;
 * - movement state is tracked for both common-duty and per-wheel-duty commands.
 */
class TungLam_Control_MotorV5 {
 public:
  /**
   * @brief Construct a legacy-compatible motor controller.
   *
   * @note Hardware is not configured until Mode0() or Mode1() is called.
   */
  TungLam_Control_MotorV5();

  /**
   * @brief Initialize D5..D8 motor PWM at approximately 976.56 Hz.
   *
   * Uses Timer3 and Timer4 with TOP=255 and prescaler 64.
   * Also configures D30..D37 as motor-direction outputs.
   */
  void Mode0();

  /**
   * @brief Initialize D5..D8 motor PWM at approximately 7.8125 kHz.
   *
   * Uses 8-bit Fast PWM on Timer3/Timer4 with prescaler 8.
   * This is the recommended/default legacy mode for the four drive motors.
   */
  void Mode1();

  /**
   * @brief Configure legacy auxiliary Timer1 PWM outputs on D11/D12.
   * @param duty11 Initial PWM duty for D11 / OC1A, range 0..255.
   * @param duty12 Initial PWM duty for D12 / OC1B, range 0..255.
   *
   * @warning This function takes ownership of Timer1.
   */
  void Init_Timer1(uint8_t duty11, uint8_t duty12);

  /**
   * @brief Configure legacy auxiliary Timer2 PWM outputs on D9/D10.
   * @param duty9  Initial PWM duty for D9, range 0..255.
   * @param duty10 Initial PWM duty for D10, range 0..255.
   *
   * @warning This function takes ownership of Timer2.
   */
  void Init_Timer2(uint8_t duty9, uint8_t duty10);

  /**
   * @brief Stop all four drive motors and clear legacy motion/ABS state.
   *
   * PWM is set to zero and all direction outputs are cleared.
   */
  void STOP();

  /** @brief Drive straight forward with one common PWM duty. */
  void moveForward(uint8_t duty);

  /** @brief Drive straight backward with one common PWM duty. */
  void moveBackward(uint8_t duty);

  /** @brief Drive diagonally forward-right using the active diagonal wheel pair. */
  void Forward_Right(uint8_t duty);

  /** @brief Drive diagonally backward-right using the active diagonal wheel pair. */
  void Backward_Right(uint8_t duty);

  /** @brief Rotate the chassis clockwise/right in place. */
  void moveRight(uint8_t duty);

  /** @brief Rotate the chassis counter-clockwise/left in place. */
  void moveLeft(uint8_t duty);

  /** @brief Translate the chassis sideways to the left. */
  void moveLeftSide(uint8_t duty);

  /** @brief Translate the chassis sideways to the right. */
  void moveRightSide(uint8_t duty);

  /** @brief Drive diagonally forward-left using the active diagonal wheel pair. */
  void Forward_Left(uint8_t duty);

  /** @brief Drive diagonally backward-left using the active diagonal wheel pair. */
  void Backward_Left(uint8_t duty);

  /**
   * @brief Apply a strong active reverse-brake pulse without blocking the sketch.
   * @param duty Reverse-brake PWM selected directly by the user, range 0..255.
   *
   * @details
   * The function:
   * 1. identifies the previous movement direction;
   * 2. calculates the configured brake time from how long the robot had moved;
   * 3. removes PWM briefly before changing bridge direction;
   * 4. commands the opposite movement at exactly the requested duty;
   * 5. arms a Timer3 overflow one-shot;
   * 6. returns immediately;
   * 7. the ISR later sets all wheel PWM to zero at the configured deadline.
   *
   * No robot.update() call is required for this legacy class.
   */
  void ABS(uint8_t duty);

  /**
   * @brief Set one wheel direction directly using the original V5 convention.
   * @param BanhNumber Wheel index: 1..4.
   * @param Set Direction flag used by the historical V5 wiring convention.
   *
   * @note Invalid wheel numbers are ignored.
   * @note A new direction command cancels a pending legacy ABS pulse first.
   */
  void Dir(uint8_t BanhNumber, bool Set);

  // --------------------------------------------------------------------------
  // Legacy independent-per-wheel movement API.
  //
  // duty1..duty4 map to M1..M4 respectively. These functions preserve the old
  // names so old competition code can tune individual wheel PWM values without
  // source changes.
  // --------------------------------------------------------------------------

  /** @brief Forward motion with independent PWM for M1..M4. */
  void Tien(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);

  /** @brief Backward motion with independent PWM for M1..M4. */
  void Lui(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);

  /** @brief Rotate left with independent PWM for M1..M4. */
  void Trai(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);

  /** @brief Rotate right with independent PWM for M1..M4. */
  void Phai(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);

  /** @brief Forward-left diagonal with independent active-wheel PWM. */
  void T_Trai(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);

  /** @brief Forward-right diagonal with independent active-wheel PWM. */
  void T_Phai(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);

  /** @brief Backward-left diagonal with independent active-wheel PWM. */
  void L_Trai(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);

  /** @brief Backward-right diagonal with independent active-wheel PWM. */
  void L_Phai(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);

  /** @brief Translate left with independent PWM for M1..M4. */
  void N_Trai(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);

  /** @brief Translate right with independent PWM for M1..M4. */
  void N_Phai(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);

  /**
   * @brief Configure the six legacy reverse-brake time ranges.
   *
   * @param t500       Brake pulse in ms when movement duration is < 500 ms.
   * @param t1000      Brake pulse in ms when movement duration is < 1000 ms.
   * @param t1500      Brake pulse in ms when movement duration is < 1500 ms.
   * @param t2000      Brake pulse in ms when movement duration is < 2000 ms.
   * @param t3000      Brake pulse in ms when movement duration is < 3000 ms.
   * @param tAbove3000 Brake pulse in ms when movement duration is >= 3000 ms.
   *
   * Default table: 45, 65, 70, 75, 80, 85 ms.
   */
  void setTimABS(uint8_t t500,
                 uint8_t t1000,
                 uint8_t t1500,
                 uint8_t t2000,
                 uint8_t t3000,
                 uint8_t tAbove3000);

 private:
  // Historical helper used by old diagonal-direction logic.
  void Reset_45(bool Off);

  // Reset one AVR timer register set before reconfiguration.
  void Reset_Timer(uint8_t timerNumber);

  // Write the complete PORTC direction bit pattern.
  void setSTOP(uint8_t pattern);

  // Apply one common PWM duty to all four drive motors.
  void setPWM(uint8_t duty);

  // Apply independent PWM duties to M1..M4.
  void PWM(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);

  // Start motion-duration measurement on the first command of a movement phase.
  void Tim();

  // Select the ABS pulse duration from the six-stage timing table.
  void Timer();

  bool isMoving = false;          ///< True while legacy motion timing is active.
  unsigned long startTime = 0;   ///< millis() timestamp at movement-phase start.
  uint8_t TIM = 0;               ///< Selected reverse-brake pulse duration in ms.
  uint8_t pre = 0;               ///< Previous movement code, values 1..10.
  uint8_t tim500 = 45;           ///< Brake time for movement < 500 ms.
  uint8_t tim1000 = 65;          ///< Brake time for movement < 1000 ms.
  uint8_t tim1500 = 70;          ///< Brake time for movement < 1500 ms.
  uint8_t tim2000 = 75;          ///< Brake time for movement < 2000 ms.
  uint8_t tim3000 = 80;          ///< Brake time for movement < 3000 ms.
  uint8_t timAbove3000 = 85;     ///< Brake time for movement >= 3000 ms.
  uint8_t pwmMode = 1;           ///< 0 = ~976.56 Hz, 1 = ~7.8125 kHz.
  bool Set = false;              ///< Historical logical forward-direction flag.
};

/**
 * @brief Transitional 0.5.x type alias retained at zero implementation cost.
 *
 * @note New code should use TungLamDrive4WD or TungLam_Control_MotorV5.
 */
using TungLamMecanumL298N = TungLam_Control_MotorV5;

// ============================================================================
// MODERN V6+ API
// ============================================================================

/**
 * @enum TungLamPwmMode
 * @brief Selects the hardware PWM frequency used on drive-motor EN pins.
 */
enum class TungLamPwmMode : uint8_t {
  Low976Hz = 0,   ///< ~976.56 Hz, compatible with historical Mode0 behavior.
  High7k8Hz = 1   ///< ~7.8125 kHz, recommended default for the four drive motors.
};

/**
 * @enum TungLamChassis
 * @brief Selects the built-in holonomic mixer used by drive().
 */
enum class TungLamChassis : uint8_t {
  MecanumX = 0,  ///< Canonical four-wheel Mecanum-X layout.
  OmniX = 1      ///< Canonical four-wheel Omni X-drive layout.
};

/**
 * @class TungLamDrive4WD
 * @brief Modern four-wheel motor and holonomic-drive controller.
 *
 * @details
 * Coordinate convention:
 * - +vx = forward
 * - +vy = right
 * - +wz = clockwise
 *
 * Raw wheel convention:
 * - positive value = logical forward for that wheel;
 * - negative value = logical reverse;
 * - magnitude = PWM duty 0..255.
 *
 * Motor inversion is applied in the physical-output layer so the kinematic
 * equations do not need to be modified when one motor is mounted reversed.
 */
class TungLamDrive4WD {
 public:
  /** @brief Construct a controller with safe default state and zero wheel output. */
  TungLamDrive4WD();

  /**
   * @brief Configure GPIO and Timer3/Timer4 for the four drive motors.
   * @param pwmMode Desired drive PWM frequency.
   *
   * @note Call once from setup() before issuing motion commands.
   */
  void begin(TungLamPwmMode pwmMode = TungLamPwmMode::High7k8Hz);

  /**
   * @brief Service the modern non-blocking active-brake state machine.
   *
   * @note Call repeatedly from loop() when using TungLamDrive4WD::ABS() or
   * activeBrake(). This requirement applies only to the modern class; the
   * legacy TungLam_Control_MotorV5 ABS uses Timer3 ISR timing internally.
   */
  void update();

  /** @brief Select the mixer used by drive(). */
  void setChassis(TungLamChassis chassis);

  /** @brief Return the currently selected chassis mixer. */
  TungLamChassis chassis() const;

  /**
   * @brief Invert one motor in the low-level physical layer.
   * @param wheel Wheel number 1..4.
   * @param inverted true to reverse the logical sign of that wheel.
   *
   * @note Invalid wheel numbers are ignored.
   */
  void setMotorInverted(uint8_t wheel, bool inverted);

  /**
   * @brief Set dead-time inserted before an electrical direction transition.
   * @param deadTimeUs Dead-time in microseconds; 0 disables the delay.
   */
  void setDirectionDeadTimeUs(uint16_t deadTimeUs);

  /**
   * @brief Command all four wheels directly.
   * @param m1 Signed M1 demand, clamped to -255..255.
   * @param m2 Signed M2 demand, clamped to -255..255.
   * @param m3 Signed M3 demand, clamped to -255..255.
   * @param m4 Signed M4 demand, clamped to -255..255.
   *
   * This is the lowest public motion layer and bypasses chassis mixing.
   */
  void setWheels(int16_t m1, int16_t m2, int16_t m3, int16_t m4);

  /**
   * @brief Drive using the currently selected chassis mixer.
   * @param vx Forward/backward demand.
   * @param vy Right/left translation demand.
   * @param wz Clockwise/counter-clockwise rotation demand.
   */
  void drive(int16_t vx, int16_t vy, int16_t wz);

  /** @brief Apply the canonical Mecanum-X mixer directly. */
  void driveMecanum(int16_t vx, int16_t vy, int16_t wz);

  /** @brief Apply the canonical four-wheel Omni-X mixer directly. */
  void driveOmniX(int16_t vx, int16_t vy, int16_t wz);

  /** @brief Convenience helper for pure forward motion. */
  void forward(uint8_t duty);

  /** @brief Convenience helper for pure backward motion. */
  void backward(uint8_t duty);

  /** @brief Convenience helper for pure right translation. */
  void strafeRight(uint8_t duty);

  /** @brief Convenience helper for pure left translation. */
  void strafeLeft(uint8_t duty);

  /** @brief Convenience helper for pure clockwise/right rotation. */
  void rotateRight(uint8_t duty);

  /** @brief Convenience helper for pure counter-clockwise/left rotation. */
  void rotateLeft(uint8_t duty);

  /**
   * @brief Coast-stop all four motors.
   *
   * PWM is disabled and motion/brake state is cleared.
   */
  void stop();

  /** @brief Alias of stop(), explicitly describing the electrical stop mode. */
  void coast();

  /**
   * @brief Apply L298N dynamic/electrical braking.
   *
   * Both direction inputs of each bridge are driven equal while EN/PWM is high.
   * This is different from active reverse braking.
   */
  void dynamicBrake();

  /**
   * @brief Start a user-controlled active reverse-brake pulse.
   * @param brakeDuty Reverse-brake PWM selected directly by the user, 0..255.
   *
   * @details
   * The previous signed wheel directions are captured and each moving wheel is
   * driven in the opposite direction. update() later terminates the pulse.
   *
   * @warning This is intentionally a strong plug/reverse-braking mechanism.
   */
  void ABS(uint8_t brakeDuty);

  /** @brief Descriptive alias of ABS(brakeDuty). */
  void activeBrake(uint8_t brakeDuty);

  /** @brief Return true while the modern active-brake pulse is in progress. */
  bool isBraking() const;

  /** @brief Cancel any modern active/dynamic brake and coast-stop the motors. */
  void cancelBrake();

  /**
   * @brief Configure the six-stage modern active-brake timing table.
   *
   * Parameters have the same meaning/default model as legacy setTimABS().
   */
  void setTimABS(uint8_t t500,
                 uint8_t t1000,
                 uint8_t t1500,
                 uint8_t t2000,
                 uint8_t t3000,
                 uint8_t tAbove3000);

  /** @brief Descriptive alias of setTimABS(). */
  void setBrakeTimings(uint8_t t500,
                       uint8_t t1000,
                       uint8_t t1500,
                       uint8_t t2000,
                       uint8_t t3000,
                       uint8_t tAbove3000);

 private:
  /**
   * @struct Wheels
   * @brief Internal signed wheel vector in logical M1..M4 order.
   */
  struct Wheels {
    int16_t m1;  ///< Motor 1 signed command.
    int16_t m2;  ///< Motor 2 signed command.
    int16_t m3;  ///< Motor 3 signed command.
    int16_t m4;  ///< Motor 4 signed command.
  };

  TungLamChassis chassis_;  ///< Mixer selected by drive().
  TungLamPwmMode pwmMode_;  ///< Hardware PWM mode selected by begin().
  bool inverted_[4];        ///< Per-wheel physical inversion flags.
  uint16_t deadTimeUs_;     ///< Direction-transition dead-time in microseconds.

  Wheels commanded_;        ///< Latest logical command requested by the user.
  Wheels applied_;          ///< Latest physical signed command sent to hardware.
  Wheels preBrake_;         ///< Logical wheel command captured before ABS starts.

  bool moving_;             ///< True while commanded_ contains motion.
  bool braking_;            ///< True while modern reverse-brake pulse is active.
  bool dynamicBraking_;     ///< True while L298N dynamic brake state is active.
  uint32_t motionStartMs_;   ///< Start time of the current movement phase.
  uint32_t brakeStartMs_;    ///< Start time of the current active-brake pulse.
  uint8_t brakeDurationMs_;  ///< Selected active-brake pulse duration.

  uint8_t brakeT500_;       ///< Brake time for movement < 500 ms.
  uint8_t brakeT1000_;      ///< Brake time for movement < 1000 ms.
  uint8_t brakeT1500_;      ///< Brake time for movement < 1500 ms.
  uint8_t brakeT2000_;      ///< Brake time for movement < 2000 ms.
  uint8_t brakeT3000_;      ///< Brake time for movement < 3000 ms.
  uint8_t brakeTAbove3000_; ///< Brake time for movement >= 3000 ms.

  // Configure Timer3/Timer4 based on pwmMode_.
  void initPwm();

  // Convert logical wheel commands to physical commands and write hardware.
  void applyWheelsRaw(const Wheels& wheels);

  // Build and write the PORTC direction-bit pattern for all four motors.
  void writeDirectionPattern(const Wheels& wheels);

  // Write absolute PWM magnitudes to OCR3A/OCR4A/OCR4B/OCR4C.
  void writePwm(const Wheels& wheels);

  // Write one common duty to all four drive PWM compare registers.
  void writeAllPwm(uint8_t duty);

  // Choose one brake duration from the six-stage timing table.
  uint8_t selectBrakeDuration(uint32_t motionDurationMs) const;

  // Clamp a signed wheel demand to the hardware-supported -255..255 range.
  static int16_t clampWheel(int32_t value);

  // Convert a signed wheel command to an unsigned PWM magnitude.
  static uint8_t magnitude(int16_t value);

  // Return -1, 0 or +1 for a signed wheel command.
  static int8_t signOf(int16_t value);

  // Return true when at least one wheel command is non-zero.
  static bool anyMoving(const Wheels& wheels);

  // Detect an actual non-zero sign reversal on any wheel.
  static bool directionChanged(const Wheels& a, const Wheels& b);

  // Proportionally scale a four-wheel vector when any magnitude exceeds 255.
  static Wheels normalize(int32_t m1, int32_t m2, int32_t m3, int32_t m4);
};

#endif  // TUNGLAM_OMNIMECANUM_4WD_H
