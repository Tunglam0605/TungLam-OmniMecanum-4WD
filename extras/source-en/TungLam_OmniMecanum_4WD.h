/*==============================================================================
  TUNGLAM OMNI / MECANUM 4WD MOTOR LIBRARY
  ==============================================================================
  T├üC GIß║ó / AUTHOR
  ------------------------------------------------------------------------------
  Hß╗ì v├á t├¬n : Nguyß╗àn Khß║»c T├╣ng L├óm
  Lß╗¢p       : DHTD16A2CL
  MSSV       : 2210430016
  Th╞░╞íng hiß╗çu: Tung L├óm Automation

  Th╞░ viß╗çn gß╗æc:
    TungLam_Control_MotorV5
    "Th╞░ viß╗çn ─æiß╗üu khiß╗ân xe 4 b├ính ─æa h╞░ß╗¢ng"

  Th╞░ viß╗çn hiß╗çn tß║íi:
    TungLam_OmniMecanum_4WD

  Mß╗Ñc ti├¬u t╞░╞íng th├¡ch:
    - Giß╗» nguy├¬n API V5 ─æß╗â code robot c┼⌐ kh├┤ng phß║úi sß╗¡a h├ám.
    - Bß╗ò sung API hiß╗çn ─æß║íi cho Mecanum-X / Omni-X.
    - Giß╗» c╞í chß║┐ ABS h├úm ng╞░ß╗úc mß║ính, cho ph├⌐p ng╞░ß╗¥i d├╣ng tß╗▒ ─æß║╖t lß╗▒c v├á thß╗¥i gian.
    - Cß║úi tiß║┐n ABS legacy th├ánh non-blocking m├á code c┼⌐ kh├┤ng cß║ºn gß╗ìi update().

  ==============================================================================
  S╞á ─Éß╗Æ ─Éß║ñU Nß╗ÉI V5 - ─É├â ─Éß╗ÉI CHIß║╛U Vß╗ÜI IMPLEMENTATION
  (Giß╗» bß╗æ cß╗Ñc quen thuß╗Öc cß╗ºa V5 nh╞░ng sß╗¡a mapping ─æß╗â khß╗¢p ch├¡nh x├íc code AVR.)
  ------------------------------------------------------------------------------

  CH├éN PWM ------ CH├éN Sß╗É
    EN B├üNH 1 ----- 5
    EN B├üNH 2 ----- 6
    EN B├üNH 3 ----- 7
    EN B├üNH 4 ----- 8

  CH├éN CHIß╗ÇU DIR (B├üNH) ---- CH├éN CHIß╗ÇU TIß║╛N ---- CH├éN CHIß╗ÇU L├ÖI
              DIR1                    30                   31
              DIR2                    32                   33
              DIR3                    34                   35
              DIR4                    37                   36

  ==============================================================================
  MAPPING THß╗░C Tß║╛ THEO CODE AVR HIß╗åN Tß║áI
  ------------------------------------------------------------------------------
  Arduino Mega 2560 ├ính xß║í PORTC nh╞░ sau:

    D30 = PC7     D31 = PC6
    D32 = PC5     D33 = PC4
    D34 = PC3     D35 = PC2
    D36 = PC1     D37 = PC0

  Implementation thß╗▒c tß║┐ cß╗ºa th╞░ viß╗çn:

    Motor   PWM / EN       CHIß╗ÇU TIß║╛N (+)       CHIß╗ÇU L├ÖI (-)
    -----   -------------  -------------------  -------------------
    M1      D5  / OC3A     D30 / PC7            D31 / PC6
    M2      D6  / OC4A     D32 / PC5            D33 / PC4
    M3      D7  / OC4B     D34 / PC3            D35 / PC2
    M4      D8  / OC4C     D37 / PC0            D36 / PC1

  ==============================================================================
  QUY ╞»ß╗ÜC THß╗¿ Tß╗░ 4 B├üNH - API HIß╗åN ─Éß║áI
  ------------------------------------------------------------------------------

                        ─Éß║ªU XE / FRONT
                             +vx
                              ^
                              |
                 M1                         M4
            FRONT-LEFT                 FRONT-RIGHT
               D5                         D8

                 M2                         M3
             REAR-LEFT                  REAR-RIGHT
               D6                         D7
                              |
                       ─ÉU├öI XE / REAR

    +vx : tiß║┐n / forward
    -vx : l├╣i / backward
    +vy : ngang tr├íi / strafe left
    -vy : ngang phß║úi / strafe right
    +wz : quay tr├íi, ng╞░ß╗úc chiß╗üu kim ─æß╗ông hß╗ô / counter-clockwise
    -wz : quay phß║úi, chiß╗üu kim ─æß╗ông hß╗ô / clockwise

  Quy ╞░ß╗¢c tr├¬n l├á thß╗⌐ tß╗▒ logic d├╣ng bß╗ƒi mixer Mecanum-X v├á Omni-X hiß╗çn ─æß║íi.
  Vß╗¢i robot V5 c┼⌐ ─æ├ú ─æß║Ñu d├óy v├á chß║íy ß╗òn ─æß╗ïnh, n├¬n giß╗» nguy├¬n wiring thß╗▒c tß║┐
  cß╗ºa xe c┼⌐; compatibility API sß║╜ giß╗» nguy├¬n h├ánh vi V5.

  ==============================================================================
  QUICK CLASS REFERENCE
  ------------------------------------------------------------------------------

  Code c┼⌐ / Legacy:
    #include <TungLam_Control_MotorV5.h>
    TungLam_Control_MotorV5 motor;

  Code mß╗¢i / Modern:
    #include <TungLam_OmniMecanum_4WD.h>
    TungLamDrive4WD motor;

  ==============================================================================
  ABS H├âM NG╞»ß╗óC / ACTIVE REVERSE BRAKE
  ------------------------------------------------------------------------------

  Legacy:
    motor.ABS(duty);
    motor.setTimABS(45, 65, 70, 75, 80, 85);

    - duty: lß╗▒c h├úm ng╞░ß╗úc do ng╞░ß╗¥i d├╣ng tß╗▒ chß╗ìn 0..255.
    - setTimABS(): thß╗¥i gian h├úm theo 6 khoß║úng thß╗¥i gian chß║íy.
    - Legacy ABS chß║íy non-blocking bß║▒ng Timer3 overflow interrupt.
    - Code V5 c┼⌐ KH├öNG cß║ºn th├¬m update().

  Modern:
    motor.ABS(duty);

    - Modern ABS c┼⌐ng d├╣ng Timer3 overflow one-shot ─æß╗â tß╗▒ ngß║»t ─æ├║ng hß║ín.
    - update() vß║½n ─æ╞░ß╗úc giß╗» ─æß╗â ─æß╗ông bß╗Ö state phß║ºn mß╗üm, nh╞░ng KH├öNG c├▓n bß║»t buß╗Öc
      ─æß╗â cß║»t lß╗▒c h├úm ß╗ƒ mß╗⌐c phß║ºn cß╗⌐ng.

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
 *    - Modern active braking uses the same hardware-timed Timer3 cutoff as legacy.
 *
 * Hardware mapping used by both APIs:
 *
 *   Motor   PWM pin   AVR output      Forward DIR   Reverse DIR
 *   -----   -------   --------------  -----------   -----------
 *   M1      D5        PE3 / OC3A      D30 / PC7     D31 / PC6
 *   M2      D6        PH3 / OC4A      D32 / PC5     D33 / PC4
 *   M3      D7        PH4 / OC4B      D34 / PC3     D35 / PC2
 *   M4      D8        PH5 / OC4C      D37 / PC0     D36 / PC1
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
  /**
   * @brief Forward-left diagonal with independent PWM values for M1..M4.
   * @param duty1 PWM for M1 front-left, 0..255.
   * @param duty2 PWM for M2 rear-left, 0..255.
   * @param duty3 PWM for M3 rear-right, 0..255.
   * @param duty4 PWM for M4 front-right, 0..255.
   */
  void T_Trai(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);
  /**
   * @brief Forward-right diagonal with independent PWM values for M1..M4.
   * @param duty1 PWM for M1 front-left, 0..255.
   * @param duty2 PWM for M2 rear-left, 0..255.
   * @param duty3 PWM for M3 rear-right, 0..255.
   * @param duty4 PWM for M4 front-right, 0..255.
   */
  void T_Phai(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);
  /**
   * @brief Backward-left diagonal with independent PWM values for M1..M4.
   * @param duty1 PWM for M1 front-left, 0..255.
   * @param duty2 PWM for M2 rear-left, 0..255.
   * @param duty3 PWM for M3 rear-right, 0..255.
   * @param duty4 PWM for M4 front-right, 0..255.
   */
  void L_Trai(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);
  /**
   * @brief Backward-right diagonal with independent PWM values for M1..M4.
   * @param duty1 PWM for M1 front-left, 0..255.
   * @param duty2 PWM for M2 rear-left, 0..255.
   * @param duty3 PWM for M3 rear-right, 0..255.
   * @param duty4 PWM for M4 front-right, 0..255.
   */
  void L_Phai(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);
  /**
   * @brief Translate left with independent PWM values for M1..M4.
   * @param duty1 PWM for M1 front-left, 0..255.
   * @param duty2 PWM for M2 rear-left, 0..255.
   * @param duty3 PWM for M3 rear-right, 0..255.
   * @param duty4 PWM for M4 front-right, 0..255.
   */
  void N_Trai(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);
  /**
   * @brief Translate right with independent PWM values for M1..M4.
   * @param duty1 PWM for M1 front-left, 0..255.
   * @param duty2 PWM for M2 rear-left, 0..255.
   * @param duty3 PWM for M3 rear-right, 0..255.
   * @param duty4 PWM for M4 front-right, 0..255.
   */
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
  // Reset one AVR timer register set before reconfiguration.
  void Reset_Timer(uint8_t timerNumber);

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
  MecanumX = 0,  ///< Four-wheel Mecanum-X using the documented M1..M4 order.
  OmniX = 1      ///< Canonical four-wheel 45-degree Omni X-drive layout.
};

/**
 * @struct TungLamDriveConfig
 * @brief Physical parameters used by the SI-unit kinematics/open-loop model.
 *
 * @details
 * motorNoLoadRpm is the gearbox/output-shaft no-load speed at
 * motorNominalVoltageV. speedScale is an empirical correction factor for
 * driver voltage drop, load, battery sag and other real-world losses.
 *
 * A value of 1.0f means ideal theoretical behavior. For open-loop operation
 * this model estimates wheel speed; it does not measure actual speed.
 */
struct TungLamDriveConfig {
  float motorNominalVoltageV;  ///< Rated voltage associated with motorNoLoadRpm.
  float motorNoLoadRpm;        ///< No-load gearbox/output-shaft RPM at rated voltage.
  float supplyVoltageV;        ///< Motor supply voltage applied to the H-bridge.
  float wheelRadiusM;          ///< Effective wheel radius in metres.
  float wheelbaseM;            ///< Front-to-rear wheel-centre distance in metres.
  float trackWidthM;           ///< Left-to-right wheel-centre distance in metres.
  float speedScale;            ///< Empirical speed correction; 1.0 = ideal model.

  TungLamDriveConfig(float motorVoltageV = 0.0f,
                     float noLoadRpm = 0.0f,
                     float supplyV = 0.0f,
                     float radiusM = 0.0f,
                     float wheelbase = 0.0f,
                     float trackWidth = 0.0f,
                     float calibrationScale = 1.0f)
      : motorNominalVoltageV(motorVoltageV),
        motorNoLoadRpm(noLoadRpm),
        supplyVoltageV(supplyV),
        wheelRadiusM(radiusM),
        wheelbaseM(wheelbase),
        trackWidthM(trackWidth),
        speedScale(calibrationScale) {}
};

/**
 * @struct TungLamWheelVelocity
 * @brief Signed wheel-perimeter linear velocities in metres per second.
 */
struct TungLamWheelVelocity {
  float m1Mps;  ///< M1 front-left wheel-perimeter speed.
  float m2Mps;  ///< M2 rear-left wheel-perimeter speed.
  float m3Mps;  ///< M3 rear-right wheel-perimeter speed.
  float m4Mps;  ///< M4 front-right wheel-perimeter speed.
};

/**
 * @struct TungLamBodyVelocity
 * @brief Robot body velocity in the standard right-handed mobile-robot frame.
 */
struct TungLamBodyVelocity {
  float vxMps;    ///< +X forward, metres per second.
  float vyMps;    ///< +Y left, metres per second.
  float wzRadps;  ///< +Z yaw counter-clockwise, radians per second.
};

/**
 * @class TungLamDrive4WD
 * @brief Modern four-wheel motor and holonomic-drive controller.
 *
 * @details
 * Coordinate convention (right-handed body frame / ROS-style):
 * - +X / +vx = forward
 * - +Y / +vy = left
 * - +Z / +wz = counter-clockwise yaw when viewed from above
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
   * @brief Synchronize modern software state after a hardware-timed brake pulse.
   *
   * Timer3 overflow ISR now terminates the physical ABS pulse independently of
   * loop() latency. Calling update() is therefore optional for brake safety;
   * it only clears stale high-level state after the ISR has already stopped
   * the motors.
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
   * @brief Drive using normalized body-frame commands and the selected mixer.
   * @param vx +255 forward, -255 backward.
   * @param vy +255 left, -255 right.
   * @param wz +255 counter-clockwise, -255 clockwise.
   *
   * @note These are normalized command units, not SI velocity units. Use
   * driveVelocity() for metres/second and radians/second.
   */
  void drive(int16_t vx, int16_t vy, int16_t wz);

  /**
   * @brief Configure the physical model used by SI-unit kinematics.
   * @return true when all required parameters are strictly positive.
   *
   * Invalid configurations are rejected and do not replace the previous model.
   */
  bool setDriveConfig(const TungLamDriveConfig& config);

  /** @brief Return the currently stored physical drive configuration. */
  const TungLamDriveConfig& driveConfig() const;

  /** @brief Return true when a valid SI/open-loop drive model is configured. */
  bool hasDriveConfig() const;

  /**
   * @brief Estimate motor output RPM available at the configured supply voltage.
   *
   * This is motorNoLoadRpm * supplyVoltage / nominalVoltage * speedScale.
   */
  float estimatedMotorRpmAtSupply() const;

  /** @brief Estimated maximum wheel-perimeter speed in m/s. */
  float maxWheelLinearSpeedMps() const;

  /** @brief Estimated maximum pure +X/-X body speed in m/s for the selected chassis. */
  float maxBodyLinearSpeedMps() const;

  /** @brief Estimated maximum pure yaw rate in rad/s for the selected chassis. */
  float maxYawRateRadps() const;

  /**
   * @brief Compute inverse kinematics without commanding the hardware.
   * @param vxMps Body +X forward velocity in m/s.
   * @param vyMps Body +Y left velocity in m/s.
   * @param wzRadps Body +Z CCW yaw rate in rad/s.
   * @return Signed wheel-perimeter speeds in m/s.
   */
  TungLamWheelVelocity inverseKinematics(float vxMps,
                                         float vyMps,
                                         float wzRadps) const;

  /**
   * @brief Compute forward kinematics from wheel-perimeter speeds.
   *
   * This helper is useful for teaching and is also the intended future bridge
   * for encoder odometry. It does not read any sensors by itself.
   */
  TungLamBodyVelocity forwardKinematics(
      const TungLamWheelVelocity& wheels) const;

  /**
   * @brief Command body velocity in SI units using open-loop feed-forward.
   * @param vxMps +X forward velocity in m/s.
   * @param vyMps +Y left velocity in m/s.
   * @param wzRadps +Z CCW yaw rate in rad/s.
   * @return false when no valid drive model is configured; otherwise true.
   *
   * Requested wheel speeds are proportionally scaled when they exceed the
   * estimated available wheel speed, preserving the requested motion vector.
   *
   * @warning With no wheel encoders this is an estimate, not closed-loop speed
   * control. Real velocity changes with load, battery, friction and L298N loss.
   */
  bool driveVelocity(float vxMps, float vyMps, float wzRadps);

  /**
   * @brief Enable command watchdog and SI velocity smoothing with one call.
   * @param timeoutMs Command timeout [ms]; 0 disables watchdog.
   * @param linearAccelMps2 Maximum vx/vy slew rate [m/s^2]; <=0 disables smoothing.
   * @param yawAccelRadps2 Maximum wz slew rate [rad/s^2]; <=0 disables smoothing.
   */
  void enableSmartSafety(uint16_t timeoutMs = 500,
                         float linearAccelMps2 = 1.0f,
                         float yawAccelRadps2 = 2.0f);

  /** @brief Disable watchdog and SI velocity smoothing. */
  void disableSmartSafety();

  /** @brief Configure command-loss watchdog. @param timeoutMs Timeout [ms]; 0 disables it. */
  void setCommandTimeoutMs(uint16_t timeoutMs);

  /** @brief Return true if watchdog stopped the robot after command loss. */
  bool commandTimedOut() const;

  /**
   * @brief Enable SI body-velocity slew-rate limiting.
   * @param linearAccelMps2 Maximum vx/vy slew rate [m/s^2].
   * @param yawAccelRadps2 Maximum wz slew rate [rad/s^2].
   * @return true for valid positive limits; false disables the ramp.
   */
  bool setVelocityRamp(float linearAccelMps2, float yawAccelRadps2);

  /** @brief Disable SI velocity slew-rate limiting. */
  void disableVelocityRamp();

  /** @brief Return true if the latest SI command required wheel-speed scaling. */
  bool wasVelocityLimited() const;

  /** @brief Return latest SI saturation scale; 1.0 means no scaling. */
  float lastVelocityScale() const;

  /** @brief Return requested body velocity before shaping. */
  TungLamBodyVelocity requestedBodyVelocity() const;

  /** @brief Return estimated applied body velocity after shaping/saturation. */
  TungLamBodyVelocity appliedBodyVelocity() const;

  /**
   * @brief Apply the TungLam/V5-compatible Mecanum-X mixer directly.
   *
   * Standard body-frame basis vectors:
   * - +vx forward = [+,+,+,+]
   * - +vy left    = [-,+,-,+]
   * - +wz CCW     = [-,-,+,+]
   *
   * These vectors preserve the proven V5 physical movement patterns while
   * assigning the modern vx/vy/wz signs to the right-handed robot convention.
   */

  void driveMecanum(int16_t vx, int16_t vy, int16_t wz);

  /** @brief Apply the canonical four-wheel Omni-X mixer directly. */
  void driveOmniX(int16_t vx, int16_t vy, int16_t wz);

  /** @brief Convenience helper for pure forward motion. */
  void forward(uint8_t duty);

  /** @brief Convenience helper for pure backward motion. */
  void backward(uint8_t duty);

  /** @brief Convenience helper for right translation (-vy). */
  void strafeRight(uint8_t duty);

  /** @brief Convenience helper for left translation (+vy). */
  void strafeLeft(uint8_t duty);

  /** @brief Convenience helper for clockwise/right rotation (-wz). */
  void rotateRight(uint8_t duty);

  /** @brief Convenience helper for counter-clockwise/left rotation (+wz). */
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
   * driven in the opposite direction. Timer3 overflow ISR terminates the pulse
   * at the configured deadline even if loop() is blocked.
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
  TungLamDriveConfig driveConfig_; ///< SI/open-loop physical drive model.
  bool driveConfigValid_;   ///< True after a valid physical model is configured.

  float cachedLeverM_;
  float cachedMotorRpm_;
  float cachedMaxWheelMps_;
  float cachedPwmPerMps_;
  float cachedMaxBodyMps_;
  float cachedMaxYawRadps_;

  TungLamBodyVelocity requestedBodyVelocity_;
  TungLamBodyVelocity rampedBodyVelocity_;
  TungLamBodyVelocity appliedBodyVelocity_;
  float lastVelocityScale_;
  bool velocityLimited_;

  uint16_t commandTimeoutMs_;
  uint32_t lastCommandMs_;
  bool commandTimedOut_;
  bool velocityRampEnabled_;
  bool velocityRampPrimed_;
  float linearAccelMps2_;
  float yawAccelRadps2_;
  uint32_t lastVelocityUpdateMs_;

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

  // Choose one brake duration from the six-stage timing table.
  uint8_t selectBrakeDuration(uint32_t motionDurationMs) const;

  // Clamp a signed wheel demand to the hardware-supported -255..255 range.
  static int16_t clampWheel(int32_t value);

  // Return -1, 0 or +1 for a signed wheel command.
  static int8_t signOf(int16_t value);

  // Return true when at least one wheel command is non-zero.
  static bool anyMoving(const Wheels& wheels);

  // Detect an actual non-zero sign reversal on any wheel.
  static bool directionChanged(const Wheels& a, const Wheels& b);

  // Validate positive physical-model parameters.
  static bool driveConfigIsValid(const TungLamDriveConfig& config);

  void rebuildDerivedModel();
  void rebuildChassisLimits();
  void noteCommandReceived();
  void resetVelocityRampState();
  TungLamBodyVelocity applyVelocityRamp(const TungLamBodyVelocity& target,
                                        uint32_t nowMs);
  static float approachFloat(float current, float target, float maxDelta);

  // Convert one signed wheel-perimeter velocity to signed PWM feed-forward.
  int16_t wheelVelocityToPwm(float wheelMps) const;

  // Proportionally scale a metric wheel vector to the available wheel speed.
  TungLamWheelVelocity limitWheelVelocities(
      const TungLamWheelVelocity& wheels,
      float* scaleOut = nullptr) const;

  // Proportionally scale a four-wheel vector when any magnitude exceeds 255.
  static Wheels normalize(int32_t m1, int32_t m2, int32_t m3, int32_t m4);
};

#endif  // TUNGLAM_OMNIMECANUM_4WD_H
