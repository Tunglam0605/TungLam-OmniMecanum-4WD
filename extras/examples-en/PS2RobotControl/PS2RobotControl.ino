/**
 * @file PS2RobotControl.ino
 * @brief RECOMMENDED example: RoboBall/V5-style robot-base control.
 *
 * PURPOSE
 * ==========================================================================
 * This example focuses only on DRIVING LOGIC, without auxiliary mechanisms:
 *
 *   LEFT STICK = primary translation
 *     UP    -> forward
 *     DOWN  -> backward
 *     LEFT  -> strafe left
 *     RIGHT -> strafe right
 *
 *   RIGHT STICK = rotation with HIGHER PRIORITY than the left stick
 *     LEFT  -> rotate left / CCW
 *     RIGHT -> rotate right / CW
 *
 * IMPORTANT BEHAVIOR
 * ==========================================================================
 * 1. Hold left stick UP -> robot moves forward.
 * 2. Keep holding UP and push right stick RIGHT -> robot rotates right.
 * 3. Return the right stick to CENTER -> forward motion resumes because the
 *    left stick is still held UP.
 *
 * This mirrors the effective priority used by older RoboBall projects, but
 * expresses it explicitly with return instead of depending on statement order.
 *
 * PS2 WIRING - ARDUINO MEGA 2560
 * ==========================================================================
 *   PS2 DAT/MISO -> D50
 *   PS2 CMD/MOSI -> D51
 *   PS2 CLK/SCK  -> D52
 *   PS2 CS/ATT   -> D53
 *   PS2 GND      -> common GND
 *   PS2 VCC      -> supply required by the receiver
 *
 * V5 MOTOR BASELINE
 * ==========================================================================
 *   M1 = front-left  = PWM D5
 *   M2 = rear-left   = PWM D6
 *   M3 = rear-right  = PWM D7
 *   M4 = front-right = PWM D8
 *
 *   Code order      : M1, M2, M3, M4
 *   Clockwise order : M1 -> M4 -> M3 -> M2
 *
 * SAFETY
 * ==========================================================================
 * - Lost PS2 connection -> robot.stop() immediately.
 * - The right stick takes priority only for LEFT/RIGHT rotation commands.
 * - LEFT stick CENTER/UNKNOWN -> stop.
 * - No delay().
 * - Default PS2 polling is 50 Hz.
 *
 * DEBUG
 * ==========================================================================
 * Set PS2_ROBOT_DEBUG = 1 to enable ps2.debug(Serial).
 * Debug prints only on changes; production should keep it at 0.
 *
 * For button(), pressed(), released(), raw analog, or reconnect examples,
 * see the TungLam_PS2 library examples.
 */

#include <TungLam_PS2.h>
#include <TungLam_OmniMecanum_4WD.h>

#define PS2_ROBOT_DEBUG 0

TungLamPS2 ps2;
TungLamDrive4WD robot;

constexpr uint8_t PS2_CS_PIN = 53;
constexpr uint8_t MOVE_SPEED = 180;
constexpr uint8_t TURN_SPEED = 155;

void setup() {
#if PS2_ROBOT_DEBUG
  Serial.begin(115200);
#endif

  robot.begin();
  robot.setChassis(TungLamChassis::MecanumX);

  ps2.begin(PS2_CS_PIN);
  ps2.setPollRate(PS2PollRate::Hz50);
}

void loop() {
  ps2.update();
  robot.update();

#if PS2_ROBOT_DEBUG
  ps2.debug(Serial);
#endif

  // FAIL-SAFE: do not keep stale commands after controller loss.
  if (!ps2.connected()) {
    robot.stop();
    return;
  }

  // ========================================================================
  // PRIORITY 1 - RIGHT STICK: ROTATION
  // ========================================================================
  // A rotation request is executed immediately and returns from loop().
  // Therefore right-stick rotation always overrides left-stick translation.
  switch (ps2.rightDirection()) {
    case PS2StickDirection::Left:
      robot.rotateLeft(TURN_SPEED);
      return;

    case PS2StickDirection::Right:
      robot.rotateRight(TURN_SPEED);
      return;

    default:
      // UP/DOWN/CENTER/UNKNOWN do not take control.
      break;
  }

  // ========================================================================
  // PRIORITY 2 - LEFT STICK: TRANSLATION
  // ========================================================================
  switch (ps2.leftDirection()) {
    case PS2StickDirection::Up:
      robot.forward(MOVE_SPEED);
      break;

    case PS2StickDirection::Down:
      robot.backward(MOVE_SPEED);
      break;

    case PS2StickDirection::Left:
      robot.strafeLeft(MOVE_SPEED);
      break;

    case PS2StickDirection::Right:
      robot.strafeRight(MOVE_SPEED);
      break;

    case PS2StickDirection::Center:
    case PS2StickDirection::Unknown:
    default:
      robot.stop();
      break;
  }
}
