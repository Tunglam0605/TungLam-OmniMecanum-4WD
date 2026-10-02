/**
 * @file PS2RobotControl.ino
 * @brief RECOMMENDED example for controlling the robot base with TungLam_PS2.
 *
 * CONTROL STYLE
 * ==========================================================================
 * This follows the control style used by the older RoboBall/V5 projects:
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
 * Example:
 *
 *   1. Hold left stick UP -> robot moves forward.
 *   2. Keep holding UP and push right stick RIGHT -> robot rotates right.
 *   3. Release the right stick to CENTER -> robot resumes forward motion,
 *      because the left stick is still held UP.
 *
 * Difference from the old code:
 * - Old projects got priority because right-stick if statements ran later.
 * - This example expresses priority explicitly with return.
 *
 * PS2 WIRING - ARDUINO MEGA 2560
 * ==========================================================================
 *   PS2 DAT/MISO -> D50
 *   PS2 CMD/MOSI -> D51
 *   PS2 CLK/SCK  -> D52
 *   PS2 CS/ATT   -> D53
 *   PS2 GND      -> common GND
 *   PS2 VCC      -> receiver-required supply
 *
 * MOTOR V5 / MODERN
 * ==========================================================================
 *   M1 = front-left  = PWM D5
 *   M2 = rear-left   = PWM D6
 *   M3 = rear-right  = PWM D7
 *   M4 = front-right = PWM D8
 *
 * DIR uses D30..D37 according to the motor library.
 *
 * SAFETY
 * ==========================================================================
 * - Lost PS2 connection -> robot.stop() immediately.
 * - LEFT CENTER/UNKNOWN -> stop.
 * - No delay().
 * - Default PS2 polling is 50 Hz.
 *
 * DEBUG
 * ==========================================================================
 * Set PS2_ROBOT_DEBUG = 1 for one-line event debug.
 * At 0, the production path does not print Serial output.
 */

#include <TungLam_PS2.h>
#include <TungLam_OmniMecanum_4WD.h>

#define PS2_ROBOT_DEBUG 0

TungLamPS2 ps2;
TungLamDrive4WD robot;

constexpr uint8_t PS2_CS_PIN = 53;

constexpr uint8_t SPEED_SLOW = 120;
constexpr uint8_t SPEED_NORMAL = 180;
constexpr uint8_t SPEED_FAST = 230;

constexpr uint8_t TURN_SLOW = 110;
constexpr uint8_t TURN_NORMAL = 155;
constexpr uint8_t TURN_FAST = 200;

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

  // ------------------------------------------------------------------------
  // FAIL-SAFE
  // ------------------------------------------------------------------------
  if (!ps2.connected()) {
    robot.stop();
    return;
  }

  // ------------------------------------------------------------------------
  // SPEED - example mapping only.
  // L1 = slow, R1 = fast, neither = normal.
  // If L1 and R1 are held together, L1 has safety priority.
  // ------------------------------------------------------------------------
  uint8_t moveSpeed = SPEED_NORMAL;
  uint8_t turnSpeed = TURN_NORMAL;

  if (ps2.button(PS2Button::L1)) {
    moveSpeed = SPEED_SLOW;
    turnSpeed = TURN_SLOW;
  } else if (ps2.button(PS2Button::R1)) {
    moveSpeed = SPEED_FAST;
    turnSpeed = TURN_FAST;
  }

  // ------------------------------------------------------------------------
  // PRIORITY 1 - RIGHT STICK: ROTATION
  // ------------------------------------------------------------------------
  // When the right stick requests rotation, ignore left-stick translation.
  // Returning the right stick to center lets the left stick take control again.
  switch (ps2.rightDirection()) {
    case PS2StickDirection::Left:
      robot.rotateLeft(turnSpeed);
      return;

    case PS2StickDirection::Right:
      robot.rotateRight(turnSpeed);
      return;

    default:
      // RIGHT UP/DOWN/CENTER/UNKNOWN do not take control.
      break;
  }

  // ------------------------------------------------------------------------
  // PRIORITY 2 - LEFT STICK: TRANSLATION
  // ------------------------------------------------------------------------
  switch (ps2.leftDirection()) {
    case PS2StickDirection::Up:
      robot.forward(moveSpeed);
      break;

    case PS2StickDirection::Down:
      robot.backward(moveSpeed);
      break;

    case PS2StickDirection::Left:
      robot.strafeLeft(moveSpeed);
      break;

    case PS2StickDirection::Right:
      robot.strafeRight(moveSpeed);
      break;

    case PS2StickDirection::Center:
    case PS2StickDirection::Unknown:
    default:
      robot.stop();
      break;
  }
}
