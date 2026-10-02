/**
 * @file PS2RobotControl.ino
 * @brief Control a Mecanum/Omni drive base directly with TungLam_PS2.
 *
 * PURPOSE
 * ==========================================================================
 * This example combines both libraries while keeping them independent:
 *
 *   PS2 controller
 *        |
 *        v
 *   TungLam_PS2
 *        |
 *        | filtered joystick/button state
 *        v
 *   application mapping
 *        |
 *        | normalized vx, vy, wz
 *        v
 *   TungLam_OmniMecanum_4WD
 *        |
 *        v
 *   4 motors
 *
 * PS2 WIRING ON ARDUINO MEGA 2560
 * ==========================================================================
 *   PS2 DAT/MISO -> D50
 *   PS2 CMD/MOSI -> D51
 *   PS2 CLK/SCK  -> D52
 *   PS2 CS/ATT   -> D53
 *   PS2 GND      -> common GND
 *   PS2 VCC      -> supply required by the receiver
 *
 * MOTOR LIBRARY PINS
 * ==========================================================================
 *   PWM: M1=D5, M2=D6, M3=D7, M4=D8
 *   DIR: D30..D37 according to TungLam_OmniMecanum_4WD documentation
 *
 * The two pin groups do not conflict.
 *
 * MAPPING USED BY THIS EXAMPLE
 * ==========================================================================
 * Left stick:
 *   UP    -> forward
 *   DOWN  -> backward
 *   LEFT  -> strafe left
 *   RIGHT -> strafe right
 *
 * Right stick:
 *   LEFT  -> rotate left / CCW
 *   RIGHT -> rotate right / CW
 *
 * Buttons:
 *   hold L1 -> slow speed
 *   hold R1 -> fast speed
 *   START   -> toggle drive enable/disable
 *
 * This mapping belongs ONLY to the example. TungLam_PS2 never hard-codes
 * application behavior to any button.
 *
 * SAFETY
 * ==========================================================================
 * - Lost PS2 connection -> robot.stop() immediately.
 * - Unknown joystick state -> corresponding axis becomes zero.
 * - No delay().
 * - Default controller polling is 50 Hz.
 *
 * DEBUG
 * ==========================================================================
 * Set PS2_ROBOT_DEBUG to 1 to enable Serial 115200 event debugging:
 *
 *   [PS2] BTN=START:PRESSED | LEFT=UP(0,-117)
 *
 * At 0, the production path does not call Serial debug.
 */

#include <TungLam_PS2.h>
#include <TungLam_OmniMecanum_4WD.h>

#define PS2_ROBOT_DEBUG 0

TungLamPS2 ps2;
TungLamDrive4WD robot;

constexpr uint8_t PS2_CS_PIN = 53;

// Example normalized PWM values. Tune for the real robot.
constexpr int16_t SPEED_SLOW = 100;
constexpr int16_t SPEED_NORMAL = 170;
constexpr int16_t SPEED_FAST = 220;
constexpr int16_t TURN_NORMAL = 140;
constexpr int16_t TURN_FAST = 190;

bool driveEnabled = true;

void setup() {
#if PS2_ROBOT_DEBUG
  Serial.begin(115200);
#endif

  // Initialize the drive base.
  robot.begin();
  robot.setChassis(TungLamChassis::MecanumX);

  // Initialize PS2 using the Mega default hardware SPI.
  // DAT=D50, CMD=D51, CLK=D52, CS=D53.
  ps2.begin(PS2_CS_PIN);
  ps2.setPollRate(PS2PollRate::Hz50);
}

void loop() {
  // Call both update methods continuously, without delay().
  ps2.update();
  robot.update();

#if PS2_ROBOT_DEBUG
  // Prints only controller changes; it does not spam Serial.
  ps2.debug(Serial);
#endif

  // Fail-safe: lost receiver/controller -> stop immediately.
  if (!ps2.connected()) {
    robot.stop();
    return;
  }

  // pressed() is one-shot, so START toggles exactly once per press.
  if (ps2.pressed(PS2Button::Start)) {
    driveEnabled = !driveEnabled;

    if (!driveEnabled) {
      robot.stop();
    }
  }

  if (!driveEnabled) {
    robot.stop();
    return;
  }

  // ------------------------------------------------------------------------
  // 1. Select speed using held buttons.
  // ------------------------------------------------------------------------
  int16_t moveDuty = SPEED_NORMAL;
  int16_t turnDuty = TURN_NORMAL;

  // If L1 and R1 are both held, L1 has safety priority and slows down.
  if (ps2.button(PS2Button::L1)) {
    moveDuty = SPEED_SLOW;
    turnDuty = SPEED_SLOW;
  } else if (ps2.button(PS2Button::R1)) {
    moveDuty = SPEED_FAST;
    turnDuty = TURN_FAST;
  }

  // ------------------------------------------------------------------------
  // 2. Left stick -> vx, vy.
  // Motor-library convention:
  //   +vx = forward
  //   +vy = left
  // ------------------------------------------------------------------------
  int16_t vx = 0;
  int16_t vy = 0;

  switch (ps2.leftDirection()) {
    case PS2StickDirection::Up:
      vx = moveDuty;
      break;

    case PS2StickDirection::Down:
      vx = -moveDuty;
      break;

    case PS2StickDirection::Left:
      vy = moveDuty;
      break;

    case PS2StickDirection::Right:
      vy = -moveDuty;
      break;

    case PS2StickDirection::Center:
    case PS2StickDirection::Unknown:
    default:
      // Center/Unknown -> no translation.
      break;
  }

  // ------------------------------------------------------------------------
  // 3. Right-stick horizontal direction -> wz.
  // Motor-library convention:
  //   +wz = rotate left / CCW
  //   -wz = rotate right / CW
  // ------------------------------------------------------------------------
  int16_t wz = 0;

  switch (ps2.rightDirection()) {
    case PS2StickDirection::Left:
      wz = turnDuty;
      break;

    case PS2StickDirection::Right:
      wz = -turnDuty;
      break;

    default:
      // Right-stick UP/DOWN/CENTER/UNKNOWN do not rotate in this example.
      break;
  }

  // ------------------------------------------------------------------------
  // 4. Send one vector to the chassis mixer.
  // ------------------------------------------------------------------------
  // drive(vx, vy, wz) permits translation and rotation simultaneously.
  // Do not call forward() and then rotate() because the later command can
  // replace the earlier command.
  if (vx == 0 && vy == 0 && wz == 0) {
    robot.stop();
  } else {
    robot.drive(vx, vy, wz);
  }
}
