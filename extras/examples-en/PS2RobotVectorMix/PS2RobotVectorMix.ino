/**
 * @file PS2RobotVectorMix.ino
 * @brief ADVANCED example: combine translation and rotation using vx/vy/wz.
 *
 * This is NOT the default recommendation for the older RoboBall control style.
 * Use this file when you want:
 *
 *   left stick  -> vx / vy
 *   right stick -> wz
 *
 * with both sticks affecting the chassis AT THE SAME TIME.
 *
 * Example:
 * - hold LEFT UP;
 * - push RIGHT RIGHT at the same time;
 * -> the robot translates forward while rotating right through the Mecanum mixer.
 *
 * If you want the right stick to OVERRIDE the left stick like the old projects,
 * use PS2RobotControl instead.
 */

#include <TungLam_PS2.h>
#include <TungLam_OmniMecanum_4WD.h>

#define PS2_ROBOT_DEBUG 0

TungLamPS2 ps2;
TungLamDrive4WD robot;

constexpr uint8_t PS2_CS_PIN = 53;
constexpr int16_t MOVE_DUTY = 180;
constexpr int16_t TURN_DUTY = 150;

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

  if (!ps2.connected()) {
    robot.stop();
    return;
  }

  int16_t vx = 0;
  int16_t vy = 0;
  int16_t wz = 0;

  // Left stick -> translation.
  switch (ps2.leftDirection()) {
    case PS2StickDirection::Up:
      vx = MOVE_DUTY;
      break;

    case PS2StickDirection::Down:
      vx = -MOVE_DUTY;
      break;

    case PS2StickDirection::Left:
      vy = MOVE_DUTY;
      break;

    case PS2StickDirection::Right:
      vy = -MOVE_DUTY;
      break;

    default:
      break;
  }

  // Right stick -> rotation.
  switch (ps2.rightDirection()) {
    case PS2StickDirection::Left:
      wz = TURN_DUTY;
      break;

    case PS2StickDirection::Right:
      wz = -TURN_DUTY;
      break;

    default:
      break;
  }

  if (vx == 0 && vy == 0 && wz == 0) {
    robot.stop();
  } else {
    robot.drive(vx, vy, wz);
  }
}
