/**
 * @file VelocityControlTemplate.ino
 * @brief SI-unit vx/vy/wz template for ROS2, Serial, PC or autonomous control.
 *
 * Implement only readVelocityCommand().
 * When no new command arrives, do not call driveVelocity(); the watchdog stops the robot.
 */

#include <TungLam_OmniMecanum_4WD.h>

TungLamDrive4WD robot;

struct BodyCommand {
  float vxMps;
  float vyMps;
  float wzRadps;
};

TungLamDriveConfig model(
    12.0f,   // motor nominal voltage [V]
    300.0f,  // gearbox output no-load RPM
    12.0f,   // supply voltage [V]
    0.050f,  // wheel radius [m]
    0.320f,  // wheelbase [m]
    0.280f,  // track width [m]
    0.85f    // open-loop speed scale
);

bool readVelocityCommand(BodyCommand &command);
void handleMechanisms();

void setup() {
  robot.begin(TungLamPwmMode::High7k8Hz);
  robot.setChassis(TungLamChassis::MecanumX);
  robot.setDriveConfig(model);

  // No command for >500 ms -> stop.
  // Slew limits: vx/vy=1.0 m/s^2, wz=2.0 rad/s^2.
  robot.enableSmartSafety(500, 1.0f, 2.0f);
}

void loop() {
  robot.update();

  BodyCommand command;
  if (readVelocityCommand(command)) {
    robot.driveVelocity(command.vxMps, command.vyMps, command.wzRadps);
  }

  handleMechanisms();
}

bool readVelocityCommand(BodyCommand &command) {
  // TODO: read one NEW command from your project source and return true.
  //
  // ROS2:
  //   command.vxMps   = cmd_vel.linear.x;
  //   command.vyMps   = cmd_vel.linear.y;
  //   command.wzRadps = cmd_vel.angular.z;
  //
  // Serial/PC:
  //   parse packet -> assign the three values above.
  //
  // Auto mode:
  //   state machine / vision / navigation -> assign target.
  //
  // Return false while no new command is available so the watchdog works.
  (void)command;
  return false;
}

void handleMechanisms() {
  // TODO: handle auxiliary mechanisms if present.
}
