/**
 * @file VelocityControlTemplate.ino
 * @brief Template cho ROS2/Serial/PC/auto mode dùng vx, vy, wz theo đơn vị SI.
 *
 * Người dùng chỉ cần hiện thực readVelocityCommand().
 * Nếu không có lệnh mới, không gọi driveVelocity(); watchdog sẽ tự dừng robot.
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

  // Mất lệnh >500 ms -> stop.
  // Giới hạn slew vx/vy=1.0 m/s^2, wz=2.0 rad/s^2.
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
  // TODO: đọc một lệnh MỚI từ nguồn của project rồi return true.
  //
  // ROS2:
  //   command.vxMps   = cmd_vel.linear.x;
  //   command.vyMps   = cmd_vel.linear.y;
  //   command.wzRadps = cmd_vel.angular.z;
  //
  // Serial/PC:
  //   parse packet -> gán 3 giá trị trên.
  //
  // Auto mode:
  //   state machine / vision / navigation -> gán target.
  //
  // Khi chưa có lệnh mới phải return false để watchdog hoạt động đúng.
  (void)command;
  return false;
}

void handleMechanisms() {
  // TODO: xử lý cơ cấu phụ nếu có.
}
