/**
 * @file MetricKinematics.ino
 * @brief Beginner-to-student example for SI-unit Mecanum kinematics.
 *
 * This example shows the new v0.9 physical model:
 * - motor rated voltage;
 * - motor/gearbox no-load output RPM;
 * - actual motor supply voltage;
 * - wheel radius;
 * - wheelbase;
 * - track width;
 * - empirical open-loop speedScale.
 *
 * Coordinate frame:
 * - +X / +vx = forward
 * - +Y / +vy = left
 * - +Z / +wz = counter-clockwise (CCW) yaw
 *
 * IMPORTANT:
 * driveVelocity() is open-loop feed-forward. With no wheel encoders, the
 * library estimates PWM from the motor model; it cannot guarantee the real
 * robot moves at the requested SI velocity under load.
 */

// Import the modern four-wheel drive and kinematics API.
#include <TungLam_OmniMecanum_4WD.h>

// Create one controller for the four drive motors.
TungLamDrive4WD robot;

// Define the physical model.
//
// Example motor:
// - rated at 12 V;
// - gearbox output no-load speed = 300 RPM at 12 V.
//
// Example chassis:
// - wheel radius = 50 mm = 0.050 m;
// - front/rear wheel-centre distance = 320 mm = 0.320 m;
// - left/right wheel-centre distance = 280 mm = 0.280 m.
//
// speedScale = 0.85 is an example empirical correction for L298N voltage drop,
// battery sag and load. Measure your own robot and tune this value.
TungLamDriveConfig driveModel(
    12.0f,   // motorNominalVoltageV
    300.0f,  // motorNoLoadRpm at the gearbox/output shaft
    12.0f,   // supplyVoltageV at the motor driver input
    0.050f,  // wheelRadiusM
    0.320f,  // wheelbaseM: front wheel centre to rear wheel centre
    0.280f,  // trackWidthM: left wheel centre to right wheel centre
    0.85f    // speedScale: empirical open-loop correction
);

void setup() {
  // Open Serial Monitor so the calculated physical limits can be inspected.
  Serial.begin(115200);

  // Initialize the drive outputs with the recommended high-frequency PWM mode.
  robot.begin(TungLamPwmMode::High7k8Hz);

  // Select the Mecanum-X kinematic model.
  robot.setChassis(TungLamChassis::MecanumX);

  // Store and validate the motor + chassis physical parameters.
  const bool configOk = robot.setDriveConfig(driveModel);

  // Stop here if any required physical parameter was zero or negative.
  if (!configOk) {
    Serial.println(F("Invalid drive model. Check voltage, RPM and geometry."));
    robot.stop();
    while (true) {
      // Hold the robot stopped until reset.
    }
  }

  // Print the theoretical/estimated limits derived from the configuration.
  Serial.print(F("Estimated motor RPM at supply: "));
  Serial.println(robot.estimatedMotorRpmAtSupply());

  Serial.print(F("Estimated max wheel speed [m/s]: "));
  Serial.println(robot.maxWheelLinearSpeedMps(), 4);

  Serial.print(F("Estimated max body translation [m/s]: "));
  Serial.println(robot.maxBodyLinearSpeedMps(), 4);

  Serial.print(F("Estimated max yaw rate [rad/s]: "));
  Serial.println(robot.maxYawRateRadps(), 4);

  // Demonstrate inverse kinematics without moving the motors.
  //
  // Requested body motion:
  //   vx = +0.40 m/s  -> forward
  //   vy = +0.15 m/s  -> left
  //   wz = +0.50 rad/s -> CCW
  const TungLamWheelVelocity wheels =
      robot.inverseKinematics(0.40f, 0.15f, 0.50f);

  // Print the wheel-perimeter linear speeds produced by inverse kinematics.
  Serial.print(F("M1 [m/s]: "));
  Serial.println(wheels.m1Mps, 4);

  Serial.print(F("M2 [m/s]: "));
  Serial.println(wheels.m2Mps, 4);

  Serial.print(F("M3 [m/s]: "));
  Serial.println(wheels.m3Mps, 4);

  Serial.print(F("M4 [m/s]: "));
  Serial.println(wheels.m4Mps, 4);

  // Feed those wheel speeds back through forward kinematics.
  const TungLamBodyVelocity body = robot.forwardKinematics(wheels);

  // The recovered values should be close to the original 0.40, 0.15, 0.50.
  Serial.print(F("Recovered vx [m/s]: "));
  Serial.println(body.vxMps, 4);

  Serial.print(F("Recovered vy [m/s]: "));
  Serial.println(body.vyMps, 4);

  Serial.print(F("Recovered wz [rad/s]: "));
  Serial.println(body.wzRadps, 4);

  // Keep the robot stopped after the calculation demonstration.
  robot.stop();
}

void loop() {
  // Keep modern software state synchronized.
  robot.update();

  // Example real motion command in SI units:
  //
  // robot.driveVelocity(
  //     0.30f,  // vx: 0.30 m/s forward
  //     0.00f,  // vy: no lateral motion
  //     0.00f   // wz: no yaw rotation
  // );
  //
  // Future IMU heading-hold can add a PID yaw correction directly in rad/s:
  //
  // float wzCorrectionRadps = headingPidOutput;
  // robot.driveVelocity(vxMps, vyMps, wzManualRadps + wzCorrectionRadps);
}
