[🇻🇳 Tiếng Việt](KINEMATICS.md) • [🌐 English](KINEMATICS.en.md)

# Kinematics, SI velocity, and open-loop motor model

This document explains the physical model behind the modern API.

It is written for two audiences:

- beginners who only want to enter motor/chassis dimensions and call `driveVelocity()`;
- students who want to understand the equations that convert robot body velocity into individual wheel velocity.

---

## 1. Standard robot coordinate frame

The modern API uses a right-handed body frame compatible with common mobile-robot and ROS conventions:

```text
                 +X / +vx
                  forward
                     ^
                     |
                     |
          +Y  <--- ROBOT
          left
                     ⊙ +Z

Viewed from above:

+wz = counter-clockwise / turn left
-wz = clockwise / turn right
```

Therefore:

```text
+vx = forward
-vx = backward

+vy = left
-vy = right

+wz = counter-clockwise (CCW)
-wz = clockwise (CW)
```

This convention applies only to the modern Cartesian APIs.

The legacy `TungLam_Control_MotorV5` movement functions keep their historical behavior.

---

## 2. Wheel numbering

> The hardware-proven M1..M4 mapping and Mecanum command basis are locked in **[V5_BASELINE.en.md](V5_BASELINE.en.md)**.

```text
                     FRONT

              M1             M4
          front-left     front-right

              M2             M3
           rear-left      rear-right

                      REAR
```

Important ordering rule:

```text
Code/vector order        : M1, M2, M3, M4
Physical positions       : front-left, rear-left, rear-right, front-right
Clockwise physical order : M1 -> M4 -> M3 -> M2
```

All `v1..v4` equations use **channel order M1..M4**, not clockwise chassis order.

Physical dimensions:

```text
wheelbaseM
= distance from the front wheel-centre line
  to the rear wheel-centre line

trackWidthM
= distance from the left wheel-centre line
  to the right wheel-centre line

wheelRadiusM
= effective rolling radius of one wheel
```

For the equations we use:

```text
L = wheelbaseM / 2
W = trackWidthM / 2
K = L + W
  = (wheelbaseM + trackWidthM) / 2
```

---

# 3. Beginner setup

Example:

```cpp
TungLamDriveConfig driveModel(
    12.0f,   // motor rated voltage [V]
    300.0f,  // gearbox/output no-load RPM at rated voltage
    12.0f,   // motor-driver supply voltage [V]
    0.050f,  // wheel radius [m]
    0.320f,  // wheelbase: front-centre to rear-centre [m]
    0.280f,  // track width: left-centre to right-centre [m]
    0.85f    // empirical open-loop speed correction
);

robot.begin();
robot.setChassis(TungLamChassis::MecanumX);
robot.setDriveConfig(driveModel);
```

Then command physical velocity:

```cpp
robot.driveVelocity(
    0.40f,  // vx = 0.40 m/s forward
    0.10f,  // vy = 0.10 m/s left
    0.50f   // wz = 0.50 rad/s CCW
);
```

---

# 4. Meaning of the motor parameters

## `motorNominalVoltageV`

Rated voltage associated with the RPM specification.

Examples:

```text
12 V motor -> 12.0f
24 V motor -> 24.0f
```

## `motorNoLoadRpm`

No-load rotational speed at the final output shaft that directly drives the wheel.

For a geared DC motor, use the **gearbox output RPM**, not the internal motor rotor RPM.

Example:

```text
Motor label:
12 V
300 RPM

motorNominalVoltageV = 12.0
motorNoLoadRpm       = 300.0
```

If an additional external gearbox or belt reduction exists between the specified shaft and the wheel, convert the RPM to the final wheel-shaft RPM first.

## `supplyVoltageV`

Voltage supplied to the motor H-bridge.

Example:

```text
12 V battery / driver supply:
supplyVoltageV = 12.0
```

This value is used for theoretical speed scaling. It is not permission to over-voltage a motor beyond its safe rating.

## `speedScale`

Empirical correction for the difference between datasheet no-load speed and the real robot.

Typical losses include:

- L298N voltage drop;
- battery sag;
- motor load;
- gearbox loss;
- wheel deformation;
- carpet/floor friction;
- manufacturing variation.

Start with:

```cpp
speedScale = 1.0f;
```

Then measure the real robot and reduce or adjust the value.

Example:

```cpp
speedScale = 0.85f;
```

means the open-loop model assumes the real available speed is about 85% of the ideal estimate.

---

# 5. Motor RPM estimate

For a brushed DC motor, no-load speed is approximately proportional to applied voltage.

The library estimates:

```text
RPM_est =
    motorNoLoadRpm
    x supplyVoltageV / motorNominalVoltageV
    x speedScale
```

This is a feed-forward approximation only.

---

# 6. RPM to angular wheel speed

```text
omega_wheel [rad/s]
    = RPM x 2*pi / 60
```

The library uses:

```text
2*pi = 6.283185...
```

---

# 7. Angular wheel speed to linear wheel-perimeter speed

For wheel radius `r`:

```text
v_wheel = r x omega_wheel
```

Unit:

```text
m/s
```

Therefore the estimated maximum wheel-perimeter speed is:

```text
v_wheel_max
    = wheelRadiusM
    x RPM_est
    x 2*pi / 60
```

The public helper is:

```cpp
robot.maxWheelLinearSpeedMps();
```

---

# 8. Mecanum-X inverse kinematics

Body command:

```text
vx = forward velocity [m/s]
vy = left velocity [m/s]
wz = CCW yaw rate [rad/s]
```

With:

```text
K = (wheelbase + trackWidth) / 2
```

the wheel-perimeter linear velocities are:

```text
v1 = vx - vy - K*wz
v2 = vx + vy - K*wz
v3 = vx - vy + K*wz
v4 = vx + vy + K*wz
```

Wheel order:

```text
v1 -> M1 front-left
v2 -> M2 rear-left
v3 -> M3 rear-right
v4 -> M4 front-right
```

The library exposes this calculation without driving motors:

```cpp
TungLamWheelVelocity wheels =
    robot.inverseKinematics(vxMps, vyMps, wzRadps);
```

---

## 8.1 Check the basis vectors

Pure forward:

```text
vx > 0
vy = 0
wz = 0

=> + + + +
```

Pure left:

```text
vx = 0
vy > 0
wz = 0

=> - + - +
```

Pure CCW rotation:

```text
vx = 0
vy = 0
wz > 0

=> - - + +
```

These are the same physical movement vectors already proven by the original V5 library, but the modern Cartesian signs now follow the standard body frame.

---

# 9. Mecanum-X forward kinematics

If wheel-perimeter velocities are known, for example from wheel encoders:

```text
vx =
    (v1 + v2 + v3 + v4) / 4

vy =
    (-v1 + v2 - v3 + v4) / 4

wz =
    (-v1 - v2 + v3 + v4) / (4*K)
```

Public helper:

```cpp
TungLamBodyVelocity body =
    robot.forwardKinematics(wheels);
```

This function does not read encoders itself.

It provides the mathematical layer that a future encoder driver can feed.

---

# 10. Omni X-drive model

The built-in Omni-X model assumes four 45-degree wheel rolling axes and the same M1..M4 corner positions.

Let:

```text
S = 1/sqrt(2)
K = (wheelbase + trackWidth) / 2
```

Then:

```text
v1 = S * ( vx - vy - K*wz)
v2 = S * ( vx + vy - K*wz)
v3 = S * (-vx - vy - K*wz)
v4 = S * (-vx + vy - K*wz)
```

The corresponding normalized sign basis is:

```text
+vx -> + + - -
+vy -> - + - +
+wz -> - - - -
```

Omni mechanical layouts can vary, so validate real wheel orientation before relying on the canonical model.

---

# 11. Wheel-speed limiting

Suppose inverse kinematics requests:

```text
M1 = 1.20 m/s
M2 = 0.80 m/s
M3 = 1.50 m/s
M4 = 1.10 m/s
```

but the estimated maximum available wheel speed is:

```text
1.00 m/s
```

The library does **not** clip only M3.

Instead it scales the whole vector:

```text
scale = 1.00 / 1.50
```

and multiplies all four wheel speeds by the same factor.

This preserves the requested translation/rotation direction better than independent clipping.

---

# 12. Open-loop wheel velocity to PWM

After limiting:

```text
PWM_i =
    255 x v_i / v_wheel_max
```

with sign retained.

Example:

```text
requested wheel speed = +0.50 m/s
estimated max          = 1.00 m/s

PWM = +128 approximately
```

For:

```text
-0.50 m/s
```

the result is approximately:

```text
-128
```

The sign is passed to the common safe H-bridge HAL.

---

# 13. Why this is still open-loop

Without an encoder, the controller knows:

```text
requested velocity
motor datasheet RPM
battery voltage
wheel radius
chassis geometry
PWM command
```

but it does not know the **actual wheel speed**.

Real speed can differ because of:

```text
load
floor friction
battery sag
motor mismatch
L298N loss
wheel slip
gearbox friction
temperature
```

Therefore:

```cpp
robot.driveVelocity(0.40f, 0.0f, 0.0f);
```

means:

> calculate the PWM that should approximately produce 0.40 m/s according to the configured open-loop model.

It does **not** mean:

> guarantee the measured robot speed is exactly 0.40 m/s.

---

# 14. Future encoder closed-loop velocity control

The architecture is intentionally ready for wheel encoders.

Future data flow:

```text
vx, vy, wz target
      |
inverse kinematics
      |
wheel velocity targets [m/s or rad/s]
      |
+------------------------------+
| wheel PID M1                 |
| wheel PID M2                 |
| wheel PID M3                 |
| wheel PID M4                 |
+------------------------------+
      ^
      |
encoder measured wheel speeds
      |
PWM
      |
motor HAL
```

The kinematics layer does not need to change.

Only the open-loop:

```text
wheel velocity -> PWM
```

stage is replaced or augmented by closed-loop wheel-speed PID.

---

# 15. Future IMU heading hold

The standardized yaw unit is:

```text
wz [rad/s]
```

A heading controller can therefore generate a yaw-rate correction directly.

Concept:

```cpp
float yawErrorRad = targetYawRad - measuredYawRad;

float wzCorrectionRadps =
    headingPid(yawErrorRad);

robot.driveVelocity(
    vxCommandMps,
    vyCommandMps,
    wzManualRadps + wzCorrectionRadps
);
```

Recommended control architecture:

```text
Joystick / planner
      |
  vx [m/s]
  vy [m/s]
  wz manual [rad/s]
      |
      +-------------------+
                          |
IMU yaw -> heading PID -> wz correction [rad/s]
                          |
                          v
                wz final [rad/s]
                          |
                  driveVelocity()
                          |
                inverse kinematics
                          |
                    wheel targets
```

This avoids mixing arbitrary PWM units with physical angular-rate units.

---

# 16. Future ROS2 mapping

The body-frame convention allows a direct conceptual mapping from a ROS-style velocity command:

```text
linear.x  -> vx [m/s]
linear.y  -> vy [m/s]
angular.z -> wz [rad/s]
```

Conceptually:

```cpp
robot.driveVelocity(
    cmdVelLinearX,
    cmdVelLinearY,
    cmdVelAngularZ
);
```

No sign inversion should be required when both systems use the documented right-handed body frame.

---

# 17. Recommended learning path

For a beginner:

```text
PWM commands
-> setWheels()
-> forward/left/right helpers
-> drive(vx, vy, wz) normalized
-> driveVelocity(vx, vy, wz) SI
```

For a student:

```text
coordinate frame
-> chassis geometry
-> inverse kinematics
-> wheel RPM / linear velocity
-> forward kinematics
-> open-loop feed-forward
-> encoder velocity PID
-> IMU heading PID
-> odometry / ROS2
```

That progression lets the same library serve both simple classroom robots and more advanced mobile-robot control experiments.


---

# 19. Control-loop optimization from v0.10

ATmega2560 has no hardware FPU. The library therefore precomputes motor/chassis derived constants when `setDriveConfig()` or `setChassis()` is called.

Cached values include the rotation lever arm, estimated motor RPM, maximum wheel speed, PWM-per-m/s scale, maximum body speed and maximum yaw rate.

This keeps the frequent `driveVelocity()` path focused on additions, multiplications, comparisons and only the saturation division when a requested wheel vector actually exceeds the model.

---

# 20. Smart Safety

```cpp
robot.enableSmartSafety(500, 1.0f, 2.0f);
```

enables a 500 ms command watchdog plus SI velocity slew-rate limiting.

Call:

```cpp
robot.update();
```

continuously in `loop()`.

Velocity smoothing applies to `driveVelocity()`; direct PWM APIs keep immediate behavior for compatibility.

---

# 21. Saturation telemetry

```cpp
robot.wasVelocityLimited();
robot.lastVelocityScale();
robot.requestedBodyVelocity();
robot.appliedBodyVelocity();
```

These helpers let higher-level PID/ROS2 code distinguish requested motion from the velocity vector that the open-loop drive layer could actually apply.
