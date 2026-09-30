# TungLam_OmniMecanum_4WD

Hardware-timer Arduino library for **4-wheel holonomic robots** using an **Arduino Mega 2560**, four brushed DC motors, and **two L298N dual H-bridge drivers**.

It supports both:

- **Mecanum-X 4WD**
- **Omni X-drive 4WD**
- Direct signed control of four independent motors for custom 4-wheel layouts

The V6 API is designed around a reusable low-level motor layer instead of hard-coding the library to one wheel type.

## Architecture

```text
Application / joystick / autonomy
             |
        vx, vy, wz
             |
   +---------+---------+
   |                   |
Mecanum-X mixer    Omni-X mixer
   |                   |
   +---------+---------+
             |
     signed wheel[4]
       -255 .. +255
             |
   safe direction transition
   + hardware PWM + braking
             |
      ATmega2560 timers
      Timer3 + Timer4
             |
          2x L298N
             |
       4 DC motors
```

## Target hardware

- Arduino Mega 2560 / ATmega2560
- 4 brushed DC motors
- 2x L298N modules
- Mecanum-X or four-wheel Omni X-drive chassis

The implementation intentionally uses ATmega2560 registers directly for predictable, low-overhead PWM.

## Features

- Signed per-wheel commands: `-255 ... +255`
- Mecanum-X mixer
- Omni-X mixer
- Hardware PWM on Timer3 + Timer4
- ~7.81 kHz high-frequency PWM mode
- ~976.56 Hz legacy PWM mode
- Per-motor inversion without rewiring
- Proportional normalization when a mixer exceeds PWM range
- Direction-change dead-time before reverse drive
- Coast stop
- L298N dynamic bridge brake
- **Non-blocking active reverse braking** to reduce inertia drift
- Brake duration table compatible with the idea used in legacy V5
- Automatic brake duty based on the previous drive command
- Legacy V5 class kept for backward compatibility

## Installation

### Arduino IDE — Download ZIP

1. Download this repository as ZIP from GitHub.
2. Open Arduino IDE.
3. Select **Sketch → Include Library → Add .ZIP Library...**
4. Select the ZIP file.
5. Open an example from **File → Examples → TungLam_OmniMecanum_4WD**.

### Arduino Library Manager

The project is being prepared for the Arduino Library Registry. After a tested stable release is tagged and accepted by the registry, users will be able to search for:

```text
TungLam_OmniMecanum_4WD
```

directly in Arduino IDE Library Manager.

## Wiring

### PWM / L298N EN pins

| Motor | Arduino Mega pin | AVR output | Timer |
|---|---:|---|---|
| M1 | D5 | PE3 / OC3A | Timer3 |
| M2 | D6 | PH3 / OC4A | Timer4 |
| M3 | D7 | PH4 / OC4B | Timer4 |
| M4 | D8 | PH5 / OC4C | Timer4 |

### Direction pins

| Motor | Arduino Mega direction pair |
|---|---|
| M1 | D30 / D31 |
| M2 | D32 / D33 |
| M3 | D34 / D35 |
| M4 | D36 / D37 |

See [extras/WIRING.md](extras/WIRING.md) for L298N details and wheel-order conventions.

## Quick start — Mecanum

```cpp
#include <TungLam_OmniMecanum_4WD.h>

TungLamDrive4WD robot;

void setup() {
  robot.begin(TungLamPwmMode::High7k8Hz);
  robot.setChassis(TungLamChassis::MecanumX);
}

void loop() {
  robot.update();

  robot.forward(150);
}
```

## Quick start — Omni X-drive

```cpp
#include <TungLam_OmniMecanum_4WD.h>

TungLamDrive4WD robot;

void setup() {
  robot.begin();
  robot.setChassis(TungLamChassis::OmniX);
}

void loop() {
  robot.update();

  // vx forward, vy right, wz clockwise.
  robot.drive(160, 60, 0);
}
```

## Cartesian drive convention

```text
vx > 0 : forward
vx < 0 : backward

vy > 0 : translate right
vy < 0 : translate left

wz > 0 : rotate clockwise / right
wz < 0 : rotate counter-clockwise / left
```

All three inputs use the logical range `-255 ... +255`. If the mixed wheel values exceed 255, the library scales all four proportionally so the requested motion direction is preserved.

## Direct per-wheel control

For custom chassis geometry or an external kinematics layer:

```cpp
robot.setWheels(+180, -120, +180, -120);
```

This is the lowest public control layer and is independent of Mecanum/Omni mixing.

## Active reverse braking

The original V5 `ABS()` idea is retained, but V6 implements it as a **non-blocking active reverse brake**.

```text
DRIVE
  |
  | activeBrake()
  v
REVERSE BRAKE PULSE
  |
  | update(), timer expires
  v
COAST / STOP
```

Example:

```cpp
robot.activeBrake();       // returns immediately

void loop() {
  robot.update();          // services the brake state machine
}
```

You can also use the compatibility-friendly alias:

```cpp
robot.ABS();
```

The automatic brake duty is limited and derived from the previous wheel command. The reverse pulse duration is selected from a configurable motion-time table.

> Active reverse braking can generate high current. Tune brake duty and timing conservatively for your motor, gearbox, battery and L298N temperature.

## Dynamic brake vs active reverse brake

**Coast**

```cpp
robot.stop();
```

PWM is disabled and the motor is allowed to coast.

**L298N dynamic brake**

```cpp
robot.dynamicBrake();
```

Both H-bridge inputs are driven to the same state while EN is active. This electrically damps the motor without intentionally driving it in the opposite direction.

**Active reverse brake**

```cpp
robot.activeBrake();
```

The library briefly commands the opposite wheel direction to counter inertia, then automatically stops. This is stronger and must be tuned with more care.

## Direction reversal protection

When a wheel changes directly from positive to negative command, V6:

```text
PWM = 0
  |
short dead-time (default 100 us)
  |
change DIR
  |
apply new PWM
```

The dead-time can be tuned:

```cpp
robot.setDirectionDeadTimeUs(150);
```

## Motor inversion

If a motor is physically installed in the opposite orientation:

```cpp
robot.setMotorInverted(3, true);
```

This avoids scattering sign changes through application code.

## PWM modes

```cpp
robot.begin(TungLamPwmMode::High7k8Hz); // default
robot.begin(TungLamPwmMode::Low976Hz);
```

For most L298N + DC motor setups, start with `High7k8Hz`.

## Legacy V5

The original class remains available:

```cpp
#include <TungLam_Control_MotorV5.h>

TungLam_Control_MotorV5 legacyRobot;
```

V5 is preserved so existing sketches can migrate incrementally. New projects should use `TungLamDrive4WD`.

## Repository layout

```text
TungLam-OmniMecanum-4WD/
├─ src/
│  ├─ TungLam_OmniMecanum_4WD.h
│  ├─ TungLam_OmniMecanum_4WD.cpp
│  ├─ TungLam_Control_MotorV5.h
│  └─ TungLam_Control_MotorV5.cpp
├─ examples/
│  ├─ MecanumDrive/
│  ├─ OmniXDrive/
│  ├─ PerWheelControl/
│  ├─ ActiveBrakeNonBlocking/
│  ├─ BasicMotion/
│  └─ ActiveBrake/
├─ extras/
│  └─ WIRING.md
├─ library.properties
├─ keywords.txt
├─ CHANGELOG.md
├─ LICENSE
└─ README.md
```

## Version status

- **0.5.x** — packaged legacy V5 baseline
- **0.6.0** — V6 holonomic core development
- **1.0.0** — reserved for hardware-validated stable release

## License

MIT License.

## Author

**Nguyen Khac Tung Lam — Tung Lam Automation**
