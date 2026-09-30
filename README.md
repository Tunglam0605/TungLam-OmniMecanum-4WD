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
- User-controlled active reverse-brake strength via `ABS(duty)`
- Legacy V5 class kept for backward compatibility

## Drop-in compatibility with the original V5 API

Existing sketches do **not** need their motor-control function calls rewritten.

The original include still works after installing this new library:

```cpp
#include <TungLam_Control_MotorV5.h>

TungLam_Control_MotorV5 robot;

void setup() {
  robot.Mode1();
  robot.setTimABS(45, 65, 70, 75, 80, 85);
}

void loop() {
  robot.moveForward(160);
  robot.ABS(200);
}
```

You can also switch only the include to the new umbrella header and keep the same class and functions:

```cpp
#include <TungLam_OmniMecanum_4WD.h>

TungLam_Control_MotorV5 robot;
```

The following legacy public API is preserved:

```text
Mode0 / Mode1
Init_Timer1 / Init_Timer2
STOP
moveForward / moveBackward
Forward_Right / Backward_Right
Forward_Left / Backward_Left
moveRight / moveLeft
moveLeftSide / moveRightSide
Dir
Tien / Lui
Trai / Phai
T_Trai / T_Phai
L_Trai / L_Phai
N_Trai / N_Phai
ABS(duty)
setTimABS(...)
```

### Non-blocking ABS without changing old sketches

The legacy class now uses the Timer3 PWM overflow interrupt as a short one-shot scheduler during `ABS()`.

That means old code does **not** need to add:

```cpp
robot.update();
```

The call:

```cpp
robot.ABS(200);
```

starts the same strong reverse-brake pulse, returns immediately, and the interrupt automatically sets PWM to zero when the configured `setTimABS()` interval expires.

A new drive command issued before ABS expires cancels the pending brake pulse and takes control immediately.

> Migration note: if the old manually installed V5 library is still present as a separate Arduino library folder, remove that old copy once to avoid Arduino reporting multiple libraries for `TungLam_Control_MotorV5.h`. After that, existing sketch source can remain unchanged.

## Installation

### Arduino IDE — Download ZIP

1. Download this repository as ZIP from GitHub.
2. Open Arduino IDE.
3. Select **Sketch → Include Library → Add .ZIP Library...**
4. Select the ZIP file.
5. Open an example from **File → Examples → TungLam_OmniMecanum_4WD**.

### Arduino Library Manager

The library is registered in the official Arduino Library Registry. Search for:

```text
TungLam_OmniMecanum_4WD
```

in Arduino IDE Library Manager. Once installed from Library Manager, future tagged releases can be updated directly from Arduino IDE.

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

## Active reverse braking — modern API

`TungLamDrive4WD` preserves the original V5 braking concept while using the modern `update()`-serviced state machine:

```cpp
robot.ABS(duty);
```

The `duty` argument is the **actual reverse-brake PWM strength selected by the user (0..255)**. V6 does not automatically reduce or cap it against the previous driving PWM.

For example:

```cpp
robot.ABS(180);   // strong reverse braking
```

The original time table is also preserved. Default values are:

| Previous motion duration | Reverse-brake time |
|---:|---:|
| < 500 ms | 45 ms |
| < 1000 ms | 65 ms |
| < 1500 ms | 70 ms |
| < 2000 ms | 75 ms |
| < 3000 ms | 80 ms |
| >= 3000 ms | 85 ms |

The legacy configuration function is kept with the same name:

```cpp
robot.setTimABS(45, 65, 70, 75, 80, 85);
```

The modern `TungLamDrive4WD` implementation is **non-blocking** and is serviced by `update()`. The legacy `TungLam_Control_MotorV5` compatibility class is also non-blocking, but it uses Timer3 overflow interrupt internally and therefore does **not** require `update()`.

Legacy V5 concept:

```text
reverse(duty)
delay(TIM)
STOP
```

V6 concept:

```text
ABS(duty)
   |
   +--> reverse torque starts immediately
   |
   +--> function returns immediately
              |
         robot.update()
              |
        TIM expires
              |
            STOP
```

So `ABS(180)` still applies the requested reverse torque for the selected legacy time interval, but Arduino can continue handling Serial, sensors, joystick commands and other application logic while braking.

```cpp
void loop() {
  robot.update();  // services the non-blocking ABS timer

  // Other application work can continue here.
}
```

`activeBrake(duty)` is provided as a descriptive alias of `ABS(duty)`.

> Active reverse braking can generate high current. Tune `duty` and `setTimABS()` for the actual motor, gearbox, robot mass, battery and L298N temperature.

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
robot.ABS(180);
```

The library briefly commands the opposite wheel direction at the exact user-selected `duty`, then automatically stops when the configured legacy ABS time expires.

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

The source tree is intentionally compact:

```text
TungLam-OmniMecanum-4WD/
├─ src/
│  ├─ TungLam_OmniMecanum_4WD.h      # Main public header: modern + legacy API
│  ├─ TungLam_OmniMecanum_4WD.cpp    # All implementation
│  └─ TungLam_Control_MotorV5.h      # Tiny include shim for old sketches
├─ examples/
├─ extras/
├─ library.properties
├─ keywords.txt
├─ CHANGELOG.md
├─ LICENSE
└─ README.md
```

Old sketches can continue including `TungLam_Control_MotorV5.h`; that compatibility header simply forwards to the main library header. There is no duplicated V5 implementation file anymore.

## Version status

- **0.5.x** — packaged legacy V5 baseline
- **0.6.x** — V6 holonomic core and ABS compatibility refinement
- **0.7.0** — drop-in V5 API compatibility with interrupt-driven non-blocking ABS
- **0.7.1** — compact 3-file source layout with unchanged V5/V6 public APIs
- **1.0.0** — reserved for hardware-validated stable release

## License

MIT License.

## Author

**Nguyen Khac Tung Lam — Tung Lam Automation**
