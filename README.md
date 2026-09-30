# TungLam_Mecanum_L298N

Arduino Mega library for controlling a four-wheel mecanum robot with **two L298N drivers** using direct AVR timer/register access.

The repository is structured as a standard Arduino library so it can be installed from a ZIP file and is being prepared for publication in the Arduino Library Manager.

## Target hardware

- Arduino Mega 2560 / ATmega2560
- 4 brushed DC motors
- 2x L298N dual H-bridge drivers
- Mecanum / omnidirectional four-wheel chassis

## Features

- Direct-register PWM for low overhead
- Timer3 + Timer4 control of PWM pins 5, 6, 7, 8
- 976.56 Hz and 7.81 kHz PWM modes
- Forward / reverse / rotation / strafe / diagonal helpers
- Independent PWM for all four wheels
- Legacy active reverse braking (`ABS`) to reduce coast caused by motor inertia
- Example sketches included

## Pin map

### PWM

| Motor | Arduino pin | Timer output |
|---|---:|---|
| M1 | 5 | OC3A |
| M2 | 6 | OC4A |
| M3 | 7 | OC4B |
| M4 | 8 | OC4C |

### Direction

| Motor | Direction pins |
|---|---|
| M1 | 30, 31 |
| M2 | 32, 33 |
| M3 | 34, 35 |
| M4 | 36, 37 |

See [extras/WIRING.md](extras/WIRING.md) for details.

## Installation

### Arduino IDE — ZIP

1. Open this repository on GitHub.
2. Choose **Code → Download ZIP**.
3. In Arduino IDE, choose **Sketch → Include Library → Add .ZIP Library…**.
4. Select the downloaded ZIP.
5. Open **File → Examples → TungLam_Mecanum_L298N**.

### Arduino Library Manager

The repository is prepared for Arduino Library Manager registration. After the first stable release is validated and submitted to the Arduino Library Registry, users will be able to search for **TungLam_Mecanum_L298N** directly in Library Manager.

## Quick start

```cpp
#include <TungLam_Mecanum_L298N.h>

TungLamMecanumL298N robot;

void setup() {
  robot.Mode1();   // ~7.81 kHz PWM
  robot.STOP();
}

void loop() {
  robot.moveForward(150);
  delay(1000);

  robot.STOP();
  delay(500);

  robot.moveRightSide(150);
  delay(1000);

  robot.STOP();
  delay(1000);
}
```

## PWM modes

- `Mode0()`: approximately 976.56 Hz
- `Mode1()`: approximately 7.81 kHz

For typical L298N-driven DC motors, `Mode1()` is generally the preferred starting point because it reduces audible PWM noise while preserving 8-bit duty control.

## Active reverse braking

The legacy `ABS()` function is an **active reverse-braking helper**, not automotive anti-lock braking. It briefly commands the opposite motion to reduce coast caused by inertia.

The current V5 implementation is preserved as the baseline. A future V6 is planned to make braking non-blocking, add safer direction transitions, and tune braking based on prior PWM as well as travel time.

> Active reverse braking can create high current in the motor and L298N. Test with conservative brake duty/time first.

## Repository layout

```text
TungLam-Mecanum-L298N/
├─ src/
│  ├─ TungLam_Mecanum_L298N.h
│  ├─ TungLam_Control_MotorV5.h
│  └─ TungLam_Control_MotorV5.cpp
├─ examples/
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

## Roadmap

The modernization path is intentionally incremental:

1. Preserve and package V5 as a reproducible baseline.
2. Fix direction/diagonal and braking-state issues.
3. Replace blocking braking with a non-blocking state machine.
4. Introduce signed per-wheel commands and safer direction transitions.
5. Add mecanum `vx / vy / wz` mixing without coupling the low-level motor driver to higher-level control.

## License

MIT License.

## Author

**Nguyen Khac Tung Lam — Tung Lam Automation**
