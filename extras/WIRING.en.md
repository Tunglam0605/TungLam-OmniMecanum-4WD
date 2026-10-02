[🇻🇳 Tiếng Việt](WIRING.md) • [🌐 English](WIRING.en.md)

# Wiring & commissioning guide

This document is the detailed electrical companion to the main README.

> **Target:** Arduino Mega 2560 + 2× L298N + 4 brushed DC motors.

---

## 1. Safety first

Before powering the robot:

- lift the chassis so the wheels can rotate freely;
- start with low PWM such as 60–80;
- verify common ground;
- verify motor-supply polarity;
- remove ENA/ENB jumpers when EN is driven by Arduino PWM;
- keep motor current out of the Arduino 5 V rail;
- stop immediately if an L298N becomes excessively hot.

The library controls logic only. Motor current, battery sizing, wiring gauge, mechanical load, and L298N thermal limits remain hardware responsibilities.

---

## 2. Logical wheel IDs

Top view:

```text
                     FRONT / ĐẦU XE
                           +vx
                            ↑

              M1                         M3
         FRONT-LEFT                 FRONT-RIGHT
            PWM D5                     PWM D7

              M2                         M4
          REAR-LEFT                  REAR-RIGHT
            PWM D6                     PWM D8

                            ↓
                      REAR / ĐUÔI XE
```

Keep these IDs consistent in wiring, code, and troubleshooting.

---

## 3. Recommended L298N allocation

### L298N #1

| Channel | Motor | EN/PWM | DIR input 1 | DIR input 2 |
|---|---|---:|---:|---:|
| A | M1 | D5 | D30 | D31 |
| B | M2 | D6 | D32 | D33 |

### L298N #2

| Channel | Motor | EN/PWM | DIR input 1 | DIR input 2 |
|---|---|---:|---:|---:|
| A | M3 | D7 | D34 | D35 |
| B | M4 | D8 | D37 | D36 |

For M4, the logical forward pin is **D37 / PC0** and reverse is **D36 / PC1**.

---

## 4. Complete Arduino pin map

### PWM / EN

| Motor | Mega pin | AVR output | Timer |
|---|---:|---|---|
| M1 | D5 | PE3 / OC3A | Timer3 |
| M2 | D6 | PH3 / OC4A | Timer4 |
| M3 | D7 | PH4 / OC4B | Timer4 |
| M4 | D8 | PH5 / OC4C | Timer4 |

### Direction

| Motor | Logical forward | Logical reverse |
|---|---:|---:|
| M1 | D30 / PC7 | D31 / PC6 |
| M2 | D32 / PC5 | D33 / PC4 |
| M3 | D34 / PC3 | D35 / PC2 |
| M4 | D37 / PC0 | D36 / PC1 |

The library owns PORTC D30..D37 for these four direction pairs.

---

## 5. Power and ground

Recommended topology:

```text
Motor battery (+)
      ├──────────────> L298N #1 motor supply
      └──────────────> L298N #2 motor supply

Motor battery (-)
      ├──────────────> L298N #1 GND
      ├──────────────> L298N #2 GND
      └──────────────> Arduino Mega GND
```

### Important notes

- All control electronics need a common reference ground.
- Do not power four drive motors from the Arduino 5 V pin.
- L298N modules differ in how the onboard 5 V regulator and 5V-EN jumper are wired.
- Do not blindly connect module 5 V to Arduino 5 V unless you understand the exact module revision and supply arrangement.
- Use a motor supply appropriate for your motors.

---

## 6. ENA / ENB jumpers

Many L298N modules ship with jumpers that force ENA and ENB high.

If those jumpers remain installed, the motor channel can appear to run only at full speed and Arduino PWM will not control the enable line correctly.

For this library:

```text
L298N #1 ENA <- D5
L298N #1 ENB <- D6

L298N #2 ENA <- D7
L298N #2 ENB <- D8
```

Remove the fixed EN jumpers before connecting these PWM pins.

---

## 7. First commissioning procedure

Use:

```text
File
→ Examples
→ TungLam_OmniMecanum_4WD
→ FirstMotorTest
```

The test sequence is:

```text
M1 forward
M1 reverse

M2 forward
M2 reverse

M3 forward
M3 reverse

M4 forward
M4 reverse
```

Expected physical positions:

| Motor | Position |
|---|---|
| M1 | Front-left |
| M2 | Rear-left |
| M3 | Front-right |
| M4 | Rear-right |

If one motor is reversed:

```cpp
robot.setMotorInverted(1, true);
```

Change only the affected wheel number.

---

## 8. Mecanum convention

The modern Mecanum mixer follows the physical X-layout used by this project. Legacy V5 keeps its historical vectors separately.

```text
Forward       + + + +
Backward      - - - -
Right         + - - +
Left          - + + -
Rotate right  + + - -
Rotate left   - - + +
```

Diagonals:

```text
Forward-right   + 0 0 +
Forward-left    0 + + 0
Backward-right  0 - - 0
Backward-left   - 0 0 -
```

Cartesian convention:

```text
+vx = forward
+vy = left
+wz = counter-clockwise
```

If a newly built chassis does not strafe as expected after motor polarity is correct, inspect the **mechanical Mecanum wheel orientation** before changing software equations.

---

## 9. Omni X-drive convention

For Omni X-drive:

```cpp
robot.setChassis(TungLamChassis::OmniX);
```

The software convention is still:

```text
+vx = forward
+vy = left
+wz = counter-clockwise
```

Omni mechanical layouts vary. Commission each wheel individually, then test pure vx, pure vy, and pure wz before combined motion.

---

## 10. Direction-change protection

Both modern and legacy movement commands use the common motor HAL.

When the electrical direction state changes:

```text
PWM = 0
   ↓
dead-time
   ↓
DIR update
   ↓
new PWM
```

Default dead-time:

```text
100 µs
```

Modern API adjustment:

```cpp
robot.setDirectionDeadTimeUs(150);
```

---

## 11. ABS / active reverse braking

Active reverse braking intentionally applies torque opposite to the current wheel direction.

```cpp
robot.ABS(180);
```

The requested value is the actual brake PWM.

Default timing table:

| Previous motion time | Brake pulse |
|---:|---:|
| < 500 ms | 45 ms |
| < 1000 ms | 65 ms |
| < 1500 ms | 70 ms |
| < 2000 ms | 75 ms |
| < 3000 ms | 80 ms |
| >= 3000 ms | 85 ms |

The Timer3 overflow interrupt acts as a one-shot cutoff, so the reverse pulse does not depend on normal loop timing.

### ABS risks

Strong active braking can cause:

- high motor current;
- L298N heating;
- battery voltage sag;
- mechanical shock;
- wheel slip.

Tune duty and timing conservatively.

---

## 12. Timer ownership

The drive library owns:

```text
Timer3  -> M1 PWM + ABS overflow timing
Timer4  -> M2 / M3 / M4 PWM
PORTC   -> D30..D37 direction outputs
```

Other libraries that reconfigure Timer3 or Timer4 may conflict.

The legacy V5 API also retains its historical auxiliary Timer1/Timer2 initialization functions. Use them only if your application understands those timer ownership implications.

---

## 13. Controller ownership

One Arduino Mega should use one drive-controller API path for the same motor hardware:

```text
Legacy:
TungLam_Control_MotorV5

OR

Modern:
TungLamDrive4WD
```

Do not instantiate both to control the same D5..D8 / D30..D37 outputs simultaneously.

---

## 14. Troubleshooting checklist

If the robot behaves incorrectly, check in this order:

1. common ground;
2. EN jumpers removed;
3. correct motor ID M1..M4;
4. correct PWM pins D5..D8;
5. correct DIR pins D30..D37;
6. one-wheel polarity using FirstMotorTest;
7. Mecanum/Omni mechanical wheel orientation;
8. battery voltage under load;
9. L298N temperature;
10. only then inspect software settings.

This order avoids masking hardware mistakes with software sign changes.
