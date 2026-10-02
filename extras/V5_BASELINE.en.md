# V5 Baseline — hardware-proven reference

This document is the **ground truth** for the motor mapping and Mecanum command vectors proven by the original `TungLam_Control_MotorV5` on the real robot.

## 1. Motor-channel mapping

```text
                         FRONT
                           ↑

                M1                     M4
           FRONT-LEFT              FRONT-RIGHT
             PWM D5                  PWM D8

                M2                     M3
            REAR-LEFT               REAR-RIGHT
             PWM D6                  PWM D7

                           ↓
                          REAR
```

| Channel | Physical position | PWM |
|---|---|---:|
| M1 | front-left | D5 |
| M2 | rear-left | D6 |
| M3 | rear-right | D7 |
| M4 | front-right | D8 |

Code-vector order is always `[M1, M2, M3, M4]`.

Clockwise physical order from the front-left corner is `M1 -> M4 -> M3 -> M2`.

These two orders must not be confused.

## 2. Direction pins and V5

| Motor | V5 forward branch / Set=false | opposite branch |
|---|---:|---:|
| M1 | D30 / PC7 | D31 / PC6 |
| M2 | D32 / PC5 | D33 / PC4 |
| M3 | D34 / PC3 | D35 / PC2 |
| M4 | D37 / PC0 | D36 / PC1 |

Original V5 defines `bool Set = false;` and `moveForward()` calls `Dir(1..4, Set)`.

## 3. Hardware-proven V5 command vectors

The signs below are **channel command polarities**, not a direct visual description of mechanical shaft rotation.

| Motion | M1 | M2 | M3 | M4 |
|---|---:|---:|---:|---:|
| forward | + | + | + | + |
| backward | - | - | - | - |
| strafe left | - | + | - | + |
| strafe right | + | - | + | - |
| rotate left / CCW | - | - | + | + |
| rotate right / CW | + | + | - | - |
| forward-right | + | 0 | + | 0 |
| forward-left | 0 | + | 0 | + |
| backward-right | 0 | - | 0 | - |
| backward-left | - | 0 | - | 0 |

## 4. Modern Mecanum must preserve V5 parity

```text
+vx = forward
+vy = left
+wz = rotate left / CCW

M1 = vx - vy - wz
M2 = vx + vy - wz
M3 = vx - vy + wz
M4 = vx + vy + wz
```

SI kinematics applies the lever arm `K` to the yaw term.

## 5. Historical PS2 control style

Projects such as `Mecanum_STEP_Roboball`, `RoboBall_STEP`, and `Thumon_Meganum` commonly use:

```text
left stick:
  UP/DOWN    -> forward/backward
  LEFT/RIGHT -> strafe left/right

right stick:
  LEFT/RIGHT -> rotate left/right
```

In the older Mecanum projects, right-stick commands are typically evaluated after left-stick commands, so rotation effectively overrides translation.

The modern `PS2RobotControl` example makes this priority explicit instead of depending on statement order.

## 6. Regression rule

Before changing Mecanum mapping or mixing:

1. Check this baseline first.
2. Do not re-derive M1..M4 from a textbook diagram.
3. Preserve the hardware-proven V5 channel mapping.
4. Keep compile-time basis assertions.
5. Run Legacy RoboBall Regression.
6. Change this baseline only after real hardware validation.
