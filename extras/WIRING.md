# Wiring and chassis conventions

## Electrical target

- Arduino Mega 2560
- Two L298N dual H-bridge modules
- Four brushed DC motors
- Common ground between Arduino, both L298N boards and motor supply

Do not power four motors from the Arduino 5 V rail.

## PWM / EN mapping

| Motor | Arduino | AVR | L298N role |
|---|---:|---|---|
| M1 | D5 | PE3 / OC3A | ENA/ENB for selected channel |
| M2 | D6 | PH3 / OC4A | ENA/ENB for selected channel |
| M3 | D7 | PH4 / OC4B | ENA/ENB for selected channel |
| M4 | D8 | PH5 / OC4C | ENA/ENB for selected channel |

Remove the fixed EN jumper on L298N modules when the EN pin is driven by PWM from the Arduino.

## Direction mapping

| Motor | DIR A | DIR B |
|---|---:|---:|
| M1 | D30 | D31 |
| M2 | D32 | D33 |
| M3 | D34 | D35 |
| M4 | D36 | D37 |

The V6 driver owns PORTC (D30..D37) as the four motor direction pairs.

## Recommended L298N allocation

```text
L298N #1
  Channel A -> Motor 1
  Channel B -> Motor 2

L298N #2
  Channel A -> Motor 3
  Channel B -> Motor 4
```

The exact physical left/right assignment can be changed with `setMotorInverted()`, but keep the software wheel numbering consistent.

## Mecanum-X convention

V6 uses this canonical logical order:

```text
       FRONT

   M1 -------- M3
   FL          FR

   M2 -------- M4
   RL          RR

        REAR
```

The mixer assumes a standard Mecanum-X roller arrangement.

Cartesian convention:

```text
+vx = forward
+vy = right
+wz = clockwise
```

If the robot moves in the wrong direction because motor polarity differs from the convention, correct each wheel with:

```cpp
robot.setMotorInverted(wheelNumber, true);
```

rather than editing the mixer equations.

## Omni X-drive convention

For Omni X-drive, the four wheel rolling axes must be arranged as an X-drive geometry. The V6 `driveOmniX()` mixer assumes the wheel order is consistent around the chassis and uses the same Cartesian convention:

```text
+vx = forward
+vy = right
+wz = clockwise
```

Because Omni chassis mechanical layouts vary more than Mecanum kits, validate each axis at a low PWM first.

Recommended commissioning sequence:

1. Lift the chassis so wheels are free.
2. Run `setWheels(80, 0, 0, 0)` and verify M1 polarity.
3. Repeat for M2, M3 and M4.
4. Correct polarity with `setMotorInverted()`.
5. Test pure `vx`.
6. Test pure `vy`.
7. Test pure `wz`.
8. Only then test combined motion.

## Braking safety

Active reverse braking intentionally drives opposite torque for a short time. It can cause:

- high motor current
- L298N heating
- battery voltage sag
- gearbox shock
- wheel slip

Start with conservative values and monitor temperature.

The V6 automatic brake uses the previous wheel duty as an upper bound for each wheel and limits the global brake duty with configurable min/max settings.
