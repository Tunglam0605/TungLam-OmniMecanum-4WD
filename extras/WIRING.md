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

| Motor | Forward DIR | Reverse DIR |
|---|---:|---:|
| M1 | D30 / PC7 | D31 / PC6 |
| M2 | D32 / PC5 | D33 / PC4 |
| M3 | D34 / PC3 | D35 / PC2 |
| M4 | D37 / PC0 | D36 / PC1 |

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

The modern API uses this logical wheel order:

```text
       FRONT

   M1 -------- M3
   FL          FR

   M2 -------- M4
   RL          RR

        REAR
```

The Mecanum mixer is intentionally aligned with the original TungLam V5 movement basis: forward `++++`, strafe-right `+-+-`, rotate-right `++--`. This preserves legacy robot behavior while enabling arbitrary `vx/vy/wz` mixing. Validate physical roller orientation and motor polarity at low PWM on any newly built chassis.

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

The V6 `ABS(duty)` path keeps the legacy behavior: the user selects the reverse-brake PWM directly. Use `setTimABS()` to tune the six legacy brake-time ranges. V6 only changes the timing implementation to non-blocking; it does not automatically weaken the requested brake duty.


## Timer ownership and hardware-timed ABS

The library owns Timer3 and Timer4 for the four motor PWM channels.

For both the drop-in `TungLam_Control_MotorV5` API and modern `TungLamDrive4WD`, Timer3's overflow interrupt is enabled only while an `ABS(duty)` pulse is active. It is used as a one-shot timing source so the reverse pulse is cut off independently of sketch loop latency.

The interrupt is disabled automatically when:
- the ABS interval expires,
- `STOP()` is called,
- a new direction/motion command takes control,
- or `Mode0()/Mode1()` reinitializes the motor timers.

As with the original direct-register library, other libraries that also take ownership of Timer3/Timer4 are not compatible with the four-motor PWM configuration.


## Controller ownership

D5..D8, D30..D37, Timer3 and Timer4 form one physical drive-motor peripheral. Use one motor-controller API on a board at a time: either the legacy `TungLam_Control_MotorV5` path or the modern `TungLamDrive4WD` path. Mixing both controller objects against the same hardware is unsupported.
