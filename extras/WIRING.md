# Wiring — Arduino Mega + 2x L298N

This library targets the Arduino Mega / ATmega2560 register and timer layout.

## PWM / Enable pins

| Wheel | Arduino Mega | AVR output | Timer channel |
|---|---:|---|---|
| Motor 1 EN | 5 | PE3 | OC3A |
| Motor 2 EN | 6 | PH3 | OC4A |
| Motor 3 EN | 7 | PH4 | OC4B |
| Motor 4 EN | 8 | PH5 | OC4C |

## Direction pins

| Wheel | Forward/Reverse pair |
|---|---|
| Motor 1 | D30 / D31 |
| Motor 2 | D32 / D33 |
| Motor 3 | D34 / D35 |
| Motor 4 | D36 / D37 |

A common arrangement is one L298N for motors 1–2 and one L298N for motors 3–4.

## Power notes

- Use a common ground between the Arduino and both L298N modules.
- Power motors from an appropriate external supply; do not power four motors from the Arduino 5 V rail.
- Verify motor polarity before high-duty testing.
- Active reverse braking can produce high current. Start with conservative duty and brake duration.
