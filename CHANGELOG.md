# Changelog

All notable changes to this project will be documented here.

## [Unreleased]

### Validation
- Hardware validation on Arduino Mega + two L298N boards is still required before the 1.0.0 stable release.

## [0.7.1] - 2026-09-30

### Changed
- Reduced `src/` from five files to three without changing the V5 or modern public APIs.
- Merged the complete legacy V5 implementation into `TungLam_OmniMecanum_4WD.cpp`.
- Moved the `TungLam_Control_MotorV5` class declaration into the main public header.
- Kept `TungLam_Control_MotorV5.h` as a tiny compatibility include for unchanged old sketches.
- Removed the transitional `TungLam_Mecanum_L298N.h` packaging header from the unreleased 0.5.x naming phase.
- Updated legacy examples to use the original V5 include directly.

### Compatibility
- Existing V5 projects using `#include <TungLam_Control_MotorV5.h>` and the original class/method names remain source-compatible.
- Modern projects continue using `#include <TungLam_OmniMecanum_4WD.h>`.
- No new runtime dependency or update call is required for legacy ABS.

## [0.7.0] - 2026-09-30

### Added
- Drop-in source compatibility for sketches built against `TungLam_Control_MotorV5`.
- Timer3 overflow one-shot scheduler for legacy `ABS(duty)`, so old sketches do not need to call `update()`.
- Legacy compatibility examples using both the original header and the new umbrella header.
- ABS state support for the legacy four-duty movement API (`Tien/Lui/Trai/Phai/T_Trai/T_Phai/L_Trai/L_Phai/N_Trai/N_Phai`).

### Changed
- Legacy `ABS(duty)` now returns immediately while preserving the user-selected brake PWM and `setTimABS(...)` timing table.
- Any new legacy motor direction command safely cancels an active asynchronous ABS pulse before taking control.
- Arduino Mega CI now compiles every example folder automatically.

### Compatibility
- All original public V5 method names and signatures remain available.
- Existing sketches can continue including `TungLam_Control_MotorV5.h` unchanged.
- The new `TungLam_OmniMecanum_4WD.h` umbrella header also exposes the complete V5 class.

## [0.6.1] - 2026-09-30

### Changed
- Restored legacy `ABS(duty)` semantics in the V6 API: the caller directly controls reverse-brake PWM from 0..255.
- Restored the legacy `setTimABS(...)` function name and the original six-stage time table.
- `ABS(duty)` is now non-blocking: it starts the reverse pulse immediately and `update()` stops it when the configured time expires.
- Removed automatic brake-duty limiting from the primary ABS path so a heavy robot can be tuned for strong reverse braking.
- Kept `activeBrake(duty)` and `setBrakeTimings(...)` as descriptive aliases.

### Compatibility
- Default ABS times remain 45/65/70/75/80/85 ms.
- Existing V5 sketches retain their original `ABS(duty)` and `setTimABS(...)` control model.

## [0.6.0] - 2026-09-30

### Added
- New `TungLamDrive4WD` V6 API
- Shared four-motor signed output layer (`-255..255`)
- Mecanum-X kinematics mixer
- Omni X-drive kinematics mixer
- Proportional wheel normalization
- Per-motor inversion configuration
- Configurable direction-change dead-time
- Coast stop and L298N dynamic braking
- Non-blocking active reverse braking state machine
- `ABS()` alias for the active reverse-brake concept
- New Mecanum, Omni-X, per-wheel and non-blocking brake examples

### Changed
- Project scope expanded from Mecanum-only to four-wheel holonomic Mecanum/Omni robots.
- Public library name changed to `TungLam_OmniMecanum_4WD`.
- Repository renamed to `TungLam-OmniMecanum-4WD`.

### Compatibility
- Legacy `TungLam_Control_MotorV5` remains available.

## [0.5.0] - 2026-09-30

### Added
- Initial GitHub packaging of the legacy V5 library
- Arduino Library Specification layout
- Library metadata, examples, wiring documentation, and lint CI
- Original V5 source preserved as the baseline for modernization
