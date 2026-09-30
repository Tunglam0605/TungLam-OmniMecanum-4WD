# Changelog

All notable changes to this project will be documented here.

## [Unreleased]

### Validation
- Hardware validation on Arduino Mega + two L298N boards is still required before the 1.0.0 stable release.

## [0.8.1] - 2026-09-30

### Cleanup
- Removed private legacy helpers made obsolete by the shared Motor HAL: `Reset_45`, `setSTOP`, `setPWM` and `PWM`.
- Removed unused modern wrapper helpers around the common HAL while retaining the single PWM initialization path.
- Kept the complete 29-method legacy V5 public API unchanged.
- Pinned RoboBall 2024 and PS2X regression dependencies to immutable commit SHAs.
- Clarified that one Arduino Mega should use one motor-controller API path at a time.
- Cleaned README formatting and clarified that modern `update()` is optional state synchronization, not a brake-safety requirement.

### Validation intent
- No intended movement-vector, ABS timing, pin mapping, PWM mode or public API behavior change.

## [0.8.0] - 2026-09-30

### Changed
- Unified legacy V5 movement commands and the modern API on one internal signed-wheel Motor HAL.
- Every normal direction-state transition now follows `PWM=0 -> dead-time -> DIR -> PWM`, not only ABS.
- Changed the modern Mecanum mixer to the proven V5 movement basis: forward `++++`, right `+-+-`, clockwise `++--`.
- Modern and legacy active reverse braking now share the same Timer3 overflow one-shot cutoff.
- Modern ABS no longer depends on `update()` or loop timing to remove reverse torque.
- Kept `update()` as an optional software-state synchronization function for backward source compatibility.
- Legacy per-wheel movement functions retain their original signatures and ABS state codes while using the common safe HAL.

### Fixed
- Eliminated live-PWM direction changes from normal legacy movement commands.
- Removed the architecture split where legacy commands wrote registers directly while modern commands used the safe wheel-output path.
- Ensured a new command can safely preempt an active timed brake through the shared scheduler/HAL.

### Compatibility
- Existing `TungLam_Control_MotorV5` sketches keep the same include, class name and public function calls.
- `ABS(duty)` still uses the exact caller-selected reverse PWM and the six-stage `setTimABS(...)` timing table.
- Modern source code using `update()` continues to compile; the call is simply no longer required for physical brake cutoff.

## [0.7.3] - 2026-09-30

### Documentation
- Restored the original V5 author identity block in the public headers: Nguyen Khac Tung Lam, class DHTD16A2CL and the original student identifier.
- Restored the familiar V5 quick wiring reference and corrected it against the actual AVR implementation so users opening the header immediately see accurate PWM and DIR connections.
- Added the modern logical wheel-order diagram (M1 front-left, M2 rear-left, M3 front-right, M4 rear-right) and vx/vy/wz direction convention.
- Added quick class examples for both legacy and modern APIs directly at the top of the main header.
- Added an ABS quick-reference block describing legacy vs modern non-blocking behavior.
- Corrected DIR4 documentation to D37/PC0 for forward and D36/PC1 for reverse, matching the implementation.
- Expanded the legacy compatibility header so old users still see author and wiring information without opening another file.

### Compatibility
- Documentation/header-comment change only; class names, APIs and executable behavior remain unchanged.

## [0.7.2] - 2026-09-30

### Documentation
- Reworked all three `src/` files into documentation-grade source code without changing executable behavior.
- Added comprehensive Doxygen documentation for the complete modern and legacy public APIs.
- Documented Arduino Mega pin mapping, Timer/PORT ownership, PWM modes, motor inversion, Mecanum/Omni mixers, normalization, direction dead-time and all brake modes.
- Documented the legacy V5 movement-state codes and per-wheel movement helpers.
- Added detailed comments for Timer3 overflow one-shot ABS scheduling and ISR safety.
- Cleaned legacy mojibake/corrupted comments and replaced them with readable technical comments.
- Expanded private-state and helper documentation to make maintenance/debugging easier.

### Validation
- Source code before and after the documentation pass is identical after stripping comments and whitespace.
- No public API, register expression, algorithm or runtime behavior was intentionally changed.

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
