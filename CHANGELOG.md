# Changelog

All notable changes to this project will be documented here.

## [Unreleased]

### Validation
- Hardware validation on Arduino Mega + two L298N boards is still required before the 1.0.0 stable release.

## [0.11.1] - 2026-10-06

### PS2 + IMU field-centric integration
- Added `PS2IMUHeadless`, an application-level template combining TungLam_PS2, TungLam_HWT901B, TungLam_FuzzyPID and this drive library.
- Left-stick translation is interpreted in the field frame and rotated into the robot body frame from IMU yaw.
- Right-stick X provides manual yaw rate while translation remains field-centric.
- Releasing the right stick captures the new heading and enables PID/Fuzzy heading hold.
- SELECT re-zeros the field frame without modifying the IMU driver's internal angle offset.
- Added stale angle/gyro checks and fail-safe stop on PS2/IMU loss.
- Added 100 Hz heading control with gyro-Z derivative feedback and explicit loop dt.
- Compile CI now pins PS2 v0.5.0, HWT901B v0.1.1 and FuzzyPID v0.1.0.

### Compatibility
- No motor HAL, timer, braking, Mecanum/Omni equations or public drive API changed.
- The new headless behavior lives entirely in the example/application layer.
- Existing V5 and PS2RobotControl behavior remains unchanged.

## [0.11.0] - 2026-10-02

### Project-template examples
- Reduced the Arduino IDE menu from fourteen API-focused examples to four reusable project templates: `FirstMotorTest`, `RobotTemplate`, `PS2RobotControl`, and `VelocityControlTemplate`.
- Added `RobotTemplate` with separated input, drive, safety and mechanism hooks.
- Reworked `PS2RobotControl` into an application skeleton with the proven right-stick rotation priority plus ready-made button/mechanism hooks.
- Added `VelocityControlTemplate` for ROS2/Serial/PC/autonomous command sources; users only implement `readVelocityCommand()`.
- Simplified `FirstMotorTest` while preserving low-PWM M1..M4 commissioning.
- Moved the previous detailed demos to `extras/reference-examples/` and their English mirrors to `extras/reference-examples-en/` instead of deleting them.

### Compatibility
- No motor HAL, timer, Mecanum/Omni equations, braking, SI kinematics or public API behavior changed.
- V5 wheel mapping and PS2 right-stick priority remain regression-protected.
- Hardware validation is still required before the 1.0.0 stable release.

## [0.10.5] - 2026-10-02

### Full V5 baseline audit
- Added `extras/V5_BASELINE.md` and English mirror as the single ground-truth reference for the hardware-proven V5 chassis.
- Locked the physical mapping:
  - M1 = front-left = PWM D5
  - M2 = rear-left = PWM D6
  - M3 = rear-right = PWM D7
  - M4 = front-right = PWM D8
  - code order = M1,M2,M3,M4
  - clockwise physical order = M1 -> M4 -> M3 -> M2
- Clarified that wheel-vector signs are channel command polarities, not a direct visual description of shaft rotation.
- Cross-linked README, wiring guide and kinematics guide to the V5 baseline.

### PS2 control examples
- Simplified `PS2RobotControl` so it teaches only the proven RoboBall/V5 driving policy:
  - left stick = forward/backward/strafe
  - right-stick LEFT/RIGHT = rotation with explicit higher priority
  - releasing the right stick returns control to the still-held left-stick command
- Removed unrelated L1/R1 speed-mode behavior from the recommended driving example.
- Kept `PS2RobotVectorMix` as the only advanced second driving example for simultaneous translation + rotation.
- Specialized button/debug/raw/reconnect examples remain in TungLam_PS2, keeping responsibilities separated.

### Regression protection
- Added `tools/check_v5_baseline.py`.
- CI now fails if M3/M4 physical mapping, V5 Mecanum basis, clockwise wheel order, or recommended PS2 right-stick priority regresses.
- Expanded modern API Doxygen comments with V5-proven wheel order and lateral basis.
- Compile CI now pins `TungLam_PS2 v0.4.1`.

### Compatibility
- Legacy V5 executable behavior is unchanged.
- Modern Mecanum mixer remains at the V5-proven equations restored in v0.10.3.
- Omni-X, timers, braking and public executable motor APIs are unchanged.

## [0.10.4] - 2026-10-02

### PS2 driving examples
- Reworked `PS2RobotControl` into the recommended RoboBall/V5-style control flow.
- Left stick controls forward/backward/strafe.
- Right-stick LEFT/RIGHT has explicit higher priority and overrides left-stick translation.
- Releasing the right stick returns control to the still-held left-stick command.
- Added `PS2RobotVectorMix` as a separate advanced example for simultaneous translation + rotation through `drive(vx, vy, wz)`.
- Kept PS2 application mapping outside both library cores.
- Updated Vietnamese/English documentation to explain the two control styles and the correct V5 physical wheel layout:
  - M1 front-left
  - M2 rear-left
  - M3 rear-right
  - M4 front-right

### Reference to historical projects
- The priority example mirrors the behavior found in earlier RoboBall projects where right-stick rotation commands were evaluated after left-stick translation commands.
- The new implementation makes that arbitration explicit instead of depending on statement order.

### Compatibility
- No motor core, Mecanum/Omni kinematics, V5 API, timer, braking, or PS2 protocol behavior changed.

## [0.10.3] - 2026-10-02

### Fixed
- Restored the proven V5/modern Mecanum command basis after the v0.10.2 regression.
- Corrected the physical wheel mapping everywhere in documentation and API comments:
  - M1 = front-left = PWM D5
  - M2 = rear-left = PWM D6
  - M3 = rear-right = PWM D7
  - M4 = front-right = PWM D8
  - clockwise physical order: M1 -> M4 -> M3 -> M2
- Restored modern Mecanum basis to:
  - +vx forward: `+ + + +`
  - +vy left: `- + - +`
  - -vy right: `+ - + -`
  - +wz CCW: `- - + +`
- Restored diagonal vectors:
  - forward-right: `+ 0 + 0`
  - forward-left: `0 + 0 +`
  - backward-right: `0 - 0 -`
  - backward-left: `- 0 - 0`
- Restored SI inverse/forward kinematics to the v0.10.1 equations, which match the proven V5 channel ordering.
- Added compile-time regression assertions for both lateral directions and both forward diagonals.
- Clarified throughout the docs that vector order `[M1,M2,M3,M4]` is not the same as clockwise physical order.

### Compatibility
- Legacy V5 behavior remains unchanged.
- PS2 integration example remains unchanged; `+vy` again maps to the same physical left-strafe behavior as V5.
- Omni-X equations are unchanged.

## [0.10.2] - 2026-10-02

> **Superseded by v0.10.3.** This release introduced a Mecanum regression because the physical wheel positions were assumed incorrectly.

### Historical regression
- v0.10.2 incorrectly assumed:
  - M3 = front-right
  - M4 = rear-right
  - clockwise order = M1 -> M3 -> M4 -> M2
- Based on that wrong assumption, the modern Mecanum lateral basis was changed away from the proven V5 behavior.
- v0.10.3 restores the v0.10.1/V5-proven basis and documents the correct physical mapping:
  - M1 = front-left
  - M2 = rear-left
  - M3 = rear-right
  - M4 = front-right
  - clockwise physical order = M1 -> M4 -> M3 -> M2

## [0.10.1] - 2026-10-02

### PS2 integration example
- Added `examples/PS2RobotControl` with detailed Vietnamese comments.
- Added matching English reference under `extras/examples-en/PS2RobotControl`.
- Example uses TungLam_PS2 v0.4.0 as an optional input layer, not a hard motor-core dependency.
- Left stick controls vx/vy; right-stick horizontal direction controls wz.
- L1/R1 demonstrate held-button speed selection; START demonstrates one-shot button events.
- Lost PS2 connection immediately calls `robot.stop()`.
- Optional event-driven PS2 debug can be enabled without adding Serial spam to production builds.
- Mega compile CI now pins and installs TungLam_PS2 v0.4.0 before compiling all examples.

### Compatibility
- No motor-control, timer, pin-map, braking, kinematics, modern API, or Legacy V5 behavior changed.

## [0.10.0] - 2026-10-01

### Smart control and AVR optimization
- Cached motor/chassis derived constants at configuration time so the frequent SI control path no longer recomputes repeated RPM/geometry divisions per wheel.
- Added `enableSmartSafety()` as a beginner-friendly one-call setup for command watchdog and SI velocity smoothing.
- Added configurable command watchdog through `setCommandTimeoutMs()`; `update()` safely stops a moving robot after command loss.
- Added `setVelocityRamp()` / `disableVelocityRamp()` for component-wise vx/vy/wz slew-rate limiting in SI units.
- Added saturation telemetry: `wasVelocityLimited()`, `lastVelocityScale()`, `requestedBodyVelocity()`, and `appliedBodyVelocity()`.
- Added `StudentQuickStart` as the recommended beginner example.
- Added automatic public-API documentation coverage checking; 75/75 public functions must keep Vietnamese Arduino IDE hints.

### Compatibility
- Legacy V5 executable behavior remains unchanged.
- Smart Safety is opt-in, so existing modern projects keep their previous immediate-command behavior unless explicitly enabled.
- Velocity ramp affects `driveVelocity()` only; direct PWM/normalized APIs remain immediate.

## [0.9.2] - 2026-10-01

### Complete Arduino IDE API hints
- Completed Vietnamese Doxygen parameter documentation for all remaining Legacy V5 directional helpers.
- Verified every public function in both APIs has a Vietnamese `@brief`; every function with parameters has `@param`; every non-void function has `@return`.
- Public API documentation coverage is now 65/65 functions: 29 Legacy V5 + 36 Modern API.
- Synchronized the corresponding English source-reference comments.

### Compatibility
- Documentation-only patch; no executable behavior change from v0.9.1.

## [0.9.1] - 2026-10-01

### Vietnamese-first localization
- Made the default README, wiring guide, kinematics guide, Arduino examples, public header comments, compatibility header comments, and implementation comments Vietnamese-first for Vietnamese students and learners.
- Added `README.en.md`, `extras/WIRING.en.md`, `extras/KINEMATICS.en.md`, `extras/examples-en/`, and `extras/source-en/` as international English references.
- Kept API names and executable logic language-neutral; localization changes comments/documentation only.
- Added explicit language navigation between Vietnamese and English documentation.
- Added localization parity validation so Vietnamese and English source/example views must have identical executable/declaration code after comments and whitespace are removed.
- Expanded Vietnamese Doxygen `@brief`, `@param`, `@return`, enum and struct-member documentation so Arduino IDE 2.x code-completion/signature-help/hover can explain what each public API does while students type.
- Expanded `keywords.txt` to highlight SI configuration and velocity fields such as `wheelRadiusM`, `wheelbaseM`, `vxMps`, `vyMps`, and `wzRadps`.

### Compatibility
- No motor-control, kinematics, ABS, timer, pin-map, SI model, Legacy V5, or public API behavior change from v0.9.0.

## [0.9.0] - 2026-10-01

### Standard body-frame convention
- Standardized the modern Cartesian API to a right-handed mobile-robot frame: +X/+vx forward, +Y/+vy left, +Z/+wz counter-clockwise.
- Updated the modern Mecanum mixer so +vy produces the proven V5 left-strafe vector `[-,+,-,+]` and +wz produces the proven V5 CCW vector `[-,-,+,+]`.
- Corrected the built-in normalized Omni-X mixer to a documented canonical 45-degree wheel-axis model.
- Kept all legacy `TungLam_Control_MotorV5` movement functions and physical movement semantics unchanged.

### SI kinematics and physical drive model
- Added `TungLamDriveConfig` for motor rated voltage, gearbox/output no-load RPM, supply voltage, wheel radius, wheelbase, track width and empirical `speedScale`.
- Added `driveVelocity(vxMps, vyMps, wzRadps)` using metres/second and radians/second.
- Added Mecanum-X and canonical Omni-X inverse kinematics through `inverseKinematics()`.
- Added forward kinematics through `forwardKinematics()` for teaching and future encoder/odometry integration.
- Added estimated motor RPM, maximum wheel speed, body translation speed and yaw-rate helpers.
- Added proportional metric wheel-speed limiting before open-loop conversion to signed PWM.
- Added explicit open-loop documentation: requested SI velocity is a feed-forward estimate until encoders close the wheel-speed loop.

### Education and future control
- Added `examples/MetricKinematics` with line-by-line comments, SI configuration, inverse/forward kinematics and future IMU heading-hold integration.
- Added `extras/KINEMATICS.md` with coordinate frames, Mecanum/Omni equations, RPM/voltage/wheel-radius conversion, PWM feed-forward, encoder PID architecture, IMU yaw PID architecture and ROS-style velocity mapping.
- Updated README, wiring documentation and modern examples to the new +Y-left / +Z-CCW convention.

### Compatibility
- This is an intentional modern-API Cartesian sign change for `drive()`, `driveMecanum()` and `driveOmniX()`: callers that passed raw positive `vy` for right or positive `wz` for clockwise must invert those arguments when migrating.
- Named helpers such as `strafeRight()`, `strafeLeft()`, `rotateRight()` and `rotateLeft()` keep their semantic physical direction.
- Legacy V5 API behavior is unchanged.

## [0.8.2] - 2026-09-30

### Documentation & onboarding
- Rebuilt the README into a beginner-friendly and production-style project landing page with CI/release badges, Mermaid architecture diagrams, wiring maps, quick-start flows, braking diagrams, troubleshooting, and API guidance.
- Added explicit Arduino Mega -> two L298N -> four motor wiring tables and a complete common-ground/power checklist.
- Added a PS2 integration section that keeps PS2X as an optional external input layer instead of coupling it to the motor core.
- Expanded `extras/WIRING.md` into a step-by-step commissioning and troubleshooting guide.
- Added `FirstMotorTest` to verify M1..M4 placement and polarity safely at low PWM before chassis testing.
- Reworked every `.ino` example with detailed file-level, setup, state, command, timing, braking, and per-line operational comments so beginners can learn directly from Arduino IDE examples.
- Expanded Arduino IDE keyword highlighting for the complete modern and legacy public API.
- Improved `library.properties` metadata for Library Manager users.

### Compatibility
- No intended motor-control, ABS, timer, pin-map, Mecanum, Omni, or V5 public API behavior change.

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
- Added the modern logical wheel-order diagram (M1 front-left, M2 rear-left, M3 rear-right, M4 front-right) and vx/vy/wz direction convention.
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
