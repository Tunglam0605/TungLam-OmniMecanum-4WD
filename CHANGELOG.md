# Changelog

All notable changes to this project will be documented here.

## [Unreleased]

### Validation
- Hardware validation on Arduino Mega + two L298N boards is still required before the 1.0.0 stable release.

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
- Automatic reverse-brake duty limiting
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
