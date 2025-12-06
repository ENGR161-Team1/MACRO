# Changelog

All notable changes to the MACRO (Mars Autonomous Cargo Rover Operations) project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

---

## [0.8.1] - 2024-12-06

### Added
- Motor velocity display in `NavigationDisplay` info panel
  - `motor_velocity` state variable (degrees/second)
  - `motor_vel_label` showing live motor velocity
- `update_from_navigator()` now pulls motor velocity from `navigator.motion_controller`

---

## [0.8.0] - 2024-12-06

### Added
- Motor encoder velocity tracking in `MotionController`
  - `motor_position` and `motor_velocity` state variables
  - `update_motor_state(dt)` async method for encoder updates
  - `get_velocity()` and `get_position()` methods
- `motion_controller` parameter in `Location3D`/`Navigation3D` for motor-based velocity decay
- `motor_velocity_threshold` parameter (default: 1.0 degrees/second)
- Test fixtures system in `tests/fixtures/`
  - `NavigationConfig` dataclass for navigation settings
  - `MobilityConfig` dataclass for mobility settings
  - `create_navigator()` and `create_motion_controller()` factory functions

### Changed
- `print_state()` now accepts `fields` array parameter for customizable output
  - Options: `["all"]`, `["position", "velocity"]`, `["magnetic"]`, etc.
- Velocity decay now triggers based on motor velocity threshold (if motion_controller provided)
- Default `velocity_decay`: 0.02 → 0.04
- Default `accel_threshold`: 0.1 → 0.05 m/s²
- Refactored test files to use shared fixtures with configuration sections

### Tests
- Created `tests/fixtures/` module with shared configurations
- `navigation_test.py` - navigation-only with configurable modes
- `nav_mobility_test.py` - navigation + mobility with toggle flags
- `navigation_display_test.py` - full visualization test with all toggles

---

## [0.7.1] - 2024-12-06

### Added
- Magnetic field gradient background in `NavigationDisplay`
  - 0 µT = white (#FFFFFF), 400 µT = pure blue (#0000FF)
  - Linear interpolation based on `magnetic_magnitude`
- `magnetic_magnitude` tracking in display
- Magnetic field info label in display panel

### Changed
- Minor grid lines (10cm) made much lighter (#101010)

---

## [0.7.0] - 2024-12-05

### Added
- `update_state()` method in `Navigation3D` - unified update for position, orientation, and magnetic field
- `get_magnetic_field()` async method returning magnetic field magnitude in micro-tesla
- `magnetic_magnitude` attribute and `mag_baseline` calibration value
- Magnetic field baseline calibration during `calibrate()`
- Magnetism logging in `log_state()` with `magnetic_magnitude` field
- Magnetism display in `print_state()` output

### Changed
- Separated orientation update from `update_position()` into `update_state()`
- `run_continuous_update()` now calls `update_state()` instead of `update_position()`
- Position and velocity rounded to 3 decimal places in `print_state()`
- `get_magnetic_field()` is now async

### Tests
- Renamed `navigation_test.py` → `nav-mobility_test.py`
- Renamed `navigation_test_exclusive.py` → `navigation_test.py`
- Updated `navigation_test.py` to use async `get_magnetic_field()`

---

## [0.6.1] - 2024-12-05

### Fixed
- Correct integration order in `update_position()` - position and velocity now update using previous timestep values
- Position uses: `p(t) = p(t-1) + v(t-1)*dt + 0.5*a(t-1)*dt²`
- Velocity uses: `v(t) = v(t-1) + a(t-1)*dt`
- New acceleration calculated after position/velocity update for next iteration
- Test files now calibrate IMU before starting motors

---

## [0.6.0] - 2024-12-04

### Added
- `NavigationDisplay` UI component for real-time rover visualization
- 2D grid display with 1m major gridlines and 0.1m minor gridlines
- Black dot + blue velocity arrow for rover position/heading
- Info panel showing position, orientation, velocity, acceleration
- Resizable window with adaptive canvas
- `Navigation3D` integration via `run_continuous()` async loop
- IMU calibration system with `calibrate()` method
- `velocity_decay` parameter to reduce drift when stationary
- `accel_threshold` parameter to filter sensor noise
- `tests/navigation_test.py` - mobility + Navigation3D test
- `tests/navigation_display_test.py` - mobility + Navigation3D + display test
- `tests/__init__.py` for test package
- `ui/__init__.py` module exports

### Changed
- Arrow direction now based on velocity vector (vx, vy) instead of yaw
- Arrow length scales with speed
- `run_continuous_update()` now auto-calibrates IMU on start
- Acceleration is thresholded to filter noise
- Velocity decays toward zero when acceleration is near zero

### Fixed
- IMU drift causing constant velocity accumulation when stationary
- Gravity removal now uses calibrated bias instead of fixed offset
- Orientation updated before acceleration transform

---

## [0.5.2] - 2024-12-04

### Added
- `Transformation3D` export in `systems/__init__.py`
- `get_position()` method in `Location3D` returning position as tuple

---

## [0.5.1] - 2024-12-04

### Added
- `log_state` kwarg for `Navigation3D.run_continuous_update()` (default: True)
- `print_state` kwarg for `Navigation3D.run_continuous_update()` (default: False)
- `print_state()` method for formatted state output
- `log_state()` method (renamed from private)

---

## [0.5.0] - 2024-12-04

### Added
- `Navigation3D` class inheriting from `Location3D` with timestamped logging
- Navigation state logging (position, velocity, acceleration, orientation)
- `start_time` tracking for relative timestamps

### Changed
- Moved `run_continuous_update` from `Location3D` to `Navigation3D`
- Updated `systems/__init__.py` to export `Location3D` and `Navigation3D`

---

## [0.4.1] - 2024-12-04

### Added
- `CONTRIBUTING.md` with contribution guidelines
- `docs/API.md` with module and class reference documentation
- `docs/HARDWARE.md` with hardware setup guide
- `poc/poc1.py` line following motor control demonstration
- `poc/poc_2.py` additional POC from poc2 branch

### Changed
- Renamed `poc/proof_of_concept.py` to `poc/poc1.py`
- Merged and deleted `poc1` and `poc2` branches into main
- Updated `docs/README.md` with complete documentation links
- Updated `poc/README.md` to reflect current POC contents
- Updated version to 0.4.1 in `pyproject.toml`

### Removed
- `poc/LineFinder.py` (moved to basehat module)

---

## [0.4.0] - 2024-12-04

### Added
- Comprehensive docstrings for `navigation_system.py` classes and methods
- `Transformation3D` class with full 3D rotation matrix support (yaw, pitch, roll)
- `Location3D` class for IMU-based position tracking with dead reckoning
- Gravity compensation in position tracking
- First-iteration handling to prevent acceleration spikes

### Changed
- All `Transformation3D` methods now use `**kwargs` for consistent API
- Improved code formatting and readability in navigation system
- Enhanced rotation matrix generation with proper ZYX convention

### Fixed
- IMU no longer auto-instantiates when not provided
- Proper null checks for IMU sensor in update methods

---

## [0.3.0] - 2024-12-04

### Added
- `basehat/__init__.py` for clean module imports
- `systems/__init__.py` for clean module imports
- Location class in `navigation_system.py` for position tracking

### Changed
- **Project renamed from MARCO to MACRO** (Mars Autonomous Cargo Rover Operations)
- Consolidated branch structure: merged `mobility`, `navigation`, `IMU-test` branches
- Standardized basehat file naming to snake_case
- Renamed `UltrasonicSensor.py` → `ultrasonic_sensor.py`
- Fixed import paths across all modules

### Removed
- Removed duplicate `HallSensor.py` (keeping `hall_sensor.py`)
- Deleted obsolete branches: `mobility`, `navigation`, `IMU-test`, `hatModules`

---

## [0.2.0] - 2024-12-04

### Added
- Build HAT communication and motion control system
- Motor control and serial interface modules for Build HAT
- Obstacle detection and motor control in `mobility_test.py`
- Hall sensor modules (`HallSensor.py`, `hall_sensor.py`)
- Ultrasonic sensor module (`UltrasonicSensor.py`)
- Button module for user input
- IMU sensor module for orientation and motion sensing
- Line finder module for line detection
- Mobility system module
- Color sensor support via Build HAT
- CHANGELOG.md for version tracking
- Improved README with repository structure documentation
- POC directory documentation
- Testing branch with `tests/` directory structure
- Documentation branch with `docs/` directory structure

### Changed
- Reorganized basehat modules from `modules/basehat/` to `basehat/`
- Reorganized buildhat modules from `modules/buildhat/` to `buildhat/`

### Removed
- Deprecated `motion_control.py` module (functionality moved to `mobility_system.py`)
- Removed `thermal_system.py` (thermal system not needed for robot design)
- Deleted `hatModules` branch (merged and obsolete)

---

## [0.1.0] - 2024-10-15

### Added
- Initial project structure
- Basic sensor input class in `sensors.py`
- Navigation system foundation
- Task manager system
- Proof of concept files for motor control testing
- Project configuration via `pyproject.toml`

---

## POC (Proof of Concept) History

### POC 1 - Basic Motor Control
- Simple motor pair initialization and timed movement
- Demonstrated Build HAT motor control basics

### POC 2 - Line Following
- LineFinder class integration
- Motor control logic for line following behavior

---

## Branch Overview

| Branch | Purpose | Status |
|--------|---------|--------|
| `main` | Production-ready code | Active |
| `systems` | Unified systems development (mobility, navigation, sensors) | Active |
| `testing` | Test files and test development | Active |
| `documentation` | Documentation updates | Active |
| `poc1` | Proof of Concept 1 | Archived |
| `poc2` | Proof of Concept 2 | Archived |
