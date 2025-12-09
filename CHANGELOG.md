# Changelog

All notable changes to the MACRO (Mars Autonomous Cargo Rover Operations) project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

## [2.0.0] - 2025-12-09

### 🎉 Major Release - Reverse Recovery & Line State Tracking

### Added
- **Reverse Recovery System**: Automatic reverse when stuck in left/right state
  - `reverse_enabled` config - Enable/disable reverse recovery (default: true)
  - `reverse_speed` config - Speed when reversing (default: 10)
  - `stuck_threshold` config - Intervals in left/right state before triggering reverse
  - `reverse_intervals` config - Number of intervals to reverse
  - Robot straightens wheels while reversing, then resumes forward movement
- **Line State Tracking**: Line state now stored in `State.line_state`
  - Tracks "left", "center", "right" states for external monitoring
- `reverse_speed` as configurable variable separate from `forward_speed_slow`

### Changed
- Line state tracking moved from `MotionController.line_state` to `State.line_state`
- Turn functions (`turn_left`, `turn_right`) now clamp amount to max turn limit
- Fixed mode line following now uses `turn_left(max_turn)` / `turn_right(max_turn)`

### Fixed
- Fixed typo in `turn_right()` - was printing "Turning left" instead of "Turning Right"
- Reverse recovery now properly continues to next loop iteration after triggering

---

## [1.2.0] - 2025-12-08

### Added
- **Override Mode System**: Temporary override of line following
  - `override`, `override_mode`, `override_distance` in `[mobility]` config
  - `override_start_distance` and `override_end_distance` in State
  - Three override modes: `"straight"`, `"left"`, `"right"`
  - `trigger_override(distance)` method in MotionController
  - Robot straightens wheels when override ends and resumes line following
- **Target Cargo System**: Deploy at specific cargo number
  - `target_cargo_number` config - which cargo spot to deploy at (1-indexed)
  - `buffer_distance` config - distance to travel after non-target cargo
  - `cargo_number` tracking in State
  - `reset_detection()` method to clear magnetic values after passing cargo
- **Run Mode Toggle**: Display vs Control modes
  - `run_mode` in `[display]` config: `"display"` or `"control"`
  - Display mode: prints state to console (existing behavior)
  - Control mode: interactive keyboard console for override triggers
- **Control Console**: Keyboard-based override control
  - `w` - Trigger straight override
  - `a` - Trigger left override
  - `d` - Trigger right override
  - `q` - Quit
- `trigger_override(mode, distance)` method in Controller

### Changed
- Line following fixed mode now uses `turn_left(max_turn)` / `turn_right(max_turn)` functions
- Cargo system tracks multiple cargo spots and only deploys at target
- `run()` method now supports both display and control modes

---

## [1.1.0] - 2025-12-08

### Added
- `mobility_enabled` field in State for button-controlled mobility toggle
- Button press now toggles robot mobility on/off
- `_toggle_mobility()` method in Controller for button callback
- `cargo_bay_open` flag to track and confirm cargo bay state

### Changed
- `auto_line_follow()` now checks `state.mobility_enabled` in addition to `state.deploying_cargo`
- Cargo system uses raw `magnetic_field` instead of `mag_delta` for detection

### Removed
- `motor_speed` parameter from Cargo system (motor runs at default speed)
- `speed` setting from `[cargo.motor]` config section

---

## [1.0.1] - 2025-12-08

### Fixed
- Fixed async deploy loop blocking event loop - now uses `await asyncio.sleep()` to yield control
- Fixed distance comparison using `>=` instead of exact `==` for float comparison
- Cargo detection now uses raw `magnetic_field` instead of `mag_delta`
- Added `cargo_bay_open` flag to confirm deployment before closing
- Fixed TOML syntax error (trailing period in print_fields array)
- `deploy_and_close()` now properly awaits deployment confirmation before closing
- Moved `deploying_cargo` state management to `deploy()` and `close()` methods

### Changed
- Simplified magnetic field tracking - removed `get_magnetic_delta()`, uses `state.magnetic_field` directly
- `run_cargo_update_loop()` now calls `deploy_and_close()` instead of just `deploy()`
- Reduced close delay from 10 seconds to 2 seconds after confirmed deployment

---

## [1.0.0] - 2025-12-08

### 🎉 First Stable Release

This marks the first stable release of MACRO - Mars Autonomous Cargo Rover Operations.
The robot successfully navigates using line following, detects cargo via magnetic field sensing,
and deploys cargo at the correct location.

### Added
- `[mobility.line_follow]` configuration section in `macro_config.toml`:
  - `wheel_ratio` - Ratio between wheel motor degrees and turn motor degrees (default: 9.0)
  - `turn_amount` - Degrees to turn when correcting line position (default: 20)
  - `update_interval` - Seconds between line following updates (default: 0.1)
- `deploy_distance` parameter for Cargo system - uses `imu_to_cargo` from sensor config
- Distance-based cargo deployment - deploys when robot travels `deploy_distance` past max magnetic reading

### Changed
- Line following now uses configurable `line_follow_interval` instead of hardcoded value
- Cargo deployment timing now based on distance traveled from magnetic detection point
- `MobilityConfig` now includes `wheel_ratio`, `turn_amount`, and `line_follow_interval`

### Fixed
- Cargo deploy distance now correctly calculated from IMU to cargo position offset

---

## [0.12.4] - 2025-12-07

### Added
- Debouncing for cargo detection to prevent false positives from motor EMF
- `required_detections` config option (default: 5 consecutive detections needed)
- `[cargo.detection]` section in `macro_config.toml`

### Fixed
- `deploy()` and `close()` now use `blocking=True` so robot actually stops during deployment
- `close()` now sets `deploying_cargo` state to pause motion
- Improved line following state machine logic

---

## [0.13.0] - 2025-12-07

### Added
- Cargo motor support via Build HAT Motor class (port B)
- `deploy()` async method for deploying cargo (+180 degrees)
- `close()` async method for closing cargo bay (-180 degrees)
- `deploying_cargo` field in `State` to track deployment status
- Auto-deploy functionality: cargo automatically deploys on full detection, closes, and won't deploy again
- `[cargo.motor]` section in `macro_config.toml` with `port`, `speed`, and `deploy_angle` settings

### Changed
- `MotionController.auto_line_follow()` now pauses front motor when `state.deploying_cargo` is True
- `Cargo.deployed` field now serves dual purpose: tracks open state and prevents re-deployment after closing

---

## [0.12.3] - 2025-12-07

### Added
- `shutdown()` async method in `Controller` for graceful shutdown sequence

### Changed
- `run()` now calls `shutdown()` on KeyboardInterrupt for proper cleanup
- Shutdown sequence: straighten wheels → stop data collection → stop motors

---

## [0.12.2] - 2025-12-07

### Added
- `LineFinder` class now exported from `basehat` module

### Changed
- `line_finders` enabled by default in `macro_config.toml`

### Fixed
- `LineFinder` was not being imported or used in `sensors.py` (was commented out)
- `basehat/__init__.py` now correctly exports `LineFinder` class

---

## [0.12.1] - 2025-12-07

### Added
- `turn_position` field in `State` for tracking turn motor position
- `"turn"` option in controller's `print_state()` for displaying turn position

### Fixed
- `auto_line_follow()` now properly runs - combined safety monitoring and line following into single loop (was blocked by infinite `start_safety_ring()` call)

---

## [0.12.0] - 2025-12-07

### Added
- `CargoConfig` dataclass for cargo detection configuration
- `[cargo]` section in `macro_config.toml` with magnetic thresholds
- `turn_speed` and `max_turn` parameters in `MobilityConfig`
- `[mobility.turn]` section in config for turn limits
- `line_finder` option in controller's `print_state()` for displaying line finder values
- Line following auto-started in `Controller.run()` via `auto_line_follow()` task

### Changed
- All configuration now loaded from `macro_config.toml` (no hardcoded defaults in controller)
- `Cargo` class now receives threshold config from controller
- `Controller.print_state()` moved from `Navigation` class to `Controller`
- `Controller.run()` now starts line following task automatically
- `Navigation.run_continuous_update()` simplified - print functionality moved to controller
- Gyro bias subtraction now properly reorders bias from `[gx, gy, gz]` to `[yaw, pitch, roll]`
- `Location.update()` now checks calibration flags before accessing bias values

### Removed
- `print_state()` method from `Navigation` class
- `reverse_speed` from `MobilityConfig` (unused - reverse uses `-forward_speed`)
- Broken `start_update_loop()` method from `Controller`

### Fixed
- `Navigation.update_state()` no longer overwrites state values with boolean returns
- Motor velocity reference in navigation now uses `self.state.motor_velocity` instead of `self.motion_controller.motor_velocity`
- Gyro bias axis order mismatch in `update_orientation()`
- Potential crash in `Location.update()` when calibration not performed

---

## [0.11.0] - 2025-12-07

### Added
- `State` dataclass in `systems/state.py` for centralized state management
- `sensor_positions` dict in `State` for tracking sensor locations (imu, lf_left, lf_right, color_sensor, cargo_deploy)
- Sensor offset configuration parameters in `macro_config.toml`:
  - `imu_height`, `color_sensor_height`, `lf_height`, `lf_offset`
  - `imu_to_lf`, `imu_to_color`, `imu_to_cargo`

### Changed
- `Navigation` and `Location` classes now use `State` dataclass instead of instance variables
- `NavigationDisplay` now pulls from `State` directly via `update_from_state()` method
- `NavigationDisplay` runs independent update loop from Navigation class
- `update_imu_position()` now updates IMU position in `sensor_positions["imu"]` and calculates ground position
- `update_positions_from_imu()` calculates other sensor positions relative to ground position
- `State` uses `field(default_factory=...)` for mutable numpy array defaults

### Removed
- Logging functionality from `Navigation` class (`log_state()` method, `log` attribute)
- `log_state` configuration option from `NavigationConfig` and `macro_config.toml`
- Duplicate `motor_velocity` field in `State` dataclass
- Unused `dataclass` import from `systems/__init__.py`

### Fixed
- `State` dataclass now properly exports in `systems/__init__.py` via `__all__`
- Orientation order comment corrected to (yaw, pitch, roll)

---

## [0.10.1] - 2025-12-06

### Added
- Customizable readings and logs in the test runner via `test_config.toml`.
- New sections in `test_config.toml`: `[logging]`, `[console]`, `[readings]`.
- Updated `test.py` to parse and use these new configurations.

---

## [0.10.0] - 2025-12-06

### Added
- `macro_config.toml` - Centralized configuration file for all MACRO systems
- `controller.py` - Central controller that reads config and initializes all modules
- Configuration sections: `[sensors]`, `[mobility]`, `[navigation]`, `[display]`, `[testing]`
- `Config`, `SensorConfig`, `MobilityConfig`, `NavigationConfig`, `DisplayConfig`, `TestingConfig` dataclasses
- `load_config()` function for parsing TOML configuration
- `Controller` class with `initialize()`, `run()`, and `stop()` methods

### Changed
- Renamed `Transformation3D` → `Transformation`
- Renamed `Location3D` → `Location`
- Renamed `Navigation3D` → `Navigation`
- Moved `task_manager.py` from `systems/` to `controller.py` at project root
- All sensor methods now async (get_accel, get_gyro, get_mag, get_distance, etc.)

### Removed
- `systems/task_manager.py` (replaced by `controller.py`)
- Thermal configuration section (not implemented)

---

## [0.9.0] - 2025-12-06

### Added
- `ColorSensor` support in `SensorInput` via Build HAT
- `color_sensor` and `color_sensor_port` parameters for initialization
- `get_color()` method returning detected color name
- `is_black()` method returning 1 for black, 0 for other colors
- `has_color_sensor()` availability check

### Deprecated
- `HallSensor` removed from `basehat` exports - use IMU magnetometer instead
- `get_hall_value()` method in `SensorInput` - use `get_mag()` or `get_magnetic_magnitude()`
- `has_hall()` method in `SensorInput` - use `has_imu()` with magnetometer methods

---

## [0.8.2] - 2025-12-06

### Added
- Dynamic scale in `NavigationDisplay` based on window size and world bounds
- Zoom controls: mouse wheel, +/- keys, `zoom_in()`, `zoom_out()` methods
- `set_world_bounds(min, max)` for programmatic view control
- `world_min` and `world_max` parameters (default: -10 to 10 meters)
- Magnetic field indicator ring around rover (0.06m radius)

### Changed
- Scale now auto-calculates from window dimensions and world bounds
- Magnetic field visualization: ring around rover instead of background color
- Default scale adjusted for -10 to 10m world bounds

---

## [0.8.1] - 2025-12-06

### Added
- Motor velocity display in `NavigationDisplay` info panel
  - `motor_velocity` state variable (degrees/second)
  - `motor_vel_label` showing live motor velocity
- `update_from_navigator()` now pulls motor velocity from `navigator.motion_controller`

### Changed
- Magnetic field gradient range: 0-400 µT → 0-5000 µT

### Fixed
- Added missing `await` for `get_magnetic_field()` in `calibrate()` method

---

## [0.8.0] - 2025-12-06

### Added
- Motor encoder velocity tracking in `MotionController`
  - `motor_position` and `motor_velocity` state variables
  - `update_motor_state(dt)` async method for encoder updates
  - `get_velocity()` and `get_position()` methods
- `motion_controller` parameter in `Location`/`Navigation` for motor-based velocity decay
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

## [0.7.1] - 2025-12-06

### Added
- Magnetic field gradient background in `NavigationDisplay`
  - 0 µT = white (#FFFFFF), 400 µT = pure blue (#0000FF)
  - Linear interpolation based on `magnetic_magnitude`
- `magnetic_magnitude` tracking in display
- Magnetic field info label in display panel

### Changed
- Minor grid lines (10cm) made much lighter (#101010)

---

## [0.7.0] - 2025-12-05

### Added
- `update_state()` method in `Navigation` - unified update for position, orientation, and magnetic field
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

## [0.6.1] - 2025-12-05

### Fixed
- Correct integration order in `update_position()` - position and velocity now update using previous timestep values
- Position uses: `p(t) = p(t-1) + v(t-1)*dt + 0.5*a(t-1)*dt²`
- Velocity uses: `v(t) = v(t-1) + a(t-1)*dt`
- New acceleration calculated after position/velocity update for next iteration
- Test files now calibrate IMU before starting motors

---

## [0.6.0] - 2025-12-04

### Added
- `NavigationDisplay` UI component for real-time rover visualization
- 2D grid display with 1m major gridlines and 0.1m minor gridlines
- Black dot + blue velocity arrow for rover position/heading
- Info panel showing position, orientation, velocity, acceleration
- Resizable window with adaptive canvas
- `Navigation` integration via `run_continuous()` async loop
- IMU calibration system with `calibrate()` method
- `velocity_decay` parameter to reduce drift when stationary
- `accel_threshold` parameter to filter sensor noise
- `tests/navigation_test.py` - mobility + Navigation test
- `tests/navigation_display_test.py` - mobility + Navigation + display test
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

## [0.5.2] - 2025-12-04

### Added
- `Transformation` export in `systems/__init__.py`
- `get_position()` method in `Location` returning position as tuple

---

## [0.5.1] - 2025-12-04

### Added
- `log_state` kwarg for `Navigation.run_continuous_update()` (default: True)
- `print_state` kwarg for `Navigation.run_continuous_update()` (default: False)
- `print_state()` method for formatted state output
- `log_state()` method (renamed from private)

---

## [0.5.0] - 2025-12-04

### Added
- `Navigation` class inheriting from `Location` with timestamped logging
- Navigation state logging (position, velocity, acceleration, orientation)
- `start_time` tracking for relative timestamps

### Changed
- Moved `run_continuous_update` from `Location` to `Navigation`
- Updated `systems/__init__.py` to export `Location` and `Navigation`

---

## [0.4.1] - 2025-12-04

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

## [0.4.0] - 2025-12-04

### Added
- Comprehensive docstrings for `navigation_system.py` classes and methods
- `Transformation` class with full 3D rotation matrix support (yaw, pitch, roll)
- `Location` class for IMU-based position tracking with dead reckoning
- Gravity compensation in position tracking
- First-iteration handling to prevent acceleration spikes

### Changed
- All `Transformation` methods now use `**kwargs` for consistent API
- Improved code formatting and readability in navigation system
- Enhanced rotation matrix generation with proper ZYX convention

### Fixed
- IMU no longer auto-instantiates when not provided
- Proper null checks for IMU sensor in update methods

---

## [0.3.0] - 2025-12-04

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

## [0.2.0] - 2025-12-04

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

## [0.1.0] - 2025-10-15

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
