# Changelog

All notable changes to the MACRO (Mars Autonomous Cargo Rover Operations) project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

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
