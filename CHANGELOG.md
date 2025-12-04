# Changelog

All notable changes to the MARCO (Mars Cargo Rover) project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

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

### Changed
- Reorganized basehat modules from `modules/basehat/` to `basehat/`
- Reorganized buildhat modules from `modules/buildhat/` to `buildhat/`

### Removed
- Deprecated `motion_control.py` module (functionality moved to `mobility_system.py`)

---

## [0.1.0] - 2024-11-XX (Initial Development)

### Added
- Initial project structure
- Basic sensor input class in `sensors.py`
- Navigation system foundation
- Task manager system
- Thermal system module
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
| `mobility` | Mobility system development | Merged to main |
| `navigation` | Navigation system development | Active |
| `hatModules` | HAT module development | Merged to main |
| `IMU-test` | IMU sensor testing | Active |
| `poc1` | Proof of Concept 1 | Archived |
| `poc2` | Proof of Concept 2 | Archived |
