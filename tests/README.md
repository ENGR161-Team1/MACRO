# Tests

This directory contains all test files for the MACRO project.

## Unified Test Runner

The primary way to run tests is via the unified `test.py` with configurable `test_config.toml`:

```bash
# Run with default config
python tests/test.py

# Run with a preset config
python tests/test.py tests/configs/navigation_only.toml
python tests/test.py tests/configs/manual_display.toml
python tests/test.py tests/configs/full_auto.toml
python tests/test.py tests/configs/magnetism.toml
python tests/test.py tests/configs/mobility_only.toml
```

## Configuration

Edit `test_config.toml` or create a custom config to toggle features:

```toml
[features]
navigation = true       # Position tracking via IMU
mobility = true         # Motor control
display = true          # Visual navigation display
safety_ring = true      # Ultrasonic obstacle avoidance
manual_control = false  # Keyboard w/a/s/d control
autonomous = true       # Automatic forward movement
payload = false         # Payload motor control
magnetism = false       # Magnetic field monitoring
```

## Preset Configs

| Config | Description |
|--------|-------------|
| `navigation_only.toml` | IMU navigation without motors |
| `manual_display.toml` | Keyboard control with display |
| `full_auto.toml` | Autonomous with safety ring and display |
| `magnetism.toml` | Magnetic field monitoring only |
| `mobility_only.toml` | Motor control without navigation |

## Legacy Test Files

Individual test files are still available for specific testing:

| File | Description |
|------|-------------|
| `imu_test.py` | Raw IMU sensor readings |
| `mobility_test.py` | Motor control with manual input |
| `navigation_test.py` | Navigation-only testing |
| `nav_mobility_test.py` | Navigation + mobility combined |
| `navigation_display_test.py` | Navigation + display |
| `nav_manual_display_test.py` | Manual control + navigation + display |

## Running Legacy Tests

```bash
python tests/mobility_test.py
python tests/navigation_test.py
```

