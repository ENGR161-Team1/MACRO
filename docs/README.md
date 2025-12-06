# MACRO Documentation

This directory contains detailed documentation for the MACRO (Mars Autonomous Cargo Rover Operations) project.

**Current Version: 0.8.0**

## Contents

| Document | Description |
|----------|-------------|
| [API.md](API.md) | Module and class reference documentation |
| [HARDWARE.md](HARDWARE.md) | Pin configurations, wiring diagrams, and hardware setup |

## Quick Start

```python
import asyncio
from basehat import IMUSensor
from systems import Navigation3D
from systems.mobility_system import MotionController

imu = IMUSensor()
motion = MotionController(front_motor="A", turn_motor="B")
navigator = Navigation3D(
    imu=imu, 
    mode="degrees",
    motion_controller=motion
)

async def main():
    await navigator.run_continuous_update(
        update_interval=0.1,
        log_state=True,
        print_state=True,
        calibrate=True
    )

asyncio.run(main())
```

## Quick Links

- [Main README](../README.md) - Project overview and installation
- [CHANGELOG](../CHANGELOG.md) - Version history
- [CONTRIBUTING](../CONTRIBUTING.md) - Contribution guidelines

## Documentation Status

| Document | Status |
|----------|--------|
| README.md | ✅ Complete |
| CHANGELOG.md | ✅ Active |
| CONTRIBUTING.md | ✅ Complete |
| API Reference | ✅ Updated for v0.8.0 |
| Hardware Guide | ✅ Complete |
