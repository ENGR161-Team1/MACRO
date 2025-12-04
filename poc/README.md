# Proof of Concepts (POC)

This directory contains experimental code and proof-of-concept implementations used during MARCO development.

## Contents

### `proof_of_concept.py`
**Purpose**: Basic motor control demonstration  
**Status**: Complete  
**Description**: Simple test to verify Build HAT motor pair initialization and timed movement control.

### `poc_example.py`  
**Purpose**: Navigation system POC with PID line following  
**Status**: Reference implementation  
**Description**: Comprehensive navigation implementation featuring:
- PID-based steering control
- Line sensor array handling
- Gap detection and reacquisition
- Curvature-constrained path following
- Hardware abstraction layer (placeholder functions)

## Usage

These files are for **testing and reference only**. Production code should use the modules in `systems/` and `basehat/`.

## Related Branches

- `poc1` - Historical branch for initial motor control POC
- `poc2` - Historical branch for line following development
