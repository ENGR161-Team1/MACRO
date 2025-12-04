# Proof of Concepts (POC)

This directory contains experimental code and proof-of-concept implementations used during MACRO development.

## Contents

### `LineFinder.py`
**Purpose**: Grove Line Finder sensor driver  
**Status**: Complete (ENGR 16X Teaching Team)  
**Description**: Official driver for the Grove Line Finder sensor. Provides:
- Digital black/white surface detection
- `value` property (0 for black, 1 for white)
- `line_detected` property (True/False for line detection)

### `proof_of_concept.py`
**Purpose**: Line following motor control demonstration  
**Status**: Complete  
**Description**: Proof-of-concept for line following with motor control featuring:
- LineFinder sensor integration
- Alternating turn direction for line reacquisition
- Motor pair control with Build HAT

## Usage

These files are for **testing and reference only**. Production code should use the modules in `systems/` and `basehat/`.

## Related Branches

- `poc1` - Historical branch for line finder POC
- `poc2` - Historical branch for additional testing
