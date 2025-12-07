# Tests

This directory contains all test files for the MARCO project.

## Test Files

### `mobility_test.py`
Tests for the mobility system including:
- Motor initialization
- Obstacle detection
- Basic movement control

## Running Tests

```bash
# Run a specific test
python tests/mobility_test.py

# Run all tests (when test framework is configured)
python -m pytest tests/
```

## Writing Tests

When adding new tests:
1. Create test files with the naming convention `test_<module>.py` or `<module>_test.py`
2. Place them in this `tests/` directory
3. Document the test purpose in this README
