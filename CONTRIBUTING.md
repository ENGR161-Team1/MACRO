# Contributing to MACRO

Thank you for your interest in contributing to the Mars Autonomous Cargo Rover Operations (MACRO) project!

## Team Members

- Karley Hammond
- Advay Chandra
- Samuel Razor
- Katherine Hampton

## Branch Structure

| Branch | Purpose |
|--------|---------|
| `main` | Production-ready, stable code |
| `systems` | System module development |
| `testing` | Test files and test development |
| `documentation` | Documentation updates |

## Development Workflow

1. **Create a feature branch** from `main` or the appropriate system branch
2. **Make your changes** with clear, descriptive commits
3. **Test your changes** before pushing
4. **Push to your branch** and create a pull request
5. **Request review** from team members

## Commit Message Guidelines

Use clear, descriptive commit messages:

```
<type>: <short description>

<optional longer description>
```

Types:
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `refactor`: Code refactoring
- `test`: Adding or updating tests
- `chore`: Maintenance tasks

## Code Style

- Use Python 3.10+ features
- Follow PEP 8 style guidelines
- Add docstrings to all classes and public methods
- Use type hints where appropriate
- Use `**kwargs` for flexible function parameters

## File Naming

- Use `snake_case` for Python files (e.g., `navigation_system.py`)
- Use `PascalCase` for class names (e.g., `Location3D`)
- Keep module names descriptive and concise

## Testing

- Place test files in the `tests/` directory
- Name test files with `test_` prefix or `_test` suffix
- Run tests before submitting changes

## Questions?

Reach out to any team member for help!
