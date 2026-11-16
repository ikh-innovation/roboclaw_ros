# Changelog

## [v1.2.0] - 2026-11-16

### Fixed
- Convert controller temperature readings from tenths of a degree to degrees Celsius
  using floating-point division.
- Release the serial lock in current and status/temperature callbacks after errors.
- Throttle serial-read error messages.

### Removed
- Remove an unused commented-out current logger.

### CI
- Add reusable ROS1 package release metadata validation for `dual_prismatic`.
